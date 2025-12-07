from pymavlink import mavutil
import time
import threading
import math

# Адреса подключения к двум дронам в сети (замените на свои)
master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

# Дождаться heartbeat
master.wait_heartbeat()
print("Connected to drone")

print("Setting mode GUIDED")
master.set_mode(4)

print(f"Arming drone {master.sysid}...")
master.arducopter_arm()
time.sleep(2)


print("Takeoff to 1 meter!")
master.mav.command_long_send(master.target_system, master.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,  # Параметр 1 - Минимальный шаг (при наличии датчика воздушной скорости), желаемый шаг без датчика
    0,  # Параметры 2-3: пустые
    0,
    0,  # Параметр 4 - YAW
    0,  # Параметр 5 - Latitude
    0,  # Параметр 6 - Longitude
    1)  # Параметр 7 - высота

time.sleep(5)

def send_rc_override(drone, chan1, chan2, chan3, chan4):
    # RC_OVERRIDE: значения каналов с 1000 (минимум) до 2000 (максимум), 1500 - нейтральное положение
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        chan1,  # roll / aileron (left/right)
        chan2,  # pitch / elevator (forward/back)
        chan3,  # throttle
        chan4,  # yaw / rudder (rotation)
        0, 0, 0, 0, 0, 0  # остальные каналы без изменений
    )

# Значения для управления квадратно
neutral = 1500

# === Класс для получения данных о скорости дрона ===
class VelocityMonitor:
    """
    Мониторинг скорости дрона через MAVLink сообщения.
    """
    def __init__(self, drone):
        self.drone = drone
        self.velocity_ned = {'vx': 0.0, 'vy': 0.0, 'vz': 0.0}
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
    
    def start(self):
        """Запуск мониторинга скорости в отдельном потоке."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        print("Velocity monitoring started")
    
    def stop(self):
        """Остановка мониторинга скорости."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        print("Velocity monitoring stopped")
    
    def _monitor_loop(self):
        """Основной цикл мониторинга скорости."""
        while self.running:
            try:
                # Читаем сообщения о скорости GLOBAL_POSITION_INT (содержит скорости в см/с)
                msg = self.drone.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
                if msg:
                    with self.lock:
                        # В GLOBAL_POSITION_INT: vx, vy - скорости в см/с
                        self.velocity_ned['vx'] = msg.vx / 100.0
                        self.velocity_ned['vy'] = msg.vy / 100.0
                        self.velocity_ned['vz'] = msg.vz / 100.0 if hasattr(msg, 'vz') else 0.0
                        print(self.velocity_ned['vx'], self.velocity_ned['vy'], self.velocity_ned['vz'])

            except Exception as e:
                # Тихий режим - не выводим ошибки постоянно
                pass
            time.sleep(0.05)  # Обновление каждые 50мс
    
    def get_velocity(self):
        """Получить текущую скорость."""
        with self.lock:
            return self.velocity_ned.copy()

    def get_horizontal_speed(self, direction):
        """
        Получить скорость в заданном направлении.
        
        Args:
            direction: кортеж (roll, pitch, throttle, yaw) - направление движения
        
        Returns:
            Скорость в направлении движения (м/с), положительная = в направлении движения
        """
        roll, pitch, throttle, yaw = direction
        with self.lock:
            vx, vy, vz = self.velocity_ned['vx'], self.velocity_ned['vy'], self.velocity_ned['vz']
        
        # Вычисляем скорость в направлении движения
        # В NED системе: vx - север (positive north), vy - восток (positive east)
        # Для движения вперёд (pitch > neutral) нужна скорость на север (vx)
        # Для движения назад (pitch < neutral) нужна скорость на юг (-vx)
        # Для движения вправо (roll > neutral) нужна скорость на восток (vy)
        # Для движения влево (roll < neutral) нужна скорость на запад (-vy)
        
        if pitch > neutral:  # Вперёд (на север)
            # Положительная скорость = движение вперёд
            return vx
        elif pitch < neutral:  # Назад (на юг)
            # Положительная скорость = движение назад (но мы хотим отрицательную для торможения)
            return -vx
        elif roll > neutral:  # Вправо (на восток)
            # Положительная скорость = движение вправо
            return vy
        elif roll < neutral:  # Влево (на запад)
            # Положительная скорость = движение влево
            return -vy
        else:
            return 0.0

square_steps = [
    (neutral, 1700, neutral, neutral),  # Вперёд
    (1700, neutral, neutral, neutral),  # Вправо
    (neutral, 1300, neutral, neutral),  # Назад
    (1300, neutral, neutral, neutral)  # Влево
]

step_duration = 2  # Секунд на каждый сегмент квадрата

def move_with_pid_braking(drone, direction, velocity_monitor, kpx=2000, kpy=2000,
                          move_duration=5, target_velocity=0.0, 
                          max_brake_time=10.0, neutral_time=2.0, 
                          velocity_threshold=0.01):
    """
    Простое движение с П-регулированием для торможения.
    
    Args:
        drone: объект подключения к дрону
        direction: кортеж (roll, pitch, throttle, yaw) - направление движения
        velocity_monitor: объект VelocityMonitor для получения скорости
        kp: пропорциональный коэффициент (по умолчанию 2000)
        move_duration: длительность движения в секундах
        target_velocity: целевая скорость (обычно 0.0)
        max_brake_time: максимальное время торможения
        neutral_time: время стабилизации после торможения
        velocity_threshold: порог скорости для остановки
    """
    roll, pitch, throttle, yaw = direction
    
    # Движение
    direction_name = "Forward" if pitch > neutral else "Backward" if pitch < neutral else \
                     "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
    print(f"Moving {direction_name}: roll={roll}, pitch={pitch}")
    
    start = time.time()
    while time.time() - start < move_duration:
        send_rc_override(drone, roll, pitch, throttle, yaw)
        time.sleep(0.1)
    
    # П-регулирование для торможения
    print("Starting P-control braking...")
    start_time = time.time()
    
    while time.time() - start_time < max_brake_time:
        # Получаем текущую скорость
        current_velocity = velocity_monitor.get_velocity()
        current_x_velocity = current_velocity['vx']
        current_y_velocity = current_velocity['vy']
        
        print(f"Current velocities: vx={current_x_velocity:.3f} m/s, vy={current_y_velocity:.3f} m/s")
        
        # Если скорость достаточно мала, прекращаем
        if abs(current_x_velocity) < velocity_threshold and abs(current_y_velocity) < velocity_threshold:
            print(f"!!!!!!!!!!!!!Velocities threshold reached")#: vx={current_x_velocity:.3f} m/s, vy={current_y_velocity:.3f} m/s")
            break
        
        # П-регулирование: ошибка * коэффициент = интенсивность торможения
        error_x = target_velocity - current_x_velocity
        error_y = target_velocity - current_y_velocity
        
        # Вычисляем интенсивность торможения для каждого канала
        brake_intensity_x = 0
        brake_intensity_y = 0
        
        if kpx > 0:
            brake_intensity_x = int(abs(error_x) * kpx)
            brake_intensity_x = min(500, max(50, brake_intensity_x))
            # Направление торможения по X: если error_x > 0 (движение на восток), тормозим на запад (отрицательное)
            brake_direction_x = -1 if error_x > 0 else 1
            brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
        else:
            brake_pitch = neutral  # Не применяем торможение по pitch
        
        if kpy > 0:
            brake_intensity_y = int(abs(error_y) * kpy)
            brake_intensity_y = min(500, max(50, brake_intensity_y))
            # Направление торможения по Y: если error_y > 0 (движение на север), тормозим на юг (отрицательное)
            brake_direction_y = 1 if error_y > 0 else -1
            brake_roll = neutral + (brake_direction_y * brake_intensity_y)
        else:
            brake_roll = neutral  # Не применяем торможение по roll
        
        # Применяем торможение
        send_rc_override(drone, brake_roll, brake_pitch, neutral, neutral)
        
        print(f"Braking: error_x={error_x:.3f}, error_y={error_y:.3f},"
               f"brake_x={brake_intensity_x}, brake_y={brake_intensity_y}"
               f"pitch={brake_pitch}, roll={brake_roll}")
        
        #print(f"Braking: vel={current_velocity:.3f} m/s, error={error:.3f}, "f"brake_int={brake_intensity}")
        
        time.sleep(0.05)
    
    # # Стабилизация
    # print("Stabilizing...")
    # start = time.time()
    # while time.time() - start < neutral_time:
    #     send_rc_override(drone, neutral, neutral, neutral, neutral)
    #     time.sleep(0.1)


def brake_movement(drone, direction, brake_duration=0.5, brake_intensity=200):
    """
    Активное торможение дрона после движения в заданном направлении (статическое).
    
    Args:
        drone: объект подключения к дрону
        direction: кортеж (roll, pitch, throttle, yaw) - направление движения
        brake_duration: длительность торможения в секундах
        brake_intensity: интенсивность торможения (отклонение от нейтрального значения)
    """
    roll, pitch, throttle, yaw = direction
    
    # Определяем противоположное направление для торможения
    brake_roll = neutral
    brake_pitch = neutral
    brake_throttle = neutral
    brake_yaw = neutral
    
    # Если двигались вперёд (pitch > neutral), тормозим назад
    if pitch > neutral:
        brake_pitch = neutral - brake_intensity
    # Если двигались назад (pitch < neutral), тормозим вперёд
    elif pitch < neutral:
        brake_pitch = neutral + brake_intensity
    
    # Если двигались вправо (roll > neutral), тормозим влево
    if roll > neutral:
        brake_roll = neutral - brake_intensity
    # Если двигались влево (roll < neutral), тормозим вправо
    elif roll < neutral:
        brake_roll = neutral + brake_intensity
    
    # Применяем торможение
    print(f"Braking: roll={brake_roll}, pitch={brake_pitch}")
    start = time.time()
    while time.time() - start < brake_duration:
        send_rc_override(drone, brake_roll, brake_pitch, brake_throttle, brake_yaw)
        time.sleep(0.1)

def move_with_braking(drone, direction, move_duration=5, brake_duration=0.5, brake_intensity=200):
    """
    Движение дрона с активным торможением (статическое).
    
    Args:
        drone: объект подключения к дрону
        direction: кортеж (roll, pitch, throttle, yaw) - направление движения
        move_duration: длительность движения в секундах
        brake_duration: длительность торможения в секундах
        brake_intensity: интенсивность торможения
    """
    
    roll, pitch, throttle, yaw = direction
    direction_name = "Forward" if pitch > neutral else "Backward" if pitch < neutral else \
                     "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
    print(f"Moving {direction_name}: roll={roll}, pitch={pitch}")
    
    # Движение
    start = time.time()
    while time.time() - start < move_duration:
        send_rc_override(drone, roll, pitch, throttle, yaw)
        time.sleep(0.1)
    
    # Активное торможение
    brake_movement(drone, direction, brake_duration, brake_intensity)

'''
def set_param(drone, param_name, param_value, param_type=9):
    """
    Установка параметра ArduPilot через MAVLink.
    
    Args:
        drone: объект подключения к дрону
        param_name: имя параметра (например, "LOIT_BRK_ACCEL")
        param_value: значение параметра
        param_type: тип параметра (9 = float32, по умолчанию)
    """
    try:
        # Сначала запрашиваем текущее значение параметра
        drone.mav.param_request_read_send(
            drone.target_system,
            drone.target_component,
            param_name.encode('utf-8'),
            -1
        )
        time.sleep(0.2)
        
        # Отправляем запрос на установку параметра
        drone.mav.param_set_send(
            drone.target_system,
            drone.target_component,
            param_name.encode('utf-8'),
            param_value,
            param_type
        )
        print(f"Setting parameter {param_name} = {param_value}")
        
        # Ждём подтверждения - читаем все сообщения PARAM_VALUE
        timeout = 3.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            # Обрабатываем все входящие сообщения
            while True:
                msg = drone.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.05)
                if msg is None:
                    break
                
                # Проверяем имя параметра (нужно правильно декодировать)
                try:
                    param_id = msg.param_id
                    if isinstance(param_id, bytes):
                        param_id_str = param_id.decode('utf-8', errors='ignore').rstrip('\x00').rstrip()
                    else:
                        param_id_str = str(param_id).rstrip('\x00').rstrip()
                    
                    if param_id_str == param_name:
                        print(f"✓ Parameter {param_name} confirmed: {msg.param_value}")
                        return True
                except Exception as e:
                    pass
            
            time.sleep(0.1)
        
        # Если не получили подтверждение, проверяем параметр вручную
        print(f"Warning: No confirmation for {param_name}, checking manually...")
        drone.mav.param_request_read_send(
            drone.target_system,
            drone.target_component,
            param_name.encode('utf-8'),
            -1
        )
        time.sleep(0.5)
        
        # Проверяем ответ
        msg = drone.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.0)
        if msg:
            try:
                param_id = msg.param_id
                if isinstance(param_id, bytes):
                    param_id_str = param_id.decode('utf-8', errors='ignore').rstrip('\x00').rstrip()
                else:
                    param_id_str = str(param_id).rstrip('\x00').rstrip()
                
                if param_id_str == param_name:
                    if abs(msg.param_value - param_value) < 0.01:
                        print(f"✓ Parameter {param_name} verified: {msg.param_value}")
                        return True
                    else:
                        print(f"⚠ Parameter {param_name} value mismatch: expected {param_value}, got {msg.param_value}")
            except:
                pass
        
        print(f"✗ Failed to set parameter {param_name}")
        return False
    except Exception as e:
        print(f"Error setting parameter {param_name}: {e}")
        return False
'''
try:
    # === Инициализация мониторинга скорости ===
    velocity_monitor = VelocityMonitor(master)
    velocity_monitor.start()
    
    '''
    # === Установка параметров для быстрого торможения в LOITER/POS_HOLD ===
    # LOITER - современная замена POS_HOLD, использует те же параметры
    print("\n=== Setting Loiter/POS_HOLD braking parameters ===")
    print("Note: Parameters should be loaded from iris.parm file on startup.")
    print("If they're not applied, we'll set them programmatically.\n")
    
    # Список параметров для установки (снижены для уменьшения качаний)
    params_to_set = [
        ("LOIT_BRK_ACCEL", 800.0),    # Ускорение торможения (см/с²) = 8 м/с² (было 2000)
        ("LOIT_ACC_MAX", 1000.0),     # Максимальное ускорение (см/с²) = 10 м/с² (было 2500)
        ("ANGLE_MAX", 7000.0),        # Максимальный угол наклона (сантиградусы) = 70° (было 4500 = 45°)
        ("PSC_VELXY_P", 1.2),         # Коэффициент скорости (было 3.5)
        ("PSC_VELXY_I", 0.15),        # Интегральный коэффициент (было 0.5)
        ("PSC_VELXY_D", 0.08),        # Дифференциальный коэффициент (было 0.25)
        ("PSC_POSXY_P", 1.0),         # Коэффициент позиции (было 2.5)
        ("PSC_JERK_XY", 10.0),        # Jerk (было 30.0)
    ]
    
    # Устанавливаем параметры
    for param_name, param_value in params_to_set:
        success = set_param(master, param_name, param_value)
        if not success:
            print(f"⚠ Could not confirm {param_name}, but it may still be set. Check with 'param show {param_name}'")
        time.sleep(0.3)  # Небольшая задержка между параметрами
    
    print("\n=== Parameter setup complete ===")
    print("You can verify parameters in MAVProxy with: param show LOIT_BRK_ACCEL")
    time.sleep(1)
    '''

    # === Тестирование в режиме LOITER (современная замена POS_HOLD) ===
    '''
    LOITER режим - современная замена POS_HOLD, использует встроенный PID-регулятор ArduPilot
    Параметры настраиваются в файле config/iris.parm (PSC_*, LOIT_*, PHLD_* параметры)
    Режим LOITER (5) рекомендуется вместо POS_HOLD (16)
    '''

    # print("\n=== Testing POS_HOLD mode ===")
    # # LOITER mode = 5, POS_HOLD mode = 16 (устаревший)
    # master.set_mode(16)  # POS_HOLD
    
    # while True:
    #     # Вперёд с торможением (статическое, т.к. POS_HOLD уже имеет встроенный PID)
    #     move_with_braking(master, (neutral, 1700, neutral, neutral), 
    #                      move_duration=3, brake_duration=3, brake_intensity=0)
    #     time.sleep(1)
        
    #     # Назад с торможением
    #     move_with_braking(master, (neutral, 1300, neutral, neutral), 
    #                      move_duration=3, brake_duration=2, brake_intensity=0)
    #     time.sleep(1)
    
    # === Тестирование в режиме ALT_HOLD с PID-торможением ===
    
    print("\n=== Testing ALT_HOLD mode with PID braking ===")
    master.set_mode(2)

    # while True:
    #     # Вперёд с П-регулированием
    #     move_with_pid_braking(master, (neutral, 1700, neutral, neutral), 
    #                          velocity_monitor, kpx=1800, kpy=1800,
    #                          move_duration=5, target_velocity=0.0, 
    #                          max_brake_time=3.0, neutral_time=2.0, 
    #                          velocity_threshold=0.8)
    #     time.sleep(1)
        
    #     # Назад с П-регулированием
    #     move_with_pid_braking(master, (neutral, 1300, neutral, neutral), 
    #                          velocity_monitor, kpx=1800, kpy=1800,
    #                          move_duration=5, target_velocity=0.0, 
    #                          max_brake_time=3.0, neutral_time=2.0, 
    #                          velocity_threshold=0.8)
    #     time.sleep(1)
    

    # Пример использования для движения по квадрату с торможением
    while True:
        for step in square_steps:
            move_with_pid_braking(master, step, 
                             velocity_monitor, kpx=100, kpy=100,
                              move_duration=step_duration, target_velocity=0.0, 
                              max_brake_time=5.0, neutral_time=2.0, 
                              velocity_threshold=0.8) # для ALT_HOLD

            # move_with_braking(master, step, 
            #                  move_duration=step_duration, 
            #                  brake_duration=5, 
            #                  brake_intensity=0) # 0 - для POS_HOLD
            time.sleep(1)

except KeyboardInterrupt:
    # Останавливаем мониторинг скорости
    velocity_monitor.stop()
    
    # При выходе возвращаем нейтральные RC значения
    send_rc_override(master, neutral, neutral, neutral, neutral)
    print("Setting mode LAND")
    master.set_mode(9)
    #master.arducopter_disarm()
    print("Stopped and neutralized RC channels")
