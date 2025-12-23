from pymavlink import mavutil
import time
import threading
import math
import os

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
                        #print(self.velocity_ned['vx'], self.velocity_ned['vy'], self.velocity_ned['vz'])

            except Exception as e:
                # Тихий режим - не выводим ошибки постоянно
                pass
            time.sleep(0.05)  # Обновление каждые 50мс
    
    def get_velocity(self):
        """Получить текущую скорость."""
        with self.lock:
            return self.velocity_ned.copy()

# === Класс для получения координат дрона ===
class CoordsMonitor:
    """
    Мониторинг координат дрона через MAVLink LOCAL_POSITION_NED сообщение.
    Использует неблокирующее чтение для работы параллельно с VelocityMonitor.
    """
    def __init__(self, drone):
        self.drone = drone
        self.position_ned = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.running = False
        self.thread = None
        self.lock = threading.Lock()
    
    def start(self):
        """Запуск мониторинга координат в отдельном потоке."""
        if self.running:
            return
        self.running = True
        self.thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self.thread.start()
        print("Coordinates monitoring started")
    
    def stop(self):
        """Остановка мониторинга координат."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)
        print("Coordinates monitoring stopped")
    
    def _monitor_loop(self):
        """Основной цикл мониторинга координат."""
        while self.running:
            try:
                # Читаем сообщения о координатах LOCAL_POSITION_NED (в м)
                # Используем неблокирующее чтение для работы параллельно с VelocityMonitor
                msg = self.drone.recv_match(type='LOCAL_POSITION_NED', blocking=False, timeout=0.1)
                if msg:
                    with self.lock:
                        # В LOCAL_POSITION_NED: x, y, z - координаты в метрах (NED система координат)
                        # x - север (positive north)
                        # y - восток (positive east)
                        # z - вниз (positive down)
                        self.position_ned['x'] = msg.x
                        self.position_ned['y'] = msg.y
                        self.position_ned['z'] = msg.z

            except Exception as e:
                # Тихий режим - не выводим ошибки постоянно
                pass
            # Небольшая задержка для предотвращения перегрузки CPU
            time.sleep(0.1)
    
    def get_position(self):
        """Получить текущие координаты."""
        with self.lock:
            return self.position_ned.copy()
    
square_steps = [
    (neutral, 1700, neutral, neutral),  # Назад
    (1700, neutral, neutral, neutral),  # Вправо
    (neutral, 1300, neutral, neutral),  # Вперёд
    (1300, neutral, neutral, neutral)  # Влево
]

step_duration = 2  # Секунд на каждый сегмент квадрата

def move_with_pid_braking(drone, direction, velocity_monitor, coords_monitor, logfile,
                          kpx=2000, kpy=2000,
                          move_duration=5, target_velocity=0.0, 
                          max_brake_time=10.0, neutral_time=2.0, 
                          velocity_threshold=0.01):
    """
    Простое движение с П-регулированием для торможения.
    
    Args:
        drone: объект подключения к дрону
        direction: кортеж (roll, pitch, throttle, yaw) - направление движения
        velocity_monitor: объект VelocityMonitor для получения скорости
        coords_monitor: объект CoordsMonitor для получения координат
        logfile: файл для записи координат
        kp: пропорциональный коэффициент (по умолчанию 2000)
        move_duration: длительность движения в секундах
        target_velocity: целевая скорость (обычно 0.0)
        max_brake_time: максимальное время торможения
        neutral_time: время стабилизации после торможения
        velocity_threshold: порог скорости для остановки
    """
    roll, pitch, throttle, yaw = direction
    
    # Счётчик для записи координат
    log_iter = 0
    
    # Движение
    direction_name = "Backward" if pitch > neutral else "Forward" if pitch < neutral else \
                     "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
    print(f"Moving {direction_name}: roll={roll}, pitch={pitch}")
    
    start = time.time()
    while time.time() - start < move_duration:
        send_rc_override(drone, roll, pitch, throttle, yaw)
        
        # Записываем координаты во время движения
        log_iter += 1
        if log_iter % 10 == 0:  # Каждые 10 итераций (примерно раз в секунду)
            my_position = coords_monitor.get_position()
            logfile.write(f"{my_position}\n")
            logfile.flush()  # Принудительно записываем в файл
        
        time.sleep(0.1)
    
    # П-регулирование для торможения
    print("Starting P-control braking...")
    start_time = time.time()
    
    while time.time() - start_time < max_brake_time:
        # Записываем координаты во время торможения
        log_iter += 1
        if log_iter % 10 == 0:  # Каждые 10 итераций
            my_position = coords_monitor.get_position()
            logfile.write(f"{my_position}\n")
            logfile.flush()  # Принудительно записываем в файл
        
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
    
    # Если двигались назад (pitch > neutral), тормозим вперёд
    if pitch > neutral:
        brake_pitch = neutral - brake_intensity
    # Если двигались вперёд (pitch < neutral), тормозим назад
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
    direction_name = "Backward" if pitch > neutral else "Forward" if pitch < neutral else \
                     "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
    print(f"Moving {direction_name}: roll={roll}, pitch={pitch}")
    
    # Движение
    start = time.time()
    while time.time() - start < move_duration:
        send_rc_override(drone, roll, pitch, throttle, yaw)
        time.sleep(0.1)
    
    # Активное торможение
    brake_movement(drone, direction, brake_duration, brake_intensity)

try:
    # === Инициализация мониторинга скорости ===
    velocity_monitor = VelocityMonitor(master)
    velocity_monitor.start()
    
    # === Инициализация мониторинга координат ===
    coords_monitor = CoordsMonitor(master)
    coords_monitor.start()
    
    # === Создание директории для логов ===
    os.makedirs("logs_SQUARE", exist_ok=True)
    
    # === Создание лог-файла ===
    logfile = open("logs_SQUARE/drone_ALT_HOLD_log.txt", "w")
    print("Log file (logs_SQUARE/drone_ALT_HOLD_log.txt) was created!")
    
    # === Тестирование в режиме ALT_HOLD с PID-торможением ===
    
    print("\n=== Testing ALT_HOLD mode with PID braking ===")
    master.set_mode(2)

    # Пример использования для движения по квадрату с торможением
    while True:
        for step in square_steps:
            move_with_pid_braking(master, step, 
                             velocity_monitor, coords_monitor, logfile,
                              kpx=100, kpy=100,
                              move_duration=step_duration, target_velocity=0.0, 
                              max_brake_time=5.0, neutral_time=2.0, 
                              velocity_threshold=0.2) # для ALT_HOLD

            time.sleep(1)

except KeyboardInterrupt:
    # Останавливаем мониторинг скорости
    velocity_monitor.stop()
    
    # Останавливаем мониторинг координат
    coords_monitor.stop()
    
    # Закрываем файл лога
    if 'logfile' in locals():
        logfile.close()
        print("Log file closed")
    
    # При выходе возвращаем нейтральные RC значения
    send_rc_override(master, neutral, neutral, neutral, neutral)
    print("Setting mode LAND")
    master.set_mode(9)
    #master.arducopter_disarm()
    print("Stopped and neutralized RC channels")

