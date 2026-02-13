from pymavlink import mavutil
import time
import threading
from datetime import datetime
#from RC_Following.CoordsMonitor import CoordsMonitor
from RC_Following.VelocityMonitor import VelocityMonitor

# Конфигурация дронов
DRONES_CONFIG = [
    {
        'id': 1,
        'udp_port': 14551,
        'role': 'square'  # Первый дрон
    },
    {
        'id': 2,
        'udp_port': 14561,
        'role': 'square'  # Второй дрон
    },
    {
        'id': 3,
        'udp_port': 14571,
        'role': 'square'  # Второй дрон
    },
    {
        'id': 4,
        'udp_port': 14581,
        'role': 'square'  # Второй дрон
    },
    {
        'id': 5,
        'udp_port': 14591,
        'role': 'square'  # Второй дрон
    }
]

# Значения для управления
neutral = 1500

# Шаги для движения по квадрату
square_steps = [
    (neutral, 1700, neutral, neutral),  # Назад
    (1700, neutral, neutral, neutral),  # Вправо
    (neutral, 1300, neutral, neutral),  # Вперёд
    (1300, neutral, neutral, neutral)  # Влево
]

step_duration = 2  # Секунд на каждый сегмент квадрата

def send_rc_override(drone, chan1, chan2, chan3, chan4, controller=None):
    """
    Отправка RC_OVERRIDE команды дрону.
    
    Args:
        drone: объект MAVLink соединения
        chan1: roll / aileron (left/right)
        chan2: pitch / elevator (forward/back)
        chan3: throttle
        chan4: yaw / rudder (rotation)
        controller: объект DroneController (опционально, для обновления последних значений)
    """
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        chan1,  # roll / aileron (left/right)
        chan2,  # pitch / elevator (forward/back)
        chan3,  # throttle
        chan4,  # yaw / rudder (rotation)
        0, 0, 0, 0, 0, 0  # остальные каналы без изменений
    )
    
    # Обновляем последние значения для keepalive (если передан контроллер)
    if controller is not None:
        with controller.rc_channels_lock:
            controller.last_rc_channels['roll'] = chan1
            controller.last_rc_channels['pitch'] = chan2
            controller.last_rc_channels['throttle'] = chan3
            controller.last_rc_channels['yaw'] = chan4

class PIDRegulator:
    """
    PID-регулятор с поддержкой пропорциональной, интегральной и дифференциальной составляющих.
    Включает механизм anti-windup для предотвращения переполнения интегральной составляющей.
    """
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, integral_limit=1000.0, output_limit=500.0):
        """
        Инициализация PID-регулятора.
        
        Args:
            kp: пропорциональный коэффициент
            ki: интегральный коэффициент
            kd: дифференциальный коэффициент
            integral_limit: максимальное значение интегральной составляющей (anti-windup)
            output_limit: максимальное значение выходного сигнала
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def update(self, error, dt=None):
        """
        Обновление регулятора с новой ошибкой.
        
        Args:
            error: текущая ошибка
            dt: временной интервал (если None, вычисляется автоматически)
        
        Returns:
            Выходное значение регулятора
        """
        current_time = time.time()
        
        if dt is None:
            if self.last_time is None:
                dt = 0.1  # Первый вызов - используем стандартный интервал
            else:
                dt = current_time - self.last_time
                if dt <= 0:
                    dt = 0.1  # Защита от нулевого или отрицательного интервала
        
        self.last_time = current_time
        
        # Пропорциональная составляющая
        p_term = self.kp * error
        
        # Интегральная составляющая с anti-windup
        self.integral += error * dt
        # Ограничение интегральной составляющей (anti-windup)
        if self.integral_limit > 0:
            self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        i_term = self.ki * self.integral
        
        # Дифференциальная составляющая
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        d_term = self.kd * derivative
        
        self.last_error = error
        
        # Суммарный выход
        output = p_term + i_term + d_term
        
        # Ограничение выходного сигнала
        output = max(-self.output_limit, min(self.output_limit, output))
        
        return output
    
    def reset(self):
        """Сброс состояния регулятора (интегральной и дифференциальной составляющих)."""
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
    
    def set_integral(self, value):
        """Установка значения интегральной составляющей."""
        if self.integral_limit > 0:
            self.integral = max(-self.integral_limit, min(self.integral_limit, value))
        else:
            self.integral = value

class DroneController:
    """Класс для управления одним дроном."""
    def __init__(self, config, logging=False):
        self.config = config
        self.master = None
        #self.coords_monitor = None
        self.velocity_monitor = None
        self.lock = threading.Lock()

        self.logging = logging  # ЛОГИРОВАНИЕ КООРДИНАТ

        if self.logging:
            self.logIter = 0
            self.logfile = open(f"logs_SQUARE_two_drones/drone_{self.config['id']}_log.txt", "w")
        
        # PID-регуляторы для управления углами (торможение)
        # Roll регулятор (для компенсации дрейфа влево-вправо)
        self.roll_velocity_pid = PIDRegulator(kp=100.0, ki=100.0, kd=0.0, 
                                               integral_limit=200.0, output_limit=500.0)
        # Pitch регулятор (для компенсации дрейфа вперёд-назад)
        self.pitch_velocity_pid = PIDRegulator(kp=100.0, ki=0.0, kd=0.0,
                                                integral_limit=200.0, output_limit=500.0)
        
        # Флаг для отслеживания режима движения (для сброса интегральной составляющей при смене режима)
        self.last_movement_mode = None
        
        # Последние значения RC каналов для keepalive (чтобы не конфликтовать с активным управлением)
        self.last_rc_channels = {
            'roll': neutral,
            'pitch': neutral,
            'throttle': neutral,
            'yaw': neutral
        }
        self.rc_channels_lock = threading.Lock()
    
    def connect(self):
        """Подключение к дрону."""
        print(f"Connecting to drone {self.config['id']} on UDP port {self.config['udp_port']}...")
        self.master = mavutil.mavlink_connection(f'udp:127.0.0.1:{self.config["udp_port"]}')
        self.master.wait_heartbeat()
        print(f"Connected to drone {self.config['id']}")
        
        # Инициализация мониторов
        #self.coords_monitor = CoordsMonitor(self.master)
        self.velocity_monitor = VelocityMonitor(self.master)
    
    def initialize(self):
        """Инициализация дрона (взлёт)."""
        print(f"[Drone {self.config['id']}] Setting mode GUIDED")
        self.master.set_mode(4)  # GUIDED
        time.sleep(0.5)
        
        print(f"[Drone {self.config['id']}] Arming...")
        self.master.arducopter_arm()
        time.sleep(2)
        
        print(f"[Drone {self.config['id']}] Takeoff to 1 meter!")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, 1)  # Высота 1 метр
        
        time.sleep(5)

        print(f"[Drone {self.config['id']}] Setting mode ALT_HOLD")
        self.master.set_mode(2)  # ALT_HOLD
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        time.sleep(0.5)
        
        # Запуск мониторинга
        print(f"[Drone {self.config['id']}] Starting monitors")
        #self.coords_monitor.start()
        self.velocity_monitor.start()
    
    def start_rc_keepalive(self):
        """
        Запуск потока для непрерывной отправки RC_OVERRIDE.
        Это предотвращает истечение RC_OVERRIDE_TIME (3 секунды по умолчанию).
        Keepalive отправляет последние актуальные значения RC каналов, чтобы не конфликтовать
        с активным управлением.
        """
        def keepalive_loop():
            while True:
                try:
                    # Получаем последние актуальные значения RC каналов
                    with self.rc_channels_lock:
                        roll = self.last_rc_channels['roll']
                        pitch = self.last_rc_channels['pitch']
                        throttle = self.last_rc_channels['throttle']
                        yaw = self.last_rc_channels['yaw']
                    
                    # Отправляем последние значения для поддержания связи
                    # Это предотвращает истечение RC_OVERRIDE_TIME и не конфликтует с активным управлением
                    send_rc_override(self.master, roll, pitch, throttle, yaw)
                    time.sleep(0.2)  # Отправляем каждые 200мс (5 раз в секунду, достаточно для 3 сек таймаута)
                except Exception as e:
                    print(f"[Drone {self.config['id']}] Error in RC keepalive: {e}")
                    break
        
        keepalive_thread = threading.Thread(target=keepalive_loop, daemon=True)
        keepalive_thread.start()
        print(f"[Drone {self.config['id']}] RC keepalive thread started")
    
    # def get_my_position(self):
    #     """Получить свою позицию."""
    #     my_position = self.coords_monitor.get_position()
    #     if self.logging:
    #         self.logIter += 1
    #         if self.logIter % 20 == 0:
    #             self.logfile.write(f"{my_position}\n")
    #     return my_position
    
    def move_with_pid_braking(self, direction, kpx=2000, kpy=2000,
                              move_duration=5, target_velocity=0.0, 
                              max_brake_time=10.0, velocity_threshold=0.8,
                              use_pid=True):
        """
        Движение с PID-регулированием для торможения.
        
        Args:
            direction: кортеж (roll, pitch, throttle, yaw) - направление движения
            kpx: пропорциональный коэффициент для оси X (восток-запад) - используется только если use_pid=False
            kpy: пропорциональный коэффициент для оси Y (север-юг) - используется только если use_pid=False
            move_duration: длительность движения в секундах
            target_velocity: целевая скорость (обычно 0.0)
            max_brake_time: максимальное время торможения
            velocity_threshold: порог скорости для остановки
            use_pid: использовать ли PID-регулятор (True) или только П-регулятор (False)
        """
        roll, pitch, throttle, yaw = direction
        
        # Сброс интегральной составляющей при начале нового движения
        if self.last_movement_mode != 'moving':
            self.roll_velocity_pid.reset()
            self.pitch_velocity_pid.reset()
        self.last_movement_mode = 'moving'
        
        # Движение
        direction_name = "Backward" if pitch > neutral else "Forward" if pitch < neutral else \
                         "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
        print(f"[Drone {self.config['id']}] Moving {direction_name}: roll={roll}, pitch={pitch}")
        
        self.logfile.write(f"Движение {direction_name}: roll={roll}, pitch={pitch}\n")

        start = time.time()
        while time.time() - start < move_duration:
            send_rc_override(self.master, roll, pitch, throttle, yaw, controller=self)
            time.sleep(0.1)
        
        # Переход в режим торможения
        self.last_movement_mode = 'braking'
        
        # PID-регулирование для торможения
        print(f"[Drone {self.config['id']}] Starting P-control braking...")
        start_time = time.time()
        last_update_time = time.time()  # Для I-составляющей PID-регулятора
        
        while time.time() - start_time < max_brake_time:
            current_time = time.time()
            dt = current_time - last_update_time
            if dt <= 0:
                dt = 0.05  # Минимальный интервал
            last_update_time = current_time
            
            # Получаем текущую скорость
            current_velocity = self.velocity_monitor.get_velocity()
            # В NED: vx = север, vy = восток
            current_x_velocity = current_velocity['vx']  # Скорость на север
            current_y_velocity = current_velocity['vy']   # Скорость на восток
            
            # Если скорость достаточно мала, прекращаем
            if abs(current_x_velocity) < velocity_threshold and abs(current_y_velocity) < velocity_threshold:
                print(f"[Drone {self.config['id']}] Velocities threshold reached: vx={current_x_velocity:.3f} m/s, vy={current_y_velocity:.3f} m/s")
                # Сбрасываем интегральную составляющую при достижении цели
                self.roll_velocity_pid.reset()
                self.pitch_velocity_pid.reset()
                break
            
            # Ошибки скорости
            error_vx = target_velocity - current_x_velocity  # Ошибка скорости на север
            error_vy = target_velocity - current_y_velocity    # Ошибка скорости на восток
            
            if use_pid:
                # Используем PID-регуляторы
                # Торможение по северу-югу управляет pitch (вперёд-назад)
                brake_output_pitch = self.pitch_velocity_pid.update(error_vx, dt)
                brake_intensity_x = int(abs(brake_output_pitch))
                brake_intensity_x = min(500, max(50, brake_intensity_x))
                # Направление торможения: если error_vx > 0 (движение на север), тормозим на юг
                brake_direction_x = -1 if error_vx > 0 else 1
                brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
            
                # Торможение по востоку-западу управляет roll (влево-вправо)
                brake_output_roll = self.roll_velocity_pid.update(error_vy, dt)
                brake_intensity_y = int(abs(brake_output_roll))
                brake_intensity_y = min(500, max(50, brake_intensity_y))
                # Направление торможения: если error_vy > 0 (движение на восток), тормозим на запад
                brake_direction_y = 1 if error_vy > 0 else -1
                brake_roll = neutral + (brake_direction_y * brake_intensity_y)
            else:
                # Старый метод с только П-регулятором (для обратной совместимости)
                # Торможение по северу-югу управляет pitch (вперёд-назад)
                if kpx > 0:
                    brake_intensity_x = int(abs(error_vx) * kpx)
                    brake_intensity_x = min(500, max(50, brake_intensity_x))
                    brake_direction_x = 1 if error_vx > 0 else -1
                    brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
                else:
                    brake_pitch = neutral
                
                # Торможение по востоку-западу управляет roll (влево-вправо)
                if kpy > 0:
                    brake_intensity_y = int(abs(error_vy) * kpy)
                    brake_intensity_y = min(500, max(50, brake_intensity_y))
                    brake_direction_y = -1 if error_vy > 0 else 1
                    brake_roll = neutral + (brake_direction_y * brake_intensity_y)
                else:
                    brake_roll = neutral
            
            # Применяем торможение
            send_rc_override(self.master, brake_roll, brake_pitch, neutral, neutral, controller=self)
            
            time.sleep(0.05)  # ЗАДАЁТ dt ДЛЯ ИНТЕГРАЛЬНОЙ СОСТАВЛЯЮЩЕЙ!!!
    
    def stop(self):
        """Остановка дрона."""
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        #self.coords_monitor.stop()
        self.velocity_monitor.stop()
        if self.logging:
            self.logfile.close()
        print(f"[Drone {self.config['id']}] Setting mode LAND")
        self.master.set_mode(9)

def square_pattern(controller, step_duration=2, kpx=100, kpy=100, 
                   use_pid=True, velocity_threshold=0.2):
    """
    Паттерн движения по квадрату для дрона с PID-регулятором торможения.
    
    Args:
        controller: объект DroneController
        step_duration: длительность движения на каждом сегменте квадрата
        kpx: пропорциональный коэффициент для торможения по X (восток-запад) - используется только если use_pid=False
        kpy: пропорциональный коэффициент для торможения по Y (север-юг) - используется только если use_pid=False
        use_pid: использовать ли PID-регулятор (True) или только П-регулятор (False)
        velocity_threshold: порог скорости для остановки
    """
    if use_pid:
        print(f"[Drone {controller.config['id']}] Starting square pattern with PID-control braking")
    else:
        print(f"[Drone {controller.config['id']}] Starting square pattern with P-control braking")
    
    while True:
        for step in square_steps:
            controller.move_with_pid_braking(
                step,
                kpx=kpx, kpy=kpy,
                move_duration=step_duration,
                target_velocity=0.0,
                max_brake_time=5.0,
                velocity_threshold=velocity_threshold,
                use_pid=use_pid
            )
            
            # Пауза после торможения
            send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)
            time.sleep(1)

def initialize_drone_parallel(controller, init_barrier):
    """
    Инициализация одного дрона в отдельном потоке.
    
    Args:
        controller: объект DroneController
        init_barrier: threading.Barrier для синхронизации всех дронов
    """
    try:
        print(f"[Drone {controller.config['id']}] Thread started")
        controller.connect()
        controller.initialize()
        
        # Запускаем RC keepalive для предотвращения истечения RC_OVERRIDE_TIME
        # Это критически важно - без этого дрон упадёт через 3 секунды после последней команды
        controller.start_rc_keepalive()
        
        print(f"[Drone {controller.config['id']}] Initialization complete, waiting for others...")
        
        # Ждём, пока все дроны завершат инициализацию
        init_barrier.wait()
        print(f"[Drone {controller.config['id']}] All drones ready!")
        
    except Exception as e:
        print(f"[Drone {controller.config['id']}] Error in initialization: {e}")
        import traceback
        traceback.print_exc()

def main():
    """Основная функция."""
    print("=== Two Drones Square Pattern System ===")
    
    # Создаём контроллеры для всех дронов (только объекты, без подключения)
    controllers = []
    for config in DRONES_CONFIG:
        controller = DroneController(config, logging=True)  # Включаем логирование координат
        controllers.append(controller)
    
    # Создаём барьер для синхронизации инициализации всех дронов
    init_barrier = threading.Barrier(len(controllers) + 1)  # +1 для основного потока
    
    # Запускаем инициализацию всех дронов параллельно
    init_threads = []
    for controller in controllers:
        thread = threading.Thread(
            target=initialize_drone_parallel,
            args=(controller, init_barrier),
            daemon=False  # Не daemon, чтобы дождаться завершения инициализации
        )
        thread.start()
        init_threads.append(thread)
    
    print(f"Started {len(init_threads)} initialization threads")
    
    # Ждём, пока все дроны завершат инициализацию
    print("Waiting for all drones to initialize...")
    try:
        init_barrier.wait(timeout=30)  # Таймаут 30 секунд
        print("=== All drones initialized successfully! ===")
    except threading.BrokenBarrierError:
        print("ERROR: Barrier broken, some drones failed to initialize")
        return
    
    # Ждём завершения всех потоков инициализации
    for thread in init_threads:
        thread.join(timeout=1)
    
    print("=== Даём время на стабилизацию ===")
    time.sleep(2)  # Даём время на стабилизацию
    
    try:
        # Запускаем паттерн квадрата для каждого дрона в отдельном потоке
        square_threads = []
        for controller in controllers:
            thread = threading.Thread(
                target=square_pattern,
                args=(controller, step_duration, 100, 100, True),  # step_duration, kpx, kpy, use_pid
                kwargs={'velocity_threshold': 0.2},  # для ALT_HOLD
                daemon=True
            )
            thread.start()
            square_threads.append(thread)
            print(f"[Drone {controller.config['id']}] Square pattern thread started with PID control")
        
        while True:
            square1_velocity = controllers[0].velocity_monitor.get_velocity()
            square2_velocity = controllers[1].velocity_monitor.get_velocity()
            square3_velocity = controllers[2].velocity_monitor.get_velocity()
            square4_velocity = controllers[3].velocity_monitor.get_velocity()
            square5_velocity = controllers[4].velocity_monitor.get_velocity()
            # print(f"Leader velocity: {leader_velocity}")
            # print(f"Follower velocity: {follower_velocity}")
            now = datetime.now()
            time_stamp = f"{now.second:02d}.{now.microsecond // 1000:03d}"
            controllers[0].logfile.write(f"'t': {time_stamp}, {square1_velocity}\n")
            controllers[1].logfile.write(f"'t': {time_stamp}, {square2_velocity}\n")
            controllers[2].logfile.write(f"'t': {time_stamp}, {square3_velocity}\n")
            controllers[3].logfile.write(f"'t': {time_stamp}, {square4_velocity}\n")
            controllers[4].logfile.write(f"'t': {time_stamp}, {square5_velocity}\n")
            time.sleep(0.0025)

        # Ждём завершения всех потоков (они работают бесконечно, пока не будет KeyboardInterrupt)
        for thread in square_threads:
            thread.join()
        
    except KeyboardInterrupt:
        print("\nStopping all drones...")
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)  # LAND
        print("All drones stopped")

if __name__ == "__main__":
    main()
