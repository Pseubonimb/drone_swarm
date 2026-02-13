'''
# Оба дрона летают в POS_HOLD
# Два монитора CoordsMonitor
# Один регулятор преследования
'''

from pymavlink import mavutil
import time
import threading
from CoordsMonitor import CoordsMonitor
from VelocityMonitor import VelocityMonitor

# Конфигурация дронов
DRONES_CONFIG = [
    {
        'id': 1,
        'udp_port': 14551,
        'role': 'leader'  # Дрон, который летает вперёд-назад
    },
    {
        'id': 2,
        'udp_port': 14561,
        'role': 'follower'  # Дрон, который преследует
    }
]

# Значения для управления
neutral = 1500

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
        
        if dt is None: # Ещё желательно синхронизировать временной интервал с шагом времени симуляции Webots
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

class DroneController:
    """Класс для управления одним дроном."""
    def __init__(self, config, logging=False):
        self.config = config
        self.master = None
        self.coords_monitor = None
        self.velocity_monitor = None
        self.other_drones_positions = {}  # Словарь для хранения позиций других дронов
        self.lock = threading.Lock()

        self.logging = logging # ЛОГИРОВАНИЕ КООРДИНАТ

        if self.logging:
            self.logIter = 0
            self.logfile = open(f"logs/drone_{self.config['id']}_log.txt", "w")
        
        # PID-регуляторы для управления позицией (движение к цели)
        # Roll позиционный регулятор
        self.roll_position_pid = PIDRegulator(kp=100.0, ki=10.0, kd=0.0,
                                               integral_limit=100.0, output_limit=200.0)
        # Pitch позиционный регулятор
        self.pitch_position_pid = PIDRegulator(kp=100.0, ki=10.0, kd=0.0,
                                                integral_limit=100.0, output_limit=200.0)
        
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
        self.coords_monitor = CoordsMonitor(self.master)
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

        print(f"[Drone {self.config['id']}] Setting mode POS_HOLD")
        self.master.set_mode(16)  # POS_HOLD
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        time.sleep(0.5)
        
        # Запуск мониторинга
        print(f"[Drone {self.config['id']}] Starting monitors")
        self.coords_monitor.start()
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
    
    def update_other_drone_position(self, drone_id, position):
        """Обновить позицию другого дрона."""
        with self.lock:
            self.other_drones_positions[drone_id] = position
    
    def get_my_position(self):
        """Получить свою позицию."""
        my_position = self.coords_monitor.get_position()
        if self.logging:
            self.logIter += 1
            if self.logIter % 20 == 0:
                self.logfile.write(f"{my_position}\n")
        return my_position
    
    def get_distance_to_drone(self, target_drone_id):
        """Получить расстояние до целевого дрона."""
        if target_drone_id not in self.other_drones_positions:
            return None
        return self.coords_monitor.get_distance_to(self.other_drones_positions[target_drone_id])
    
    def get_relative_position_to_drone(self, target_drone_id):
        """Получить относительную позицию относительно целевого дрона."""
        if target_drone_id not in self.other_drones_positions:
            return None
        return self.coords_monitor.get_relative_position(self.other_drones_positions[target_drone_id])
    
    def move_towards(self, target_position, kp=500.0):
        """
        Движение к целевой позиции с P-регулированием
        """
        
        rel_pos = self.coords_monitor.get_relative_position(target_position)
        
        # Вычисляем расстояние до цели
        # В NED: x = север, y = восток
        error_x = rel_pos['x']  # Ошибка по северу-югу (x в NED)
        error_y = rel_pos['y'] - 2 # Ошибка по востоку-западу (y в NED)
                                    # Смещаем на 2 метра вправо    
        
                                    
        distance = (error_x**2 + error_y**2)**0.5
        
        
        # Если далеко от цели - двигаемся к ней

        # Старый метод с только П-регулятором (для обратной совместимости)
        # П-регулирование
        # error_x управляет pitch (вперёд-назад), error_y управляет roll (влево-вправо)
        intensity_x = int(abs(error_x) * kp)
        intensity_y = int(abs(error_y) * kp)
        
        # Ограничиваем интенсивность
        intensity_x = min(200, intensity_x)
        intensity_y = min(200, intensity_y)
        
        # Определяем направление
        # Roll управляет движением влево-вправо (восток-запад)
        if abs(error_y) > 0.1:  # Порог
            if error_y > 0:  # Цель западнее - двигаемся на запад (влево)
                roll = neutral + intensity_y
            else:  # Цель восточнее - двигаемся на восток (вправо)
                roll = neutral - intensity_y
        else:
            roll = neutral
        
        # Pitch управляет движением вперёд-назад (север-юг)
        # ВАЖНО: pitch > 1500 = назад, pitch < 1500 = вперёд
        if abs(error_x) > 0.1:  # Порог
            if error_x > 0:  # Цель южнее - двигаемся на юг (назад)
                    pitch = neutral - intensity_x
            else:  # Цель южнее - двигаемся на юг (назад)
                    pitch = neutral + intensity_x
        else:
            pitch = neutral
            
        # Отладочный вывод для преследователя
        # if self.config.get('role') == 'follower':
        #     print(f"[Follower] Moving P: error_x={error_x:.3f}, error_y={error_y:.3f}, distance={distance:.3f}, intensity_x={intensity_x}, pitch={pitch}, roll={roll}")
        
        # Отправляем команду
        send_rc_override(self.master, roll, pitch, neutral, neutral, controller=self)
        
        # Вывод информации о регуляторе roll только для преследователя
        # if self.config.get('role') == 'follower':
        #     print(f"Roll regulator input (error_y): {error_y:.3f}, Roll output: {roll}")
        
        return {
            'roll': roll,
            'pitch': pitch,
            'distance': distance,
            'mode': 'moving',
            'error_x': error_x,
            'error_y': error_y,
            'intensity_x': intensity_x,
            'intensity_y': intensity_y
        }
    
    
    def stop(self):
        """Остановка дрона."""
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        self.coords_monitor.stop()
        self.velocity_monitor.stop()
        print("Setting mode LAND")
        self.master.set_mode(9)

def leader_pattern(controller):
    """
    Паттерн движения для лидирующего дрона (вперёд-назад).
    """
    
    compensated_roll = 1523 # Из-за сноса дрона

    print("[Leader] Исходное движение вперёд")
    start = time.time()
    while time.time() - start < 3:
        send_rc_override(controller.master, compensated_roll, 1550, 1500, 1500, controller)
        time.sleep(0.1)

    while True:
        # Нейтральные стики
        print("[Leader] Пауза (нейтральные стики)")
        send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)
        time.sleep(1.5)
        
        print("[Leader] Полёт назад")
        start = time.time()
        while time.time() - start < 5:
            send_rc_override(controller.master, compensated_roll, 1450, 1500, 1500, controller)
            time.sleep(0.1)
        
        # Нейтральные стики
        print("[Leader] Пауза (нейтральные стики)")
        send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)
        time.sleep(1.5)

        print("[Leader] Полёт вперёд")
        start = time.time()
        while time.time() - start < 5:
            send_rc_override(controller.master, compensated_roll, 1550, 1500, 1500, controller)
            time.sleep(0.1)

def follower_loop(controller, target_drone_id, kp=500.0):
    """
    Основной цикл преследования для следующего дрона с автоматическим торможением.
    """
    
    print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id} with P-control moving")
    
    while True:
        # Получаем позицию целевого дрона
        if target_drone_id not in controller.other_drones_positions:
            print(f"Follower: Waiting for position of drone {target_drone_id}...")
            time.sleep(0.1)
            continue
        
        target_position = controller.other_drones_positions[target_drone_id]
        # print(f"target_position = {target_position}")
        
        # Движение к цели с автоматическим торможением
        result = controller.move_towards(target_position, kp=kp)
        
        #time.sleep(0.1)  # Обновление каждые 100мс

def coordinate_exchange_loop(controllers):
    """
    Цикл обмена координатами между дронами.
    
    Args:
        controllers: список объектов DroneController
    """
    print("Starting coordinate exchange loop")
    
    while True:
        # Собираем позиции всех дронов
        positions = {}
        for controller in controllers:
            positions[controller.config['id']] = controller.get_my_position()
            # print(f"Drone {controller.config['id']} coordinates: {positions[controller.config['id']]}")
        
        # Распространяем позиции всем дронам
        for controller in controllers:
            for drone_id, position in positions.items():
                if drone_id != controller.config['id']:
                    controller.update_other_drone_position(drone_id, position)
        
        #time.sleep(0.1)  # Обновление каждые 100мс

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
    print("=== Drone Following System ===")
    
    # Создаём контроллеры для всех дронов (только объекты, без подключения)
    controllers = []
    for config in DRONES_CONFIG:
        controller = DroneController(config,logging=False)
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
    
    # Запускаем обмен координатами в отдельном потоке
    exchange_thread = threading.Thread(
        target=coordinate_exchange_loop,
        args=(controllers,),
        daemon=True
    )
    exchange_thread.start()
    print("Coordinate exchange thread started")
    
    print("=== Даём время на первый обмен координатами ===")
    time.sleep(1)  # Даём время на первый обмен координатами
    
    try:
        ''' Ваш код роевого алгоритма '''
        # Находим лидера и последователя
        leader = None
        follower = None
        
        for controller in controllers:
            if controller.config['role'] == 'leader':
                leader = controller
            elif controller.config['role'] == 'follower':
                follower = controller
        
        if not leader or not follower:
            print("Error: Leader or follower not found!")
            return
        print("Leader and follower found!")
        
        # Запускаем паттерн лидера в отдельном потоке
        leader_thread = threading.Thread(
            target=leader_pattern,
            args=(leader),
            daemon=True
        )
        leader_thread.start()
        print("Leader thread started")
        
        # Запускаем цикл преследования в основном потоке с PID-регулятором для движения и торможения
        print("Starting follower loop...")
        follower_loop(follower, leader.config['id'], kp=1000.0)
        while True:
            leader_velocity = leader.velocity_monitor.get_velocity()
            follower_velocity = follower.velocity_monitor.get_velocity()
            print(f"Leader velocity: {leader_velocity}")
            print(f"Follower velocity: {follower_velocity}")
        
    except KeyboardInterrupt:
        print("\nStopping all drones...")
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)  # LAND
        print("All drones stopped")

if __name__ == "__main__":
    main()

