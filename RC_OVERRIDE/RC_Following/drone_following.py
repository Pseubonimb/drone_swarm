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

def send_rc_override(drone, chan1, chan2, chan3, chan4):
    """Отправка RC_OVERRIDE команды дрону."""
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        chan1,  # roll / aileron (left/right)
        chan2,  # pitch / elevator (forward/back)
        chan3,  # throttle
        chan4,  # yaw / rudder (rotation)
        0, 0, 0, 0, 0, 0  # остальные каналы без изменений
    )

class DroneController:
    """Класс для управления одним дроном."""
    def __init__(self, config):
        self.config = config
        self.master = None
        self.coords_monitor = None
        self.velocity_monitor = None
        self.other_drones_positions = {}  # Словарь для хранения позиций других дронов
        self.lock = threading.Lock()
    
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

        print(f"[Drone {self.config['id']}] Setting mode ALT_HOLD")
        self.master.set_mode(2)  # ALT_HOLD
        send_rc_override(self.master, neutral, neutral, neutral, neutral)
        time.sleep(0.5)
        
        # Запуск мониторинга
        print(f"[Drone {self.config['id']}] Starting monitors")
        self.coords_monitor.start()
        self.velocity_monitor.start()
    
    def start_rc_keepalive(self):
        """
        Запуск потока для непрерывной отправки RC_OVERRIDE.
        Это предотвращает истечение RC_OVERRIDE_TIME (3 секунды по умолчанию).
        Keepalive отправляет нейтральные значения для поддержания связи.
        """
        def keepalive_loop():
            while True:
                try:
                    # Отправляем нейтральные значения для поддержания связи
                    # Это предотвращает истечение RC_OVERRIDE_TIME
                    send_rc_override(self.master, neutral, neutral, neutral, neutral)
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
        return self.coords_monitor.get_position()
    
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
    
    def move_towards(self, target_position, kp=500.0, max_intensity=200, 
                     distance_threshold=0.5, kpx_brake=2000, kpy_brake=2000):
        """
        Движение к целевой позиции с П-регулированием и автоматическим торможением.
        Аналогично move_with_pid_braking - использует П-регулятор для торможения.
        
        Args:
            target_position: словарь с ключами 'x', 'y', 'z' - целевая позиция
            kp: пропорциональный коэффициент для движения к цели
            max_intensity: максимальная интенсивность управления при движении
            distance_threshold: порог расстояния для начала торможения (м)
            kpx_brake: пропорциональный коэффициент для торможения по X (восток-запад)
            kpy_brake: пропорциональный коэффициент для торможения по Y (север-юг)
            velocity_threshold: порог скорости для остановки торможения (м/с)
        """
        rel_pos = self.coords_monitor.get_relative_position(target_position)
        
        # Вычисляем расстояние до цели
        error_x = rel_pos['x']  # Ошибка по X (восток-запад)
        error_y = rel_pos['y']  # Ошибка по Y (север-юг)
        distance = (error_x**2 + error_y**2)**0.5
        
        # Получаем текущую скорость
        current_velocity = self.velocity_monitor.get_velocity()
        current_x_velocity = current_velocity['vx']
        current_y_velocity = current_velocity['vy']
        
        # Если близко к цели ИЛИ скорость достаточно мала - применяем торможение
        if distance < distance_threshold:
            # Режим торможения: используем П-регулятор на основе скорости
            target_velocity = 0.0
            
            # П-регулирование: ошибка * коэффициент = интенсивность торможения
            error_vx = target_velocity - current_x_velocity
            error_vy = target_velocity - current_y_velocity
            
            # Вычисляем интенсивность торможения для каждого канала
            brake_intensity_x = 0
            brake_intensity_y = 0
            
            if kpx_brake > 0 and abs(error_vx) > 0.01:
                brake_intensity_x = int(abs(error_vx) * kpx_brake)
                brake_intensity_x = min(500, max(50, brake_intensity_x))
                # Направление торможения по X: если error_vx > 0 (движение на восток), тормозим на запад
                brake_direction_x = -1 if error_vx > 0 else 1
                brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
            else:
                brake_pitch = neutral  # Не применяем торможение по pitch
            
            if kpy_brake > 0 and abs(error_vy) > 0.01:
                brake_intensity_y = int(abs(error_vy) * kpy_brake)
                brake_intensity_y = min(500, max(50, brake_intensity_y))
                # Направление торможения по Y: если error_vy > 0 (движение на север), тормозим на юг
                brake_direction_y = 1 if error_vy > 0 else -1
                brake_roll = neutral + (brake_direction_y * brake_intensity_y)
            else:
                brake_roll = neutral  # Не применяем торможение по roll
            
            # Применяем торможение
            send_rc_override(self.master, brake_roll, brake_pitch, neutral, neutral)
            
            return {
                'roll': brake_roll,
                'pitch': brake_pitch,
                'distance': distance,
                'mode': 'braking',
                'velocity_x': current_x_velocity,
                'velocity_y': current_y_velocity
            }
        else:
            # Режим движения к цели: используем П-регулятор на основе позиции
            # П-регулирование
            intensity_x = int(abs(error_x) * kp)
            intensity_y = int(abs(error_y) * kp)
            
            # Ограничиваем интенсивность
            intensity_x = min(max_intensity, intensity_x)
            intensity_y = min(max_intensity, intensity_y)
            
            # Определяем направление
            if abs(error_x) > 0.1:  # Порог для движения по X
                if error_x > 0:
                    roll = neutral - intensity_x  # Вправо (на восток)
                else:
                    roll = neutral + intensity_x  # Влево (на запад)
            else:
                roll = neutral
            
            if abs(error_y) > 0.1:  # Порог для движения по Y
                if error_y > 0:
                    pitch = neutral - intensity_y  # Вперёд (на север)
                else:
                    pitch = neutral + intensity_y  # Назад (на юг)
            else:
                pitch = neutral
            
            # Отправляем команду
            send_rc_override(self.master, roll, pitch, neutral, neutral)
            
            return {
                'roll': roll,
                'pitch': pitch,
                'distance': distance,
                'mode': 'moving',
                'velocity_x': current_x_velocity,
                'velocity_y': current_y_velocity
            }
    
    def move_with_pid_braking(self, direction, kpx=2000, kpy=2000,
                              move_duration=5, target_velocity=0.0, 
                              max_brake_time=10.0, velocity_threshold=0.01):
        """
        Движение с П-регулированием для торможения (из RC_OVERRIDE_square.py).
        
        Args:
            direction: кортеж (roll, pitch, throttle, yaw) - направление движения
            kpx: пропорциональный коэффициент для оси X (восток-запад)
            kpy: пропорциональный коэффициент для оси Y (север-юг)
            move_duration: длительность движения в секундах
            target_velocity: целевая скорость (обычно 0.0)
            max_brake_time: максимальное время торможения
            velocity_threshold: порог скорости для остановки
        """
        roll, pitch, throttle, yaw = direction
        
        # Движение
        direction_name = "Forward" if pitch > neutral else "Backward" if pitch < neutral else \
                         "Right" if roll > neutral else "Left" if roll < neutral else "Unknown"
        print(f"[Drone {self.config['id']}] Moving {direction_name}: roll={roll}, pitch={pitch}")
        
        start = time.time()
        while time.time() - start < move_duration:
            send_rc_override(self.master, roll, pitch, throttle, yaw)
            time.sleep(0.1)
        
        # П-регулирование для торможения
        print(f"[Drone {self.config['id']}] Starting P-control braking...")
        start_time = time.time()
        
        while time.time() - start_time < max_brake_time:
            # Получаем текущую скорость
            current_velocity = self.velocity_monitor.get_velocity()
            current_x_velocity = current_velocity['vx']
            current_y_velocity = current_velocity['vy']
            
            # Если скорость достаточно мала, прекращаем
            if abs(current_x_velocity) < velocity_threshold and abs(current_y_velocity) < velocity_threshold:
                print(f"[Drone {self.config['id']}] Velocities threshold reached")
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
                # Направление торможения по X: если error_x > 0 (движение на восток), тормозим на запад
                brake_direction_x = -1 if error_x > 0 else 1
                brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
            else:
                brake_pitch = neutral  # Не применяем торможение по pitch
            
            if kpy > 0:
                brake_intensity_y = int(abs(error_y) * kpy)
                brake_intensity_y = min(500, max(50, brake_intensity_y))
                # Направление торможения по Y: если error_y > 0 (движение на север), тормозим на юг
                brake_direction_y = 1 if error_y > 0 else -1
                brake_roll = neutral + (brake_direction_y * brake_intensity_y)
            else:
                brake_roll = neutral  # Не применяем торможение по roll
            
            # Применяем торможение
            send_rc_override(self.master, brake_roll, brake_pitch, neutral, neutral)
            
            time.sleep(0.05)
    
    def stop(self):
        """Остановка дрона."""
        send_rc_override(self.master, neutral, neutral, neutral, neutral)
        self.coords_monitor.stop()
        self.velocity_monitor.stop()

def leader_pattern(controller, duration=5, kpx=100, kpy=100):
    """
    Паттерн движения для лидирующего дрона (вперёд-назад) с П-регулятором торможения.
    
    Args:
        controller: объект DroneController
        duration: длительность движения в одном направлении
        kpx: пропорциональный коэффициент для торможения по X (восток-запад)
        kpy: пропорциональный коэффициент для торможения по Y (север-юг)
    """
    print(f"Leader drone {controller.config['id']} starting forward-backward pattern with P-control braking")
    
    while True:
        # Вперёд с торможением
        controller.move_with_pid_braking(
            (neutral, 1700, neutral, neutral),
            kpx=kpx, kpy=kpy,
            move_duration=duration,
            target_velocity=0.0,
            max_brake_time=5.0,
            velocity_threshold=0.8
        )
        
        # Пауза после торможения
        send_rc_override(controller.master, neutral, neutral, neutral, neutral)
        time.sleep(1)
        
        # Назад с торможением
        controller.move_with_pid_braking(
            (neutral, 1300, neutral, neutral),
            kpx=kpx, kpy=kpy,
            move_duration=duration,
            target_velocity=0.0,
            max_brake_time=5.0,
            velocity_threshold=0.8
        )
        
        # Пауза после торможения
        send_rc_override(controller.master, neutral, neutral, neutral, neutral)
        time.sleep(1)

def follower_loop(controller, target_drone_id, kp=500.0, max_intensity=200,
                 kpx_brake=2000, kpy_brake=2000, distance_threshold=0.5):
    """
    Основной цикл преследования для следующего дрона с автоматическим торможением.
    
    Args:
        controller: объект DroneController
        target_drone_id: ID дрона, которого нужно преследовать
        kp: пропорциональный коэффициент для движения к цели
        max_intensity: максимальная интенсивность управления при движении
        kpx_brake: пропорциональный коэффициент для торможения по X (восток-запад)
        kpy_brake: пропорциональный коэффициент для торможения по Y (север-юг)
        distance_threshold: порог расстояния для начала торможения (м)
        velocity_threshold: порог скорости для остановки торможения (м/с)
    """
    print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id}")
    
    iteration = 0
    while True:
        iteration += 1
        # Получаем позицию целевого дрона
        if target_drone_id not in controller.other_drones_positions:
            print(f"Follower: Waiting for position of drone {target_drone_id}...")
            time.sleep(0.1)
            continue
        
        target_position = controller.other_drones_positions[target_drone_id]
        
        # Движение к цели с автоматическим торможением
        result = controller.move_towards(
            target_position, 
            kp=kp, 
            max_intensity=max_intensity,
            distance_threshold=distance_threshold,
            kpx_brake=kpx_brake,
            kpy_brake=kpy_brake
        )
        
        distance = result['distance']
        mode = result.get('mode', 'unknown')
        
        # Выводим информацию каждые 10 итераций, чтобы не засорять консоль
        if iteration % 10 == 0:
            print(f"Follower: Distance={distance:.2f}m, Mode={mode}, "
                  f"vx={result.get('velocity_x', 0):.3f}m/s, vy={result.get('velocity_y', 0):.3f}m/s, "
                  f"roll={result['roll']}, pitch={result['pitch']}")
        
        time.sleep(0.1)  # Обновление каждые 100мс

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
        
        # Распространяем позиции всем дронам
        for controller in controllers:
            for drone_id, position in positions.items():
                if drone_id != controller.config['id']:
                    controller.update_other_drone_position(drone_id, position)
        
        time.sleep(0.1)  # Обновление каждые 100мс

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
        controller = DroneController(config)
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
            args=(leader, 5),
            daemon=True
        )
        leader_thread.start()
        print("Leader thread started")
        
        # Запускаем цикл преследования в основном потоке
        print("Starting follower loop...")
        follower_loop(follower, leader.config['id'], 
                     kp=100.0, max_intensity=200,
                     kpx_brake=100, kpy_brake=100,
                     distance_threshold=0.5)
        
    except KeyboardInterrupt:
        print("\nStopping all drones...")
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)  # LAND
        print("All drones stopped")

if __name__ == "__main__":
    main()

