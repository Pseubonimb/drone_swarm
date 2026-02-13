from pymavlink import mavutil
import time
import threading
import os
import sys
from CoordsMonitor import CoordsMonitor
from VelocityMonitor import VelocityMonitor

try:
    _proj_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if _proj_root not in sys.path:
        sys.path.insert(0, _proj_root)
    from visualizer.position_publisher import publish_positions
    HAS_VISUALIZER_PUBLISHER = True
except ImportError:
    HAS_VISUALIZER_PUBLISHER = False

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
            self.logfile = open(f"logs/EMPTY_TEST/drone_{self.config['id']}_log.txt", "w")
        
        # PID-регуляторы для управления углами (торможение)
        # Roll регулятор (для компенсации дрейфа влево-вправо)
        self.roll_velocity_pid = PIDRegulator(kp=100.0, ki=100.0, kd=0.0, 
                                               integral_limit=200.0, output_limit=500.0)
        # Pitch регулятор (для компенсации дрейфа вперёд-назад)
        self.pitch_velocity_pid = PIDRegulator(kp=100.0, ki=100.0, kd=0.0,
                                                integral_limit=200.0, output_limit=500.0)
        
        # PID-регуляторы для управления позицией (движение к цели)
        # Roll позиционный регулятор
        self.roll_position_pid = PIDRegulator(kp=100.0, ki=0.0, kd=0.0,
                                               integral_limit=100.0, output_limit=200.0)
        # Pitch позиционный регулятор
        self.pitch_position_pid = PIDRegulator(kp=100.0, ki=0.0, kd=0.0,
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

        print(f"[Drone {self.config['id']}] Setting mode ALT_HOLD")
        self.master.set_mode(2)  # ALT_HOLD
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
            if self.logIter % 10 == 0:
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
    
    def move_towards(self, target_position, kp=500.0, 
                     distance_threshold=0.5, kpx_brake=2000, kpy_brake=2000,
                     use_pid_braking=True, ki_roll_brake=50.0, ki_pitch_brake=50.0,
                     use_pid_moving=True, ki_roll_moving=10.0, ki_pitch_moving=10.0):
        """
        Движение к целевой позиции с PID-регулированием и автоматическим торможением.
        
        Args:
            target_position: словарь с ключами 'x', 'y', 'z' - целевая позиция
            kp: пропорциональный коэффициент для движения к цели (используется только если use_pid_moving=False)
            distance_threshold: порог расстояния для начала торможения (м)
            kpx_brake: пропорциональный коэффициент для торможения по X (восток-запад) - используется только если use_pid_braking=False
            kpy_brake: пропорциональный коэффициент для торможения по Y (север-юг) - используется только если use_pid_braking=False
            use_pid_braking: использовать ли PID-регулятор для торможения (True) или только П-регулятор (False)
            ki_roll_brake: интегральный коэффициент для roll регулятора при торможении
            ki_pitch_brake: интегральный коэффициент для pitch регулятора при торможении
            use_pid_moving: использовать ли PID-регулятор для движения к цели (True) или только П-регулятор (False)
            ki_roll_moving: интегральный коэффициент для roll регулятора при движении к цели
            ki_pitch_moving: интегральный коэффициент для pitch регулятора при движении к цели
        """
        
        rel_pos = self.coords_monitor.get_relative_position(target_position)
        
        # Вычисляем расстояние до цели
        # В NED: x = север, y = восток
        error_x = rel_pos['x']  # Ошибка по северу-югу (x в NED)
        error_y = rel_pos['y'] - 2 # Ошибка по востоку-западу (y в NED)
                                    # Смещаем на 2 метра вправо    
        
                                    
        distance = (error_x**2 + error_y**2)**0.5
        
        # Получаем текущую скорость
        current_velocity = self.velocity_monitor.get_velocity()
        # В NED: vx = север, vy = восток
        current_x_velocity = current_velocity['vx']  # Скорость на север
        current_y_velocity = current_velocity['vy']     # Скорость на восток
        
        # Если близко к цели - применяем торможение
        if distance < distance_threshold:
            # print("[Follower] Braking")
            # Режим торможения: используем PID-регулятор на основе скорости
            target_velocity = 0.0
            
            # Обновляем режим и сбрасываем интегральную составляющую при переходе в режим торможения
            if self.last_movement_mode != 'braking':
                self.roll_velocity_pid.reset()
                self.pitch_velocity_pid.reset()
                # Обновляем коэффициенты интегральной составляющей
                self.roll_velocity_pid.ki = ki_roll_brake
                self.pitch_velocity_pid.ki = ki_pitch_brake
            self.last_movement_mode = 'braking'
            
            # Используем фиксированный временной интервал для PID (примерно соответствует частоте вызова)
            dt = 0.1
            
            # Применяем алгоритм торможения
            brake_result = self.apply_braking(
                target_velocity=target_velocity,
                velocity_threshold=0.8,  # Можно сделать параметром, если нужно
                dt=dt
            )
            
            # Применяем торможение
            send_rc_override(self.master, brake_result['brake_roll'], brake_result['brake_pitch'], 
                          neutral, neutral, controller=self)
            
            # Вывод информации о регуляторе ТОРМОЖЕНИЯ преследователя
            print(f"BRAKE Pitch PID: error_vx={brake_result['error_vx']:.3f}, brake_pitch={brake_result['brake_pitch']}")
            print(f"BRAKE Roll PID: error_vy={brake_result['error_vy']:.3f}, brake_roll={brake_result['brake_roll']}")
            
            return {
                'roll': brake_result['brake_roll'],
                'pitch': brake_result['brake_pitch'],
                'distance': distance,
                'mode': 'braking',
                'velocity_x': brake_result['current_x_velocity'],
                'velocity_y': brake_result['current_y_velocity'],
                'error_x': error_x,
                'error_y': error_y,
                'error_vx': brake_result['error_vx'],
                'error_vy': brake_result['error_vy'],
                'intensity_x': brake_result['brake_intensity_x'],
                'intensity_y': brake_result['brake_intensity_y']
            }

        # Если далеко от цели - двигаемся к ней
        else:
            # print("[Follower] Following")
            # Режим движения к цели: используем PID-регулятор на основе позиции
            # Сбрасываем интегральную составляющую при переходе из режима торможения в режим движения
            if self.last_movement_mode == 'braking':
                self.roll_position_pid.reset()
                self.pitch_position_pid.reset()
                # Обновляем коэффициенты интегральной составляющей для движения
                self.roll_position_pid.ki = ki_roll_moving
                self.pitch_position_pid.ki = ki_pitch_moving
            elif self.last_movement_mode != 'moving':
                # Первый запуск - обновляем коэффициенты
                self.roll_position_pid.ki = ki_roll_moving
                self.pitch_position_pid.ki = ki_pitch_moving
            self.last_movement_mode = 'moving'
            
            # Используем фиксированный временной интервал для PID (примерно соответствует частоте вызова)
            dt = 0.1
            
            # Используем PID-регуляторы для движения к цели
            # error_x управляет pitch (вперёд-назад), error_y управляет roll (влево-вправо)
            pitch_output = self.pitch_position_pid.update(error_x, dt)
            roll_output = self.roll_position_pid.update(error_y, dt)
            
            # Преобразуем выход PID в значения RC каналов
            # Ограничиваем выход регуляторов
            pitch_output = max(-500, min(500, pitch_output))
            roll_output = max(-500, min(500, roll_output))
            
            # Вычисляем интенсивность для возвращаемого словаря
            intensity_x = int(abs(pitch_output))
            intensity_y = int(abs(roll_output))
            
            # Определяем направление и интенсивность
            # Pitch управляет движением вперёд-назад (север-юг)
            # ВАЖНО: pitch > 1500 = назад, pitch < 1500 = вперёд
            if error_x > 0:  # Цель севернее - двигаемся на север (вперёд)
                pitch = neutral - intensity_x  # < 1500 для движения вперёд
            else:  # Цель южнее - двигаемся на юг (назад)
                pitch = neutral + intensity_x  # > 1500 для движения назад
            
            
            # Roll управляет движением влево-вправо (восток-запад)
            if error_y > 0:
                roll = neutral + intensity_y  # > 1500 для движения вправо
            else: 
                roll = neutral - intensity_y  # < 1500 для движения влево
            
            
            # Отправляем команду
            send_rc_override(self.master, roll, pitch, neutral, neutral, controller=self)
            
            # Вывод информации о регуляторе ПРЕСЛЕДОВАНИЯ для преследователя
            print(f"FOLLOW Pitch PID: error_x={error_x:.3f}, pitch={pitch}")
            print(f"FOLLOW Roll PID: error_y={error_y:.3f}, roll={roll}")
            
            return {
                'roll': roll,
                'pitch': pitch,
                'distance': distance,
                'mode': 'moving',
                'velocity_x': current_x_velocity,
                'velocity_y': current_y_velocity,
                'error_x': error_x,
                'error_y': error_y,
                'intensity_x': intensity_x,
                'intensity_y': intensity_y
            }
    
    def apply_braking(self, target_velocity=0.0, velocity_threshold=0.8, dt=0.1):
        """
        Применение алгоритма торможения с использованием PID-регуляторов.
        
        Args:
            target_velocity: целевая скорость (обычно 0.0)
            velocity_threshold: порог скорости для остановки
            dt: временной интервал для PID-регулятора
        
        Returns:
            dict с ключами:
                'brake_roll': значение roll для торможения
                'brake_pitch': значение pitch для торможения
                'should_continue': True если нужно продолжать торможение, False если достигнут порог
                'current_x_velocity': текущая скорость на север
                'current_y_velocity': текущая скорость на восток
                'error_vx': ошибка скорости на север
                'error_vy': ошибка скорости на восток
                'brake_intensity_x': интенсивность торможения по оси X
                'brake_intensity_y': интенсивность торможения по оси Y
        """
        # Получаем текущую скорость
        current_velocity = self.velocity_monitor.get_velocity()
        # В NED: vx = север, vy = восток
        current_x_velocity = current_velocity['vx']  # Скорость на север
        current_y_velocity = current_velocity['vy']   # Скорость на восток
        
        # Проверяем, достигнут ли порог скорости
        if abs(current_x_velocity) < velocity_threshold and abs(current_y_velocity) < velocity_threshold:
            # Сбрасываем интегральную составляющую при достижении цели
            self.roll_velocity_pid.reset()
            self.pitch_velocity_pid.reset()
            return {
                'brake_roll': neutral,
                'brake_pitch': neutral,
                'should_continue': False,
                'current_x_velocity': current_x_velocity,
                'current_y_velocity': current_y_velocity,
                'error_vx': 0.0,
                'error_vy': 0.0,
                'brake_intensity_x': 0,
                'brake_intensity_y': 0
            }
        
        # Ошибки скорости
        error_vx = target_velocity - current_x_velocity  # Ошибка скорости на север
        error_vy = target_velocity - current_y_velocity    # Ошибка скорости на восток
        
        # Используем PID-регуляторы
        # Торможение по северу-югу управляет pitch (вперёд-назад)
        brake_output_pitch = self.pitch_velocity_pid.update(error_vx, dt)
        brake_intensity_x = int(abs(brake_output_pitch))
        brake_intensity_x = min(500, max(50, brake_intensity_x))
        # Направление торможения: если error_vx > 0 (движение на север), тормозим на юг
        brake_direction_x = -1 if error_vx > 0 else 1
        brake_pitch = neutral + (brake_direction_x * brake_intensity_x)
        
        # Торможение по востоку-западу управляет roll (влево-вправо)
        # Это критично для лидера - компенсирует накопление ошибки по roll
        brake_output_roll = self.roll_velocity_pid.update(error_vy, dt)
        brake_intensity_y = int(abs(brake_output_roll))
        brake_intensity_y = min(500, max(50, brake_intensity_y))
        # Направление торможения: если error_vy > 0 (движение на восток), тормозим на запад
        brake_direction_y = 1 if error_vy > 0 else -1
        brake_roll = neutral + (brake_direction_y * brake_intensity_y)
        
        return {
            'brake_roll': brake_roll,
            'brake_pitch': brake_pitch,
            'should_continue': True,
            'current_x_velocity': current_x_velocity,
            'current_y_velocity': current_y_velocity,
            'error_vx': error_vx,
            'error_vy': error_vy,
            'brake_intensity_x': brake_intensity_x,
            'brake_intensity_y': brake_intensity_y
        }
    
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
        # print("[Leader] Движение")
        start = time.time()
        while time.time() - start < move_duration:
            send_rc_override(self.master, roll, pitch, throttle, yaw, controller=self)
            time.sleep(0.1)
        
        # Переход в режим торможения
        self.last_movement_mode = 'braking'
        
        # PID-регулирование для торможения
        start_time = time.time()
        last_update_time = time.time()  # Для I-составляющей PID-регулятора
        
        while time.time() - start_time < max_brake_time:
            current_time = time.time()
            dt = current_time - last_update_time
            if dt <= 0:
                dt = 0.05  # Минимальный интервал
            last_update_time = current_time
            
            # Применяем алгоритм торможения
            brake_result = self.apply_braking(
                target_velocity=target_velocity,
                velocity_threshold=velocity_threshold,
                dt=dt
            )
            
            # Если достигнут порог скорости, прекращаем торможение
            if not brake_result['should_continue']:
                print(f"[Drone {self.config['id']}] Velocities threshold reached")
                break
            
            # Вывод информации о регуляторе для отладки (только для лидера)
            # if self.config.get('role') == 'leader':
            #     print(f"[Leader] Pitch PID: error_vx={brake_result['error_vx']:.3f}, pitch={brake_result['brake_pitch']}, error_vy={brake_result['error_vy']:.3f}, roll={brake_result['brake_roll']}")
            
            # Применяем торможение
            send_rc_override(self.master, brake_result['brake_roll'], brake_result['brake_pitch'], 
                          neutral, neutral, controller=self)
            
            time.sleep(0.05)  # ЗАДАЁТ dt ДЛЯ ИНТЕГРАЛЬНОЙ СОСТАВЛЯЮЩЕЙ!!!
    
    def stop(self):
        """Остановка дрона."""
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        self.coords_monitor.stop()
        self.velocity_monitor.stop()
        print("Setting mode LAND")
        self.master.set_mode(9)

def leader_pattern(controller, duration=5, kpx=100, kpy=100, 
                   use_pid=True):
    """
    Паттерн движения для лидирующего дрона (вперёд-назад) с PID-регулятором торможения.
    
    Args:
        controller: объект DroneController
        duration: длительность движения в одном направлении
        kpx: пропорциональный коэффициент для торможения по X (восток-запад) - используется только если use_pid=False
        kpy: пропорциональный коэффициент для торможения по Y (север-юг) - используется только если use_pid=False
        use_pid: использовать ли PID-регулятор (True) или только П-регулятор (False)
    """
    if use_pid:
        print(f"Leader drone {controller.config['id']} starting forward-backward pattern with PID-control braking")
    else:
        print(f"Leader drone {controller.config['id']} starting forward-backward pattern with P-control braking")
    
    while True:
        # # Вперёд с торможением
        # print("[Leader] Полёт вперёд")
        # controller.move_with_pid_braking(
        #     (neutral, 1700, neutral, neutral),
        #     kpx=kpx, kpy=kpy,
        #     move_duration=duration,
        #     target_velocity=0.0,
        #     max_brake_time=20.0,
        #     velocity_threshold=0.05,
        #     use_pid=use_pid
        # )
        
        # # Пауза после торможения
        # send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)
        # time.sleep(1)
        
        # # Назад с торможением
        # print("[Leader] Полёт назад")
        # controller.move_with_pid_braking(
        #     (neutral, 1300, neutral, neutral),
        #     kpx=kpx, kpy=kpy,
        #     move_duration=duration,
        #     target_velocity=0.0,
        #     max_brake_time=20.0,
        #     velocity_threshold=0.05,
        #     use_pid=use_pid
        # )
        
        # Пауза после торможения
        send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)
        time.sleep(1)

def follower_loop(controller, target_drone_id, kp=500.0,
                 kpx_brake=2000, kpy_brake=2000, distance_threshold=0.5,
                 use_pid_braking=True, ki_roll_brake=50.0, ki_pitch_brake=50.0,
                 use_pid_moving=True, ki_roll_moving=10.0, ki_pitch_moving=10.0):
    """
    Основной цикл преследования для следующего дрона с автоматическим торможением.
    
    Args:
        controller: объект DroneController
        target_drone_id: ID дрона, которого нужно преследовать
        kp: пропорциональный коэффициент для движения к цели (используется только если use_pid_moving=False)
        kpx_brake: пропорциональный коэффициент для торможения по X (восток-запад) - используется только если use_pid_braking=False
        kpy_brake: пропорциональный коэффициент для торможения по Y (север-юг) - используется только если use_pid_braking=False
        distance_threshold: порог расстояния для начала торможения (м)
        use_pid_braking: использовать ли PID-регулятор для торможения (True) или только П-регулятор (False)
        ki_roll_brake: интегральный коэффициент для roll регулятора при торможении
        ki_pitch_brake: интегральный коэффициент для pitch регулятора при торможении
        use_pid_moving: использовать ли PID-регулятор для движения к цели (True) или только П-регулятор (False)
        ki_roll_moving: интегральный коэффициент для roll регулятора при движении к цели
        ki_pitch_moving: интегральный коэффициент для pitch регулятора при движении к цели
    """
    if use_pid_braking and use_pid_moving:
        print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id} with PID-control (moving and braking)")
    elif use_pid_braking:
        print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id} with PID-control braking")
    elif use_pid_moving:
        print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id} with PID-control moving")
    else:
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
        # print(f"target_position = {target_position}")
        
        # Движение к цели с автоматическим торможением
        result = controller.move_towards(
            target_position, 
            kp=kp, 
            distance_threshold=distance_threshold,
            kpx_brake=kpx_brake,
            kpy_brake=kpy_brake,
            use_pid_braking=use_pid_braking,
            ki_roll_brake=ki_roll_brake,
            ki_pitch_brake=ki_pitch_brake,
            use_pid_moving=use_pid_moving,
            ki_roll_moving=ki_roll_moving,
            ki_pitch_moving=ki_pitch_moving
        )
        
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

        if HAS_VISUALIZER_PUBLISHER:
            publish_positions(positions)

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
        controller = DroneController(config,logging=True) #Включаем логирование координат
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
        
        # Запускаем паттерн лидера в отдельном потоке с PID-регулятором
        # ki_roll=50.0 - интегральный коэффициент для компенсации дрейфа по roll
        leader_thread = threading.Thread(
            target=leader_pattern,
            args=(leader, 5, 100, 100, True),  # duration, kpx, kpy, use_pid
            daemon=True
        )
        leader_thread.start()
        print("Leader thread started with PID control")
        
        # Запускаем цикл преследования в основном потоке с PID-регулятором для движения и торможения
        print("Starting follower loop...")
        follower_loop(follower, leader.config['id'], 
                     kp=100.0,
                     kpx_brake=100, kpy_brake=100,
                     distance_threshold=2.0,
                     use_pid_braking=True,
                     ki_roll_brake=50.0,
                     ki_pitch_brake=50.0,
                     use_pid_moving=True,
                     ki_roll_moving=10.0,
                     ki_pitch_moving=10.0)
        
    except KeyboardInterrupt:
        print("\nStopping all drones...")
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)  # LAND
        print("All drones stopped")

if __name__ == "__main__":
    main()

