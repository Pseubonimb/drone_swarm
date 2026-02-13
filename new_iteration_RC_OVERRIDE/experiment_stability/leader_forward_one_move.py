'''
# Оба дрона летают в POS_HOLD
# Два монитора CoordsMonitor
# Один регулятор преследования
'''

from pymavlink import mavutil
import time
import threading
import os
from CoordsMonitor import CoordsMonitor
from VelocityMonitor import VelocityMonitor

# Опционально: публикация позиций для визуализатора (если visualizer в PYTHONPATH)
try:
    import sys
    _proj_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    if _proj_root not in sys.path:
        sys.path.insert(0, _proj_root)
    from visualizer.position_publisher import publish_positions
    HAS_VISUALIZER_PUBLISHER = True
except ImportError:
    HAS_VISUALIZER_PUBLISHER = False

# Shared rates for visualizer: follower_loop writes follower_hz, coordinate_exchange_loop reads and adds exchange_hz, webots_step_hz
RATES_SHARED = {"follower_hz": None, "exchange_hz": None, "webots_step_hz": None}
# For Webots step rate: last timestamp per instance (file /tmp/webots_step_<i>.txt)
WEBOTS_LAST_TS = {}
FOLLOWER_HZ_HISTORY = []  # last N values of 1/loop_dt in follower_loop
FOLLOWER_HZ_WINDOW = 20
EXCHANGE_HZ_HISTORY = []
EXCHANGE_HZ_WINDOW = 20

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
    def __init__(self, kp=1.0, ki=0.0, kd=0.0, integral_limit=1000.0, output_limit=500.0, derivative_alpha=0.7):
        """
        Инициализация PID-регулятора.
        derivative_alpha: сглаживание D-члена (0=без фильтра, 0.7=умеренное); уменьшает шум при Kd>0.
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_alpha = max(0.0, min(1.0, derivative_alpha))
        
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = None
        self._derivative_filtered = 0.0
    
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
        
        # Дифференциальная составляющая (с опциональным сглаживанием)
        if dt > 0:
            derivative = (error - self.last_error) / dt
        else:
            derivative = 0.0
        if self.derivative_alpha > 0:
            self._derivative_filtered = self.derivative_alpha * self._derivative_filtered + (1.0 - self.derivative_alpha) * derivative
            derivative = self._derivative_filtered
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
        self._derivative_filtered = 0.0
    
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


def request_position_stream_rate(master, hz=50):
    """
    Запросить у автопилота частую выдачу LOCAL_POSITION_NED (см. docs/PITFALLS_AND_SOLUTIONS.md).
    Без этого поток позиции может быть 1–4 Hz по умолчанию.
    """
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        interval_us, 0, 0, 0, 0, 0)
    print(f"[MAVLink] Requested LOCAL_POSITION_NED at {hz} Hz (interval {interval_us} us)")


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
        
        # Запрос частой выдачи позиции (см. docs/PITFALLS_AND_SOLUTIONS.md)
        request_position_stream_rate(self.master, hz=50)
        time.sleep(0.2)
        
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
    
    def move_towards_with_pid(self, target_position, roll_pid=None, pitch_pid=None, dt=None, distance_threshold=0.4):
        """
        Движение к целевой позиции с использованием PID-регуляторов.
        dt: при заданном rate limit передавать фиксированный шаг (например 0.02 для 50 Hz).
        distance_threshold: когда расстояние до цели < порога, отправляем нейтраль — дрон останавливается.
        """
        rel_pos = self.coords_monitor.get_relative_position(target_position)
        
        error_x = rel_pos['x']
        error_y = rel_pos['y'] - 2  # смещаем на 2 метра
        distance = (error_x**2 + error_y**2)**0.5

        if distance < distance_threshold:
            # В зоне цели — нейтраль, дрон держит позицию
            send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
            if roll_pid is not None:
                roll_pid.reset()
            if pitch_pid is not None:
                pitch_pid.reset()
            return

        if pitch_pid is not None:
            pitch_output = pitch_pid.update(error_x, dt=dt)
        else:
            pitch_output = error_x * 500.0

        if roll_pid is not None:
            roll_output = roll_pid.update(error_y, dt=dt)
        else:
            roll_output = error_y * 500.0

        pitch = neutral - int(pitch_output)  # если pitch_output > 0, то pitch < 1500 - вперёд
        roll = neutral + int(roll_output)    # если roll_output > 0, то roll > 1500 - влево

        send_rc_override(self.master, roll, pitch, neutral, neutral, controller=self)

    
    def stop(self):
        """Остановка дрона."""
        send_rc_override(self.master, neutral, neutral, neutral, neutral, controller=self)
        self.coords_monitor.stop()
        self.velocity_monitor.stop()
        print("Setting mode LAND")
        self.master.set_mode(9)

def leader_pattern(controller, forward_duration=10):
    """
    Паттерн движения для лидирующего дрона (вперёд, затем остановка).
    После окончания команд использует BRAKE для гарантированной остановки, затем POS_HOLD.
    """
    print(f"[Leader] Движение вперёд {forward_duration} с")
    start = time.time()
    while time.time() - start < forward_duration:
        send_rc_override(controller.master, neutral, 1400, 1500, 1500, controller)
        time.sleep(0.1)

    print("[Leader] Остановка (BRAKE mode)")
    controller.master.set_mode(17)  # BRAKE — быстрая остановка
    time.sleep(2)
    print("[Leader] Возврат в POS_HOLD, нейтральные стики")
    controller.master.set_mode(16)  # POS_HOLD
    time.sleep(0.3)
    send_rc_override(controller.master, neutral, neutral, neutral, neutral, controller=controller)

def follower_loop(controller, target_drone_id, kp=500.0, ki=0.0, kd=0.0,
                  control_loop_period_sec=None, log_run_id=None):
    """
    Основной цикл преследования. Собирает данные для проверки гипотез:
    - control_loop_period_sec: если задан, ограничивает частоту цикла (например 0.02 = 50 Hz).
    - log_run_id: суффикс имени файла лога для сравнения экспериментов.
    """
    global START_TIME

    print(f"Follower drone {controller.config['id']} starting pursuit of drone {target_drone_id} "
          f"(kp={kp}, ki={ki}, kd={kd}, loop_period={control_loop_period_sec}, run_id={log_run_id})")
    
    roll_pid = PIDRegulator(kp=kp, ki=ki, kd=kd, integral_limit=100.0, output_limit=200.0)
    pitch_pid = PIDRegulator(kp=kp, ki=ki, kd=kd, integral_limit=100.0, output_limit=200.0)

    os.makedirs("logs", exist_ok=True)
    suffix = f"_{log_run_id}" if log_run_id else ""
    log_filename = f"logs/two_drones_log{suffix}.csv"
    log_file = open(log_filename, "w")
    # Расширенный заголовок для анализа: скорость преследователя и период цикла
    log_file.write("t,leader_x,leader_y,follower_x,follower_y,error_x,error_y,follower_vx,follower_vy,loop_dt\n")
    log_file.flush()

    last_iter_time = None
    try:
        while True:
            if target_drone_id not in controller.other_drones_positions:
                print(f"Follower: Waiting for position of drone {target_drone_id}...")
                time.sleep(0.1)
                continue

            now = time.time()
            loop_dt = (now - last_iter_time) if last_iter_time is not None else 0.0
            last_iter_time = now

            # Update shared follower_hz for visualizer (sliding average of 1/loop_dt)
            if loop_dt > 0:
                global FOLLOWER_HZ_HISTORY
                FOLLOWER_HZ_HISTORY.append(1.0 / loop_dt)
                if len(FOLLOWER_HZ_HISTORY) > FOLLOWER_HZ_WINDOW:
                    FOLLOWER_HZ_HISTORY = FOLLOWER_HZ_HISTORY[-FOLLOWER_HZ_WINDOW:]
                RATES_SHARED["follower_hz"] = sum(FOLLOWER_HZ_HISTORY) / len(FOLLOWER_HZ_HISTORY)

            leader_pos = controller.other_drones_positions[target_drone_id]
            follower_pos = controller.get_my_position()
            rel_pos = controller.coords_monitor.get_relative_position(leader_pos)
            error_x = rel_pos['x']
            error_y = rel_pos['y'] - 2.0

            vel = controller.velocity_monitor.get_velocity()
            vx, vy = vel['vx'], vel['vy']

            t = now - START_TIME
            log_file.write(
                f"{t:.3f},{leader_pos['x']:.3f},{leader_pos['y']:.3f},"
                f"{follower_pos['x']:.3f},{follower_pos['y']:.3f},"
                f"{error_x:.3f},{error_y:.3f},{vx:.3f},{vy:.3f},{loop_dt:.4f}\n"
            )
            log_file.flush()

            dt_loop = control_loop_period_sec if (control_loop_period_sec is not None and control_loop_period_sec > 0) else None
            controller.move_towards_with_pid(leader_pos, roll_pid, pitch_pid, dt=dt_loop)

            if dt_loop is not None:
                time.sleep(dt_loop)
    except KeyboardInterrupt:
        print("[Follower] Stopping due to KeyboardInterrupt")
    finally:
        log_file.close()
        print(f"[Follower] Log saved to {log_filename}")

def _read_webots_step_hz(num_instances=2):
    """Read /tmp/webots_step_<i>.txt timestamps and return current step rate (Hz) or None if not Webots."""
    global WEBOTS_LAST_TS
    hz_list = []
    for i in range(num_instances):
        path = f"/tmp/webots_step_{i}.txt"
        try:
            with open(path, "r") as f:
                t_curr = float(f.read().strip())
        except (FileNotFoundError, ValueError, OSError):
            continue
        if i in WEBOTS_LAST_TS:
            dt = t_curr - WEBOTS_LAST_TS[i]
            if dt > 0:
                hz_list.append(1.0 / dt)
        WEBOTS_LAST_TS[i] = t_curr
    if not hz_list:
        return None
    return min(hz_list)  # bottleneck across instances


def coordinate_exchange_loop(controllers):
    """
    Цикл обмена координатами между дронами.
    
    Args:
        controllers: список объектов DroneController
    """
    print("Starting coordinate exchange loop")
    global EXCHANGE_HZ_HISTORY

    while True:
        t0 = time.time()
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

        # Exchange loop period and rate for visualizer
        dt_exchange = time.time() - t0
        if dt_exchange > 0:
            EXCHANGE_HZ_HISTORY.append(1.0 / dt_exchange)
            if len(EXCHANGE_HZ_HISTORY) > EXCHANGE_HZ_WINDOW:
                EXCHANGE_HZ_HISTORY = EXCHANGE_HZ_HISTORY[-EXCHANGE_HZ_WINDOW:]
            RATES_SHARED["exchange_hz"] = sum(EXCHANGE_HZ_HISTORY) / len(EXCHANGE_HZ_HISTORY)
        RATES_SHARED["webots_step_hz"] = _read_webots_step_hz(num_instances=len(controllers))

        # Публикация в визуализатор (если запущен drone_position_visualizer.py)
        if HAS_VISUALIZER_PUBLISHER:
            publish_positions(positions, rates=RATES_SHARED.copy())

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
    """Основная функция. Поддержка аргументов для экспериментов: --kp, --kd, --control-hz, --run-id."""
    import argparse
    parser = argparse.ArgumentParser(description="Leader-follower experiment (see docs/STRATEGY.md)")
    parser.add_argument("--kp", type=float, default=8.0, help="P gain for follower PID (default 8 so follower can catch up in X)")
    parser.add_argument("--ki", type=float, default=0.0, help="I gain")
    parser.add_argument("--kd", type=float, default=10.0, help="D gain (damping); default 10 for stable following")
    parser.add_argument("--control-hz", type=float, default=50.0, help="Control loop rate in Hz (default 50); use 0 to disable rate limit")
    parser.add_argument("--leader-duration", type=float, default=10.0, help="Leader forward movement duration in seconds (default 10)")
    parser.add_argument("--run-id", type=str, default=None, help="Suffix for log file (e.g. sitl_only, kd10, webots)")
    args = parser.parse_args()
    args.control_loop_period_sec = (1.0 / args.control_hz if args.control_hz and args.control_hz > 0 else None)
    args.log_run_id = args.run_id

    print("=== Drone Following System ===")
    print(f"Experiment: kp={args.kp}, ki={args.ki}, kd={args.kd}, control_hz={args.control_hz}, run_id={args.run_id}")

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
        global START_TIME
        START_TIME = time.time()

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
        leader_duration = getattr(args, 'leader_duration', 10.0)
        leader_thread = threading.Thread(
            target=leader_pattern,
            args=(leader, leader_duration),
            daemon=True
        )
        leader_thread.start()
        print("Leader thread started")
        
        # Параметры эксперимента из аргументов (для тестов по docs/STRATEGY.md)
        follower_loop(
            follower, leader.config['id'],
            kp=getattr(args, 'kp', 1.0),
            ki=getattr(args, 'ki', 0.0),
            kd=getattr(args, 'kd', 0.0),
            control_loop_period_sec=getattr(args, 'control_loop_period_sec', None),
            log_run_id=getattr(args, 'log_run_id', None),
        )

        # while True:
        #     leader_velocity = leader.velocity_monitor.get_velocity()
        #     follower_velocity = follower.velocity_monitor.get_velocity()
        #     print(f"Leader velocity: {leader_velocity}")
        #     print(f"Follower velocity: {follower_velocity}")
        
    except KeyboardInterrupt:
        print("\nStopping all drones...")
        for controller in controllers:
            controller.stop()
            controller.master.set_mode(9)  # LAND
        print("All drones stopped")

if __name__ == "__main__":
    main()

