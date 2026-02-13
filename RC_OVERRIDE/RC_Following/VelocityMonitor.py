import threading
import time

# Значение нейтрального положения для RC каналов
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
                        self.velocity_ned['vx'] = msg.vx / 100.0 # Скорость при движении вперёд была отрицательной, потому что при наклоне pitch вперёд - дрон в Webots летит НАЗАД!!!!!!
                        self.velocity_ned['vy'] = msg.vy / 100.0
                        #self.velocity_ned['vz'] = msg.vz / 100.0 if hasattr(msg, 'vz') else 0.0

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

