import threading
import time
from pymavlink import mavutil
import sys

# === Класс для получения координат дрона ===
class CoordsMonitor:
    """
    Мониторинг координат дрона через MAVLink LOCAL_POSITION_NED сообщение.
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
                msg = self.drone.recv_match(type='LOCAL_POSITION_NED', blocking=True)
                #print("Waiting msg")

                # Проверка взята с сайта https://mavlink.io/en/mavgen_python/#receiving-messages
                if not msg:
                    print("NOPE..............")
                    return
                if msg.get_type() == "BAD_DATA":
                    if mavutil.all_printable(msg.data):
                        print(f"BRUH {msg.data}")
                        # sys.stdout.write(msg.data)
                        # sys.stdout.flush()
                else:
                    with self.lock:
                        # В LOCAL_POSITION_NED: x, y, z - координаты в метрах (NED система координат)
                        # x - север (positive north)
                        # y - восток (positive east)
                        # z - вниз (positive down)
                        self.position_ned['x'] = msg.x #Ось x в Webots направлена в направлении кормы дрона
                        self.position_ned['y'] = msg.y
                        self.position_ned['z'] = msg.z

            except Exception as e:
                # Тихий режим - не выводим ошибки постоянно
                pass
            # time.sleep(0.05)  # Обновление каждые 50мс !!! Это вносит критические задержки реакции!!!
    
    def get_position(self):
        """Получить текущие координаты."""
        with self.lock:
            return self.position_ned.copy()
    
    def get_distance_to(self, target_position):
        """
        Вычислить расстояние до целевой позиции.
        
        Args:
            target_position: словарь с ключами 'x', 'y', 'z' - целевая позиция
        
        Returns:
            Расстояние в метрах
        """
        with self.lock:
            dx = target_position['x'] - self.position_ned['x']
            dy = target_position['y'] - self.position_ned['y']
            dz = target_position['z'] - self.position_ned['z']
            return (dx**2 + dy**2 + dz**2)**0.5
    
    def get_relative_position(self, target_position):
        """
        Получить относительную позицию относительно целевой точки.
        
        Args:
            target_position: словарь с ключами 'x', 'y', 'z' - целевая позиция
        
        Returns:
            Словарь с относительными координатами {'x': dx, 'y': dy, 'z': dz}
        """
        with self.lock:
            return {
                'x': target_position['x'] - self.position_ned['x'],
                'y': target_position['y'] - self.position_ned['y'],
                'z': target_position['z'] - self.position_ned['z']
            }
