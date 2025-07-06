from pymavlink import mavutil
import time

# Порты подключения к дронам (ПОКА HARD-CODE)
PORTS = ["udpin:127.0.0.1:14550", "udpin:127.0.0.1:14551"]

# Подключение ко всем дронам
drones = []

for port in PORTS:
    print(f"Connecting to {port}...")
    drone = mavutil.mavlink_connection(port)

    print("Waiting for heartbeat...")
    drone.wait_heartbeat()
    
    target_system = drone.target_system
    target_component = drone.target_component
    print(f"Heartbeat from system (sysid={target_system}, compid={target_component})")

    drones.append(drone)

print("All drones connected.")

# === Функция для перевода дрона в режим GUIDED ===
def set_guided_mode(mav):
    # Переключение в GUIDED (полёт по командам)
    mav.set_mode('GUIDED')
    print("Mode change to GUIDED requested")

# === Армирование дронов ===
def arm_drone(mav):
    mav.arducopter_arm()
    print("Arm command sent")

# === Отправка команд всем дронам ===
def send_command_to_all(command_func):
    for idx, drone in enumerate(drones):
        print(f"Sending command to drone {idx}")
        command_func(drone)

# === Проверка, что команда принята === #
def wait_command_ack(mav, command, timeout=10):
    tstart = time.time()
    while time.time() < tstart + timeout:
        ack = mav.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
        if ack is not None and ack.command == command:
            print(f"Got COMMAND_ACK: {ack.result}")
            return ack.result
    print("Command ACK timeout")
    return None

# Команда: Takeoff
def takeoff(mav, altitude=5):
    print(f"Requesting takeoff to {altitude} meters")
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,  # Номер попытки подтверждения получения команды 
        0,  # Параметр 1 - минимальная положительная подъёмная сила (не используется)
        0, 0, 0,  # Параметры 2-4: пустые
        0, 0,     # Параметры 5-6: пустые
        altitude  # Параметр 7: высота
    )
    wait_command_ack(mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

# Пример использования
time.sleep(5)  # Ждём полной инициализации
send_command_to_all(set_guided_mode)
time.sleep(5)
send_command_to_all(arm_drone)
time.sleep(10)
send_command_to_all(takeoff)  # взлететь на 5 метров