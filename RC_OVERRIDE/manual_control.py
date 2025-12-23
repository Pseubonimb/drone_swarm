from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')

master.wait_heartbeat()
print("Heartbeat получен!!!")

print("Setting mode GUIDED")
mode_id = master.mode_mapping().get("GUIDED")
print(mode_id)
master.set_mode(mode_id)

print(f"Arming drone {master.sysid}...")
master.arducopter_arm() # Это синхронный вызов, но pymavlink сам отправляет MAVLink-команду
# master.mav.command_long_send(
#     master.target_system,
#     master.target_component,
#     mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
#     0, 1, 0, 0, 0, 0, 0, 0
# )
time.sleep(5)

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

# Отправляем команду MANUAL_CONTROL несколько секунд
duration_sec = 2
end_time = time.time() + duration_sec

print("Setting mode STABILIZE")
mode_id = master.mode_mapping().get("STABILIZE")
print(mode_id)
master.set_mode(mode_id)

while time.time() < end_time:
    # Параметры команды MANUAL_CONTROL:
    # target_system, x, y, z, r, buttons
    # x, y, r в диапазоне [-1000, 1000], z (газ) в [0, 1000], 500 - нейтральное положение газа
    master.mav.manual_control_send(
        #master.target_system,
        1,
        300,    # x - 0 означает нейтральное положение по оси X
        0,  # y - значение 300
        300,  # z - throttle, 500 нейтральный газ
        0,    # r - поворот, 0 нейтрально
        0     # кнопки, 0 - без нажатий
    )
    # print("Послано")
    time.sleep(0.1)  # 10 Гц - частота отправки

print("Setting mode GUIDED")
mode_id = master.mode_mapping().get("GUIDED")
print(mode_id)
master.set_mode(mode_id)