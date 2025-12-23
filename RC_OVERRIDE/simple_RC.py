from pymavlink import mavutil
import time

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

try:
    # === Тестирование в режиме POS_HOLD ===

    print("\n=== Testing POS_HOLD mode ===")
    # LOITER mode = 5, POS_HOLD mode = 16 (устаревший)
    master.set_mode(16)  # POS_HOLD (2 - ALT_HOLD)
    
    neutral = 1500 # Нейтральное значение ШИМ канала

    while True:
        # Движение вперёд
        start_time = time.time()
        while time.time() - start_time < 5: # 5 секунд движения вперёд
            send_rc_override(master, neutral, 1700, neutral, neutral)
            time.sleep(1)
        
        # стабилизация
        start_time = time.time()
        while time.time() - start_time < 5: # 5 секунд движения вперёд
            send_rc_override(master, neutral, neutral, neutral, neutral)
            time.sleep(1)

        # Назад
        start_time = time.time()
        while time.time() - start_time < 5: # 5 секунд движения назад
            send_rc_override(master, neutral, 1300, neutral, neutral)
            time.sleep(1)

        # стабилизация
        start_time = time.time()
        while time.time() - start_time < 5: # 5 секунд движения вперёд
            send_rc_override(master, neutral, neutral, neutral, neutral)
            time.sleep(1)

except KeyboardInterrupt:
    # При выходе возвращаем нейтральные RC значения
    send_rc_override(master, neutral, neutral, neutral, neutral)
    print("Setting mode LAND")
    master.set_mode(9)
    #master.arducopter_disarm()
    print("Stopped and neutralized RC channels")