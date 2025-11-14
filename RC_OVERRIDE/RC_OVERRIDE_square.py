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

# Значения для управления квадратно
neutral = 1500
throttle_hover = 1500  # Значение газа для поддержания высоты

square_steps = [
    (neutral, 1700, throttle_hover, neutral),  # Вперёд
    (1700, neutral, throttle_hover, neutral),  # Вправо
    (neutral, 1300, throttle_hover, neutral),  # Назад
    (1300, neutral, throttle_hover, neutral)  # Влево
]

step_duration = 2  # Секунд на каждый сегмент квадрата



try:
    while True:
        print("Setting mode POS_HOLD")
        master.set_mode(16)

        # ВПЕРЁД!!!!!!!!!!!!!!!!!!!
        start = time.time()
        while time.time() - start < 5:
            send_rc_override(master, neutral, 1700, throttle_hover, neutral)
            time.sleep(0.1)

        # возвращаем нейтральные RC значения
        print("self-leveling")
        start = time.time()
        while time.time() - start < 10:
            send_rc_override(master, neutral, neutral, neutral, neutral)
            time.sleep(0.1)
        
        
        # НАЗАД!!!!!!!!!!!
        start = time.time()
        while time.time() - start < 5:
            send_rc_override(master, neutral, 1300, throttle_hover, neutral)
            time.sleep(0.1)

        # возвращаем нейтральные RC значения
        print("self-leveling")
        start = time.time()
        while time.time() - start < 10:
            send_rc_override(master, neutral, neutral, neutral, neutral)
            time.sleep(0.1)

        # print("Setting mode ALT_HOLD")
        # master.set_mode(2)

        # # Держим этот шаг заданное время, отправляя команды каждые 0.1 сек
        # start = time.time()
        # while time.time() - start < 5:
        #     send_rc_override(master, neutral, 1700, throttle_hover, neutral)
        #     time.sleep(0.1)

        # # возвращаем нейтральные RC значения
        # print("self-leveling")
        # start = time.time()
        # while time.time() - start < 10:
        #     send_rc_override(master, neutral, neutral, neutral, neutral)
        #     time.sleep(0.1)

    # while True:
    #     for step in square_steps:
    #         print("Setting mode POS_HOLD")
    #         master.set_mode(16)
    #         print(f"Moving: roll={step[0]}, pitch={step[1]}")

    #         # Отправляем команду
    #         send_rc_override(master, *step)

    #         # Держим этот шаг заданное время, отправляя команды каждые 0.1 сек
    #         start = time.time()
    #         while time.time() - start < step_duration:
    #             send_rc_override(master, *step)
    #             time.sleep(0.1)

    #         # возвращаем нейтральные RC значения
    #         print("self-leveling")
    #         start = time.time()
    #         while time.time() - start < step_duration:
    #             send_rc_override(master, neutral, neutral, neutral, neutral)
    #             time.sleep(0.1)

except KeyboardInterrupt:
    # При выходе возвращаем нейтральные RC значения
    send_rc_override(master, neutral, neutral, throttle_hover, neutral)
    print("Setting mode LAND")
    master.set_mode(9)
    #master.arducopter_disarm()
    print("Stopped and neutralized RC channels")
