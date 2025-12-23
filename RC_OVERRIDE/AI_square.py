from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('udp:127.0.0.1:14550')
master.wait_heartbeat()
print("Connected to drone")

# Устанавливаем режим GUIDED для полной стабилизации и внешнего управления
master.set_mode(4)
time.sleep(1)

# Функция отправки RC_OVERRIDE (или RC_CHANNELS_OVERRIDE)
def send_rc_override(drone, roll, pitch, throttle, yaw):
    # roll/aileron, pitch/elevator, throttle, yaw/rudder
    drone.mav.rc_channels_override_send(
        drone.target_system,
        drone.target_component,
        roll,
        pitch,
        throttle,
        yaw,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# Стандартные значения RC: 1500 = нейтраль, 1000 = мин, 2000 = макс
neutral = 1500
throttle_hover = 1600  # чуть выше нейтрали для подъема
turn_yaw_right = 1800  # на 90 градусов поворот вправо
turn_yaw_neutral = 1500

# Прямоугольник в 4 шага:
# Каждое движение — примерно 2 секунды, поворот — 1.5 секунды
square_steps = [
    (neutral, 1700, throttle_hover, turn_yaw_neutral),  # Вперёд
    (neutral, neutral, throttle_hover, turn_yaw_right), # Поворот
    (1700, neutral, throttle_hover, turn_yaw_neutral),  # Вправо (по дрессировке осей поменяйте если нужно)
    (neutral, neutral, throttle_hover, turn_yaw_right), # Поворот
    (neutral, 1300, throttle_hover, turn_yaw_neutral),  # Назад
    (neutral, neutral, throttle_hover, turn_yaw_right), # Поворот
    (1300, neutral, throttle_hover, turn_yaw_neutral),  # Влево
    (neutral, neutral, throttle_hover, turn_yaw_right)  # Поворот
]

step_durations = [2, 1.5, 2, 1.5, 2, 1.5, 2, 1.5]

try:
    while True:
        for (roll, pitch, throttle, yaw), duration in zip(square_steps, step_durations):
            print(f"Moving: roll={roll}, pitch={pitch}, throttle={throttle}, yaw={yaw}, for {duration} seconds")

            start_time = time.time()
            while time.time() - start_time < duration:
                send_rc_override(master, roll, pitch, throttle, yaw)
                time.sleep(0.1)

            # После каждого шага (движения или поворота) даём время для выравнивания
            print("Self-leveling...")
            start_time = time.time()
            while time.time() - start_time < 1.0:
                send_rc_override(master, neutral, neutral, throttle_hover, turn_yaw_neutral)
                time.sleep(0.1)

except KeyboardInterrupt:
    # Безопасно нулевые команды и посадка
    print("Landing and neutralizing...")
    send_rc_override(master, neutral, neutral, neutral, neutral)
    master.set_mode(9)  # LAND
