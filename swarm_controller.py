from pymavlink import mavutil
import time
import math

def generate_circle_mission(center_lat, center_lon, altitude, num_points, radius):
    mission = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        lat = center_lat + (radius / 111319.5) * math.cos(angle)
        lon = center_lon + (radius / (111319.5 * math.cos(math.radians(center_lat)))) * math.sin(angle)
        mission.append((lat, lon, altitude))
    return mission

def send_waypoint(mav, seq, command, lat, lon, alt, param1=0, param2=0, param3=0, param4=0, current=0, autocontinue=1):
    mav.mav.mission_item_int_send(
        mav.target_system,
        mav.target_component,
        seq,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,  # frame
        command,
        current,
        autocontinue,
        param1,
        param2,
        param3,
        param4,
        int(lat * 1e7),
        int(lon * 1e7),
        alt
    )
    print(f"Sent waypoint {seq} command {command} to drone")
    time.sleep(0.5)


def upload_mission(mav, waypoints):
    # Очистка текущей миссии
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    time.sleep(1)

    mission_length = len(waypoints) + 2  # +2 для Takeoff и Land

    # Отправляем количество точек
    mav.mav.mission_count_send(mav.target_system, mav.target_component, mission_length)
    time.sleep(1)

    seq = 0

    for lat, lon, alt in waypoints:
        send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt)
        seq += 1

    print("Mission uploaded")

def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        raise ValueError(f"Unknown mode: {mode_name}")
    mav.set_mode(mode_id)
    print(f"Set mode to {mode_name}")
    time.sleep(2)  # Подождать, чтобы режим установился

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

# Функция ожидания достижения определенной высоты
def wait_for_altitude(mav, target_alt, tolerance=0.5, timeout=60):
    start_time = time.time()
    print(f"Waiting for drone to reach altitude {target_alt}m...")
    while True:
        if time.time() - start_time > timeout:
            print("Timeout waiting for altitude.")
            return False

        msg = mav.recv_match(type=['GLOBAL_POSITION_INT', 'VFR_HUD'], blocking=True, timeout=1)
        if msg:
            current_alt = 0
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                current_alt = msg.relative_alt / 1000.0  # relative_alt в мм, переводим в метры
            elif msg.get_type() == 'VFR_HUD':
                current_alt = msg.alt  # alt в VFR_HUD уже в метрах

            print(f"Current altitude: {current_alt:.2f}m")
            if abs(current_alt - target_alt) < tolerance:
                print(f"Altitude reached: {current_alt:.2f}m")
                return True
        time.sleep(0.1) # Непрерывный опрос

# Функция ожидания завершения текущей миссии (всех waypoint'ов)
def wait_for_mission_completion(mav, total_waypoints_in_mission, timeout=300):
    start_time = time.time()
    print(f"Waiting for mission to complete (last waypoint: {total_waypoints_in_mission-1})...")
    last_seq = -1
    while True:
        if time.time() - start_time > timeout:
            print("Timeout waiting for mission completion.")
            return False

        msg = mav.recv_match(type=['MISSION_CURRENT'], blocking=True, timeout=1)
        if msg:
            current_seq = msg.seq
            if current_seq != last_seq:
                print(f"Current mission item: {current_seq}")
                last_seq = current_seq

            # Если текущий счётчик seq равен общему количеству точек в миссии, значит миссия завершена
            # (нумерация точек начинается с 0, поэтому финальная точка будет total_waypoints_in_mission-1)
            if current_seq >= total_waypoints_in_mission-1:
                print("Mission completed.")
                return True
        time.sleep(0.1)


# Порты подключения к дронам (ПОКА HARD-CODE)
PORTS = ["udpin:127.0.0.1:14550", "udpin:127.0.0.1:14560"]

# Подключение ко всем дронам
def connect_drones():
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
    return drones

if __name__ == "__main__":
    drones = connect_drones()

    # Ожидание сообщения GLOBAL_POSITION_INT
    base_drone = drones[0]
    global_pos_msg = base_drone.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
    if global_pos_msg is None:
        print("Failed to get position message. Exiting.")
        exit(1)
    base_lat = global_pos_msg.lat / 1e7
    base_lon = global_pos_msg.lon / 1e7

    # Высота взлёта
    altitude=20
    
    # Общее количество точек (без учёта взлёта и посадки)
    num_points=8

    # Генерируем точки полётного задания
    mission_waypoints = generate_circle_mission(base_lat, base_lon, altitude, num_points, radius=10)
    print(f"ТОЧЕЧКИ: {mission_waypoints}")

    for drone in drones:
        print(f"Uploading mission to drone {drone.target_system}")
        upload_mission(drone, mission_waypoints)
        time.sleep(1)

        # Переключаемся в GUIDED
        set_mode(drone, 'GUIDED')

        # Взводим моторы
        print(f"Arming drone {drone.target_system}...")
        drone.arducopter_arm()
        time.sleep(3)  # Ждём, пока моторы взведутся

        # Производим взлёт
        print(f"Requesting takeoff to {altitude} meters")
        drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0,  # Параметр 1 - Минимальный шаг (при наличии датчика воздушной скорости), желаемый шаг без датчика
        0,  # Параметры 2-3: пустые
        0,
        0,  # Параметр 4 - YAW
        0,  # Параметр 5 - Latitude
        0,  # Параметр 6 - Longitude
        altitude)  # Параметр 7 - высота

        # Ожидаем подтверждения команды на взлёт
        wait_command_ack(drone, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

        # Проверка достижения заданной высоты
        if not wait_for_altitude(drone, altitude):
            print("Drone failed to reach takeoff altitude. Aborting mission.")
            # Добавить логику для безопасной посадки или повторной попытки
            exit()

        print("Takeoff successful. Drone at 20m.")


        # Переключаемся в AUTO для выполнения миссии
        set_mode(drone, 'AUTO')
        print(f"Drone {drone.target_system} set to AUTO mode")

        # Ожидаем завершения миссии
        print("Drone set to AUTO mode. Waiting for mission completion.")
        if not wait_for_mission_completion(drone, num_points):
            print("Mission did not complete in time. Aborting.")
            # Добавить логику для безопасной посадки или повторной попытки
            exit()

        print("Mission completed successfully.")

        # Производим посадку
        print(f"Requesting landing")
        drone.mav.command_long_send(drone.target_system, drone.target_component, mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0,  # Параметр 1 - Минимальная целевая высота, если посадка отменена (0 = не определено/использовать системные настройки по умолчанию)
        0,  # Параметр 2 - Режим точного приземления
        0,  # Параметр 3 - Пустой
        0,  # Параметр 4 - YAW
        0,  # Параметр 5 - Latitude
        0,  # Параметр 6 - Longitude
        0)  # Параметр 7 - Высота посадки (уровень земли в текущем кадре)

        # Ожидаем подтверждения команды на посадку
        wait_command_ack(drone, mavutil.mavlink.MAV_CMD_NAV_LAND)



