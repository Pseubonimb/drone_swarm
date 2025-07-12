from pymavlink import mavutil
import time
import math

def generate_circle_mission(center_lat, center_lon, radius=50, num_points=8, altitude=10):
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

    # 1. Команда взлёта — обычно lat/lon совпадают с первой точкой, alt — высота взлёта
    lat0, lon0, alt0 = waypoints[0]
    send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, lat0, lon0, alt0, param1=alt0)
    seq += 1

    # 2. Основные waypoints
    for lat, lon, alt in waypoints:
        send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt)
        seq += 1

    # 3. Команда посадки — на последней точке, alt обычно 0
    lat_end, lon_end, _ = waypoints[-1]
    send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_LAND, lat_end, lon_end, 0)
    print("Mission uploaded with takeoff and landing")

def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        raise ValueError(f"Unknown mode: {mode_name}")
    mav.set_mode(mode_id)
    print(f"Set mode to {mode_name}")
    time.sleep(2)  # Подождать, чтобы режим установился



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

    mission_waypoints = generate_circle_mission(base_lat, base_lon, radius=50, num_points=8, altitude=10)

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

        # Переключаемся в AUTO для выполнения миссии
        set_mode(drone, 'AUTO')
        print(f"Drone {drone.target_system} set to AUTO mode")

