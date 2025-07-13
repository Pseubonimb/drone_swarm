from pymavlink import mavutil
import time
import math
import asyncio

async def generate_circle_mission(center_lat, center_lon, num_points, altitude, radius=50):
    mission = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        lat = center_lat + (radius / 111319.5) * math.cos(angle)
        lon = center_lon + (radius / (111319.5 * math.cos(math.radians(center_lat)))) * math.sin(angle)
        mission.append((lat, lon, altitude))
    return mission


async def send_waypoint(mav, seq, command, lat, lon, alt, param1=0, param2=0, param3=0, param4=0, current=0, autocontinue=1):
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
    print(f"Sent waypoint {seq} command {command} to drone {mav.target_system}")
    await asyncio.sleep(0.05) # Уменьшенная задержка


async def upload_mission(mav, waypoints):
    # Очистка текущей миссии
    mav.mav.mission_clear_all_send(mav.target_system, mav.target_component)
    print(f"Cleared mission for drone {mav.target_system}")
    await asyncio.sleep(0.5)

    mission_length = len(waypoints) #+ 2  # +2 для Takeoff и Land

    # Отправляем количество точек
    mav.mav.mission_count_send(mav.target_system, mav.target_component, mission_length)
    print(f"Sending mission count {mission_length} for drone {mav.target_system}")
    await asyncio.sleep(0.5)

    seq = 0

    '''### ВЗЛЁТ Для эксперимента
    lat0, lon0, alt0 = waypoints[0]
    await send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, alt0, param1=alt0)
    seq += 1'''

    for lat, lon, alt in waypoints:
        await send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, lat, lon, alt)
        seq += 1

    '''### ПОСАДКА Для эксперимента
    lat_end, lon_end, _ = waypoints[-1]
    await send_waypoint(mav, seq, mavutil.mavlink.MAV_CMD_NAV_LAND, lat_end, lon_end, 0)
    print(f"Mission uploaded with takeoff and landing to drone {mav.target_system}")'''


async def set_mode(mav, mode_name):
    mode_id = mav.mode_mapping().get(mode_name)
    if mode_id is None:
        raise ValueError(f"Unknown mode: {mode_name}")
    mav.set_mode(mode_id)
    print(f"Set mode to {mode_name} for drone {mav.target_system}")
    await asyncio.sleep(1) # Подождать, чтобы режим установился


async def wait_command_ack(mav, command, timeout=10):
    """
    Асинхронно ждёт подтверждение COMMAND_ACK для указанной команды.
    """
    tstart = time.time()
    sysid = mav.target_system # Для логирования, какой дрон ждет ACK
    print(f"Drone {sysid}: Waiting for COMMAND_ACK for command {command}...")

    while time.time() < tstart + timeout:
        # Используем asyncio.to_thread для выполнения блокирующего recv_match в отдельном потоке.
        # Это предотвращает блокировку основного цикла событий asyncio.
        ack = await asyncio.to_thread(mav.recv_match, type='COMMAND_ACK', blocking=True, timeout=1)

        if ack is not None:
            print(f"Drone {sysid}: Received COMMAND_ACK (command: {ack.command}, result: {ack.result})")
            if ack.command == command: # Проверяем, что дрону пришла нужная команда
                if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print(f"Drone {sysid}: Command {command} ACCEPTED.")
                    return True
                else:
                    print(f"Drone {sysid}: Command {command} REJECTED with result: {ack.result}")
                    return False
        
        # Если ACK не получен или не совпадает с ожидаемым, ждем немного и повторяем
        await asyncio.sleep(0.1) # Средняя частота отправки MAVLink-сообщений порядка 5-10 Гц

    print(f"Drone {sysid}: COMMAND_ACK timeout for command {command}.")
    return False # Таймаут, команда не подтверждена успешно


async def wait_for_altitude(mav, target_alt, tolerance=0.5, timeout=60):
    """
    Функция ожидания достижения определенной высоты
    """
    start_time = time.time()
    sysid = mav.target_system # Для логирования высоты каждого дрона
    print(f"Drone {sysid}: Waiting for drone to reach altitude {target_alt}m...")

    while True:
        if time.time() - start_time > timeout:
            print(f"Drone {sysid}: Timeout waiting for altitude.")
            return False

        msg = await asyncio.to_thread(mav.recv_match, type='GLOBAL_POSITION_INT', blocking=True, timeout=1)

        if msg:
            current_alt = 0 # Инициируем переменную нулём, чтобы не выполнилось условие достижения заданной высоты при ошибке
            if msg.get_type() == 'GLOBAL_POSITION_INT':
                current_alt = msg.relative_alt / 1000.0  # relative_alt в мм, переводим в метры

            print(f"Drone {sysid}: Current altitude: {current_alt:.2f}m")
            if abs(current_alt - target_alt) < tolerance:
                print(f"Drone {sysid}: Altitude reached: {current_alt:.2f}m")
                return True
        await asyncio.sleep(0.1) # Средняя частота отправки MAVLink-сообщений порядка 5-10 Гц


async def wait_for_mission_completion(mav, total_waypoints_in_mission, timeout=300):
    """
    Функция ожидания завершения текущей миссии (всех waypoint'ов)
    """
    start_time = time.time()
    sysid = mav.target_system # Для логирования хода выполнения миссии каждого дрона
    print(f"Drone {sysid}: Waiting for mission to complete (last waypoint: {total_waypoints_in_mission-1})...")
    last_seq = -1
    while True:
        if time.time() - start_time > timeout:
            print("Timeout waiting for mission completion.")
            return False

        msg = await asyncio.to_thread(mav.recv_match, type='MISSION_CURRENT', blocking=True, timeout=1)

        if msg:
            current_seq = msg.seq
            if current_seq != last_seq:
                print(f"Drone {sysid}: Current mission item: {current_seq}")
                last_seq = current_seq

            # Если текущий счётчик seq равен общему количеству точек в миссии, значит миссия завершена
            # (нумерация точек начинается с 0, поэтому финальная точка будет total_waypoints_in_mission-1)
            if current_seq >= total_waypoints_in_mission-1:
                print(f"Drone {sysid}: Mission completed.")
                return True
        await asyncio.sleep(0.1) # Средняя частота отправки MAVLink-сообщений порядка 5-10 Гц


async def drone_task(drone_mav, mission_waypoints, altitude):
    """
    Асинхронная задача для управления одним дроном.
    """
    sysid = drone_mav.target_system
    print(f"Starting task for drone {sysid}")

    # 1. Проверяем содержимое миссий
    if mission_waypoints is None:
        print(f"Drone {sysid}: No mission waypoints provided. Skipping mission upload and flight.")
        return

    # 2. Загрузка миссии
    print(f"Uploading mission to drone {sysid}")
    await upload_mission(drone_mav, mission_waypoints)
    await asyncio.sleep(1)

    # 3. Переключение в GUIDED
    await set_mode(drone_mav, 'GUIDED')

    # 4. Армирование
    print(f"Arming drone {sysid}...")
    # drone_mav.arducopter_arm() # Это синхронный вызов, но pymavlink сам отправляет MAVLink-команду
    drone_mav.mav.command_long_send(
        drone_mav.target_system,
        drone_mav.target_component,
        mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    await asyncio.sleep(3) # Ждём, пока моторы взведутся и дрон подтвердит

    # 5. Ожидаем подтверждение армирования
    start_time = time.time()
    armed_status = False
    while time.time() - start_time < 5: # 5 секунд на подтверждение
        msg = drone_mav.recv_match(type='HEARTBEAT', blocking=False)
        MAV_MODE_FLAG_ARMED = 128
        if msg and (msg.base_mode & MAV_MODE_FLAG_ARMED):
            print(f"Drone {sysid}: is ARMED!")
            armed_status = True
            break
        await asyncio.sleep(0.1)
    if not armed_status:
        print(f"Drone {sysid}: failed to ARM. Skipping.")
        return
    
    # 6. Взлёт (Takeoff)
    print(f"Drone {sysid}: Requesting takeoff to {altitude} meters")
    drone_mav.mav.command_long_send(drone_mav.target_system, drone_mav.target_component, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
    0,
    0,  # Параметр 1 - Минимальный шаг (при наличии датчика воздушной скорости), желаемый шаг без датчика
    0,  # Параметры 2-3: пустые
    0,
    0,  # Параметр 4 - YAW
    0,  # Параметр 5 - Latitude
    0,  # Параметр 6 - Longitude
    altitude)  # Параметр 7 - высота

    # Ожидаем подтверждения команды на взлёт
    await wait_command_ack(drone_mav, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF)

    # Проверка достижения заданной высоты
    alt_status = await wait_for_altitude(drone_mav, altitude)
    if not alt_status:
        print(f"Drone {sysid} failed to reach takeoff altitude. Aborting mission.")
        # Добавить логику для безопасной посадки или повторной попытки
        exit()

    print(f"Drone {sysid} Takeoff successful. Drone at {altitude}m.")

    # 7. Запуск миссии: одновременное переключение всех в AUTO
    # Этот шаг будет выполнен в основном потоке после подготовки всех дронов
    # Поэтому здесь просто вернемся, чтобы основной цикл мог их запустить
    print(f"Drone {sysid} ready for AUTO mode.")
    return


async def connect_drone(port):
    """
    Функция подключения к дрону.
    """
    print(f"Connecting to {port}...")
    drone = mavutil.mavlink_connection(port)

    print("Waiting for heartbeat...")
    await asyncio.to_thread(drone.wait_heartbeat)
    
    target_system = drone.target_system
    target_component = drone.target_component
    print(f"Heartbeat from system (sysid={target_system}, compid={target_component})")
    
    return drone


async def get_initial_position(drone_mav, timeout=5):
    """
    Асинхронно ждёт первое сообщение GLOBAL_POSITION_INT от одного дрона.
    Возвращает кортеж (lat, lon) в градусах или None при таймауте.
    """
    try:
        msg = await asyncio.to_thread(drone_mav.recv_match, type='GLOBAL_POSITION_INT', blocking=True, timeout=timeout)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            print(f"Drone {drone_mav.target_system}: initial position received: lat={lat}, lon={lon}")
            return (lat, lon)
    except Exception as e:
        print(f"Drone {drone_mav.target_system}: error while receiving position: {e}")
    print(f"Drone {drone_mav.target_system}: failed to get initial position within {timeout} seconds.")
    return None


async def get_all_initial_positions(drones, timeout=5):
    """
    Запускает параллельное ожидание начальных позиций от всех дронов.
    Возвращает список кортежей (lat, lon) или None для каждого дрона.
    """
    tasks = [get_initial_position(drone, timeout) for drone in drones]
    results = await asyncio.gather(*tasks) # Ждём пока соберутся данные о начальном положении со всех дронов
    return results


async def land_and_disarm(drone):
    """
    Функция посадки и дизармирования
    """
    sysid = drone.target_system
    print(f"Drone {sysid}: Switching to LAND mode.")

    # Переходим в режим LAND для запуска посадки
    await set_mode(drone, 'LAND')

    """
    drone.mav.command_long_send(
        drone.target_system,
        drone.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0,
        0, 0, 0, 0,
        0, 0, 0
    )
    await wait_command_ack(drone, mavutil.mavlink.MAV_CMD_NAV_LAND)
    """

    print(f"Drone {sysid}: Waiting for landing (altitude ~0).")
    landed = await wait_for_altitude(drone, 0, tolerance=0.2, timeout=60)
    if landed:
        await asyncio.sleep(2)
        print(f"Drone {sysid}: Landing successful. Disarming.")
        drone.mav.command_long_send(
            drone.target_system,
            drone.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0, 0, 0, 0, 0, 0, 0, 0
        )
    else:
        print(f"Drone {sysid}: Landing timeout! Consider emergency procedures.")


async def monitor_and_land(drone, num_points):
    """
    Функция отслеживания завершения миссии и запуска посадки
    """
    mission_complete = await wait_for_mission_completion(drone, num_points)
    if not mission_complete:
        print(f"Drone {drone.target_system}: Mission did not complete in time. Aborting.")
        return False

    print(f"Drone {drone.target_system}: Mission completed. Sending LAND command.")
    await land_and_disarm(drone)
    return True


# Основная асинхронная функция
async def main():
    # HARD-CODE порты
    PORTS = ["udpin:127.0.0.1:14550", "udpin:127.0.0.1:14560"]
    
    # Подключение ко всем дронам конкурентно
    connection_tasks = [connect_drone(port) for port in PORTS]
    drones_connected = await asyncio.gather(*connection_tasks)

    drones = [d for d in drones_connected if d is not None]
    if not drones:
        print("No drones connected. Exiting.")
        return

    # Генерируем миссию для каждого дрона
    missions = [] # Массив с миссиями типа (lat1, lon1), (lat2, lon2), (lat3, lon3), ...
    num_points = 4 # Количество точек для круга
    altitude = 5 # Высота, на которой дроны будут лететь по кругу

    # Ждем первого сообщения GLOBAL_POSITION_INT от КАЖДОГО дрона для генерации миссии
    initial_positions = await get_all_initial_positions(drones, timeout=5)

    for i, pos in enumerate(initial_positions):
        if pos is None:
            print(f"Drone {drones[i].target_system}: initial position not received.")
            missions.append(None)
        else:
            lat, lon = pos
            print(f"Drone {drones[i].target_system}: initial position is lat={lat}, lon={lon}")
            waypoints = await generate_circle_mission(lat, lon, num_points, altitude, radius=10)
            missions.append(waypoints)


    # Подготовка каждого дрона (загрузка миссии, GUIDED, ARM, Takeoff) конкурентно
    preparation_tasks = [drone_task(drone_mav, missions[i], altitude) for i, drone_mav in enumerate(drones)]
    await asyncio.gather(*preparation_tasks)

    print("\n--- All drones prepared. Sending AUTO mode command simultaneously ---")

    # Переключаем всех дронов в AUTO режим ОДНОВРЕМЕННО
    auto_mode_tasks = [set_mode(drone_mav, 'AUTO') for drone_mav in drones]
    await asyncio.gather(*auto_mode_tasks)

    print("\n--- All drones should now be in AUTO mode and starting mission. ---")

    # Отслеживание выполнения миссии или завершения полета
    results = await asyncio.gather(*(monitor_and_land(drone, num_points) for drone in drones))

    if all(results):
        print("All missions completed successfully.")
    else:
        print("Some missions failed or timed out.")
    


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("Script interrupted by user.")
    
    #####################################################################
'''
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

        # Проверка достижения нулевой высоты
        if not wait_for_altitude(drone, 0):
            print("Drone did not land properly.")
        else:
            print("Landing successful. Drone at 0m.")
'''
