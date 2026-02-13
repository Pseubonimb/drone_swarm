import os
import math
import subprocess
import time
import argparse

# === Пути проекта ===
project_root = os.path.dirname(os.path.abspath(__file__))
APM_HOME = "../ardupilot"
SIM_VEHICLE_PATH = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")
mavproxy_dir = os.path.join(project_root, "MAVProxy")

# === Настройки ===
WORLDS_DIR = 'worlds'
INPUT_WORLD = os.path.join(WORLDS_DIR, 'irisAuto.wbt')
OUTPUT_WORLD = os.path.join(WORLDS_DIR, 'temp_world.wbt')

WEBOTS_PROTO_PATH = os.path.join(project_root, 'protos')

DRONE_TEMPLATE = '''Iris {{
  translation {x} {y} {z}
  rotation 0 1 0 0
  name "Iris_{id}"
  controller "ardupilot_vehicle_controller"
  controllerArgs [
    "--instance" "{id}"
    "--motors" "m1_motor, m2_motor, m3_motor, m4_motor"
  ]
}}
'''

# === Функция: генерация позиций дронов ===
def generate_position(index, drones_per_row=10, dx=2.0, dy=2.0, z=0.0549632125):
    row = index % drones_per_row
    col = index // drones_per_row
    x = col * dx
    y = row * dy
    return x, y, z

# === Функция: изменение .wbt файла ===
def modify_world_file(input_path, output_path, num_drones):
    with open(input_path, 'r') as f:
        content = f.read()

    insert_marker = '# Insert drones'
    insert_pos = content.find(insert_marker) + len(insert_marker)

    if insert_pos == -1:
        raise ValueError("Метка для вставки дронов не найдена в файле мира.")

    drones = []
    for i in range(num_drones):
        x, y, z = generate_position(i)
        drone_str = DRONE_TEMPLATE.format(x=x, y=y, z=z, id=i)
        drones.append(drone_str)

    new_content = content[:insert_pos] + '\n' + ''.join(drones) + '\n' + content[insert_pos:]

    with open(output_path, 'w') as f:
        f.write(new_content)

    print(f"Создан временный мир с {num_drones} дронами: {output_path}")

# === Функция: запуск Webots ===
def launch_webots(world_path):
    webots_executable = 'webots'
    env = os.environ.copy()
    env["WEBOTS_PROTO_PATH"] = WEBOTS_PROTO_PATH
    print("Запуск Webots...")
    return subprocess.Popen([webots_executable, world_path], env=env)

# === Функция: запуск SITL для одного дрона ===
def start_sim_vehicle(instance, tcp_port, udp_port, params=None):
    args = [
        SIM_VEHICLE_PATH,
        "-v", "ArduCopter",
        "-w",  # Сброс параметров
        "--model", "webots-python",
        f"--instance={instance}",
        f"--sysid={instance+1}", ##########!!!!!!!!!!!!!!! Системный id всегда больше instance на 1 !!!!!!!!!!!!!!
        f"--out=127.0.0.1:{tcp_port}",  # TCP для MAVProxy
        f"--out=127.0.0.1:{udp_port}",  # Для swarm_controller.py (UDP)
        "--console",
        #"--map"
    ]
    if params:
        args += params

    print(f"Запуск SITL instance {instance} на порту {tcp_port}")
    return subprocess.Popen(args)



'''
# === Функция: запуск MAVProxy для одного дрона ===
def start_mavproxy(tcp_port, udp_port):
    cmd = (
        f"mavproxy.py "
        f"--master=tcp:127.0.0.1:{tcp_port} "
        f"--out=udp:127.0.0.1:{udp_port} "
        f"--console --map"
    )
    args = ["gnome-terminal", "--", "bash", "-c", f"{cmd}; exec bash"]
    print(f"Запуск MAVProxy для порта TCP {tcp_port} → UDP {udp_port}")
    return subprocess.Popen(args)'''

# === Основной запуск ===
if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Запуск мира Webots с N дронами и SITL')
    parser.add_argument('--drones', type=int, default=2, help='Количество дронов')
    args = parser.parse_args()

    # === 1. Генерация мира ===
    modify_world_file(INPUT_WORLD, OUTPUT_WORLD, args.drones)

    # === 2. Запуск Webots ===
    webots_process = launch_webots(OUTPUT_WORLD)
    time.sleep(5)

    # === 3. Подготовка конфигурации дронов ===
    processes = []

    BASE_TCP_PORT = 5770
    BASE_UDP_PORT = 14551

    for i in range(args.drones):
        tcp_port = BASE_TCP_PORT + i * 10
        udp_port = BASE_UDP_PORT + i * 10

        # Запуск SITL
        sitl_proc = start_sim_vehicle(
            instance=i,
            tcp_port=tcp_port,
            udp_port=udp_port,
            params=["--add-param-file=config/iris.parm"]
        )
        print(sitl_proc)
        processes.append(sitl_proc)
        time.sleep(5)



    # === 4. Запуск контроллера роя ===
    # print("Запуск swarm_controller.py...")
    # swarm_proc = subprocess.Popen(["python", "swarm_controller.py", "--drones", str(args.drones)])
    # processes.append(swarm_proc)

    # === 5. Ожидание завершения работы ===
    try:
        webots_process.wait()
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("Остановка процессов...")
        for p in processes:
            p.terminate()
        webots_process.terminate()