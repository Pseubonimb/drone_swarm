import subprocess
import time
import os

# --- Установка PYTHONPATH ---
# Это важно сделать до импорта pymavlink или MAVProxy

project_root = os.path.dirname(os.path.abspath(__file__))
mavproxy_dir = os.path.join(project_root, "MAVProxy")

os.environ["PYTHONPATH"] = mavproxy_dir
print(f"PYTHONPATH установлен: {os.environ['PYTHONPATH']}")

# --- Теперь можно безопасно импортировать pymavlink и MAVProxy ---
from pymavlink import mavutil

# Путь к ArduPilot SITL
APM_HOME = "/home/user/Documents/Kursach/ardupilot"
SIM_VEHICLE_PATH = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py") # Путь до "мозга" дрона

# Конфигурация дронов (ПОКА HARD-CODE)
DRONES = [
    {
        "instance": 0,
        "tcp_port": 5760, # TCP-порт ДРОНА для MAVProxy
        "udp_port": 1451, # UDP-input-порт MavProxy для swarm_controller.py
        # (UDP-порты 9002 и 9003 стандартно используются SITL для взаимодействия с Webots)
        "sysid": 1,
        "params": ["--add-param-file=config/iris.parm"]
    },
    {
        "instance": 1,
        "tcp_port": 5770, # TCP-порт ДРОНА для MAVProxy
        "udp_port": 1461, # UDP-input-порт MavProxy для swarm_controller.py
        # (UDP-порты 9012 и 9013 стандартно используются SITL для взаимодействия с Webots)
        "sysid": 2,
        "params": ["--add-param-file=config/iris.parm"]
    }
]

def start_sim_vehicle(config): # Запустить "мозг" дрона
    args = [
        SIM_VEHICLE_PATH,
        "-v", "ArduCopter",
        "-w",
        "--model", "webots-python",
        f"--instance={config['instance']}",
        f"--out=127.0.0.1:{config['tcp_port']}",
        f"--out=127.0.0.1:{config['udp_port']}"
    ] + config["params"]

    print(f"Launching drone instance {config['instance']} on ports {config['tcp_port']}")# and {config['udp_port']}")
    return subprocess.Popen(args)

def start_mavproxy(config): # Запуск MAVProxy в новом окне
    args = [
        "gnome-terminal",
        "--",
        "bash", "-c",
        f"mavproxy.py --master=tcp:127.0.0.1:{config['tcp_port']} "
        f"--out=udpin:127.0.0.1:{config['udp_port']} "
        f"--console; exec bash"
    ]

    print(f"Starting MAVProxy for drone {config['instance']}")
    return subprocess.Popen(args)

if __name__ == "__main__":
    processes = []

    # Запуск дронов
    for drone in DRONES:
        p = start_sim_vehicle(drone)
        processes.append(p)
        time.sleep(3)  # Пауза между запусками

    # Запуск MAVProxy
    for drone in DRONES:
        p = start_mavproxy(drone)
        processes.append(p)
        time.sleep(2)

    # Запуск контроллера роя
    print("Starting swarm controller...")
    swarm_process = subprocess.Popen(["python", "swarm_controller.py"])
    processes.append(swarm_process)

    # Ждём завершения (Ctrl+C)
    try:
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("Shutting down all processes...")
        for p in processes:
            p.terminate()