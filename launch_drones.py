import subprocess
import time
import os

# Путь к ArduPilot SITL
APM_HOME = "/home/user/Documents/Kursach/ardupilot"
SIM_VEHICLE_PATH = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")

# Настройки дронов
DRONES = [
    {
        "instance": 0,
        "port": "5760",
        "params": ["--add-param-file=config/iris.parm"]
    },
    {
        "instance": 1,
        "port": "5770",
        "params": ["--add-param-file=config/iris.parm"]
    }
]

def launch_drone(config):
    args = [
        SIM_VEHICLE_PATH,
        "-v", "ArduCopter",
        "-w",
        "--model", "webots-python",
        f"--instance={config['instance']}",
        f"--out=127.0.0.1:{config['port']}"
    ] + config["params"]

    print(f"Launching drone instance {config['instance']} on port {config['port']}")
    subprocess.Popen(args)

if __name__ == "__main__":
    for drone in DRONES:
        launch_drone(drone)
        time.sleep(3)  # Небольшая пауза между запусками