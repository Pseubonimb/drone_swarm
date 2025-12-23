import os
import subprocess
import argparse
import math

# Константы
project_root = os.path.dirname(os.path.abspath(__file__))
WORLDS_DIR = 'worlds'
INPUT_WORLD = os.path.join(WORLDS_DIR, 'irisAuto.wbt')
OUTPUT_WORLD = 'temp_world.wbt'

WEBOTS_PROTO_PATH = os.path.join(project_root, 'protos')

# Шаблон дрона Iris
DRONE_TEMPLATE = '''Iris {{
  translation {x} {y} {z}
  rotation {rx} {ry} {rz} {r}
  name "Iris_{id}"
  controller "ardupilot_vehicle_controller"
  controllerArgs [
    "--instance" "{id}"
    "--motors" "m1_motor, m2_motor, m3_motor, m4_motor"
  ]
}}
'''


def generate_position(index, drones_per_row=10, dx=2.0, dy=2.0, z=0.0549632125):
    #Генерация позиций для дронов в сетке.

    row = index // drones_per_row  # Номер ряда
    col = index % drones_per_row   # Позиция в ряду

    x = col * dx                   # Сдвиг по x внутри ряда
    y = row * dy                   # Сдвиг по y для нового ряда

    return x, y, z

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
        drone_str = DRONE_TEMPLATE.format(
            x=x,
            y=y,
            z=z,
            rx=0,
            ry=0,
            rz=1,
            r=0.0007614600697470413,
            id=i+1
        )
        drones.append(drone_str)

    new_content = content[:insert_pos] + '\n' + ''.join(drones) + '\n' + content[insert_pos:]

    with open(output_path, 'w') as f:
        f.write(new_content)

    print(f"Создан временный мир с {num_drones} дронами: {output_path}")

'''def launch_webots(world_path):
    webots_executable = 'webots'  # Убедись, что Webots в PATH
    cmd = [webots_executable, world_path]
    print("Запуск Webots...")
    subprocess.run(cmd)'''


def launch_webots(world_path, log_file=None):
    webots_executable = 'webots'
    args = [webots_executable, '--batch', '--no-rendering', '--mode=fast', world_path]

    env = os.environ.copy()
    env["WEBOTS_PROTO_PATH"] = WEBOTS_PROTO_PATH

    print("Запуск Webots в headless-режиме...")

    if log_file:
        with open(log_file, 'w') as f:
            process = subprocess.Popen(args, env=env, stdout=f, stderr=subprocess.STDOUT)
    else:
        process = subprocess.Popen(args, env=env)

    return process


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Запуск мира Webots с N дронами Iris')
    parser.add_argument('--drones', type=int, default=5, help='Количество дронов')
    args = parser.parse_args()

    modify_world_file(INPUT_WORLD, OUTPUT_WORLD, args.drones)
    launch_webots(OUTPUT_WORLD, log_file='simulation_output.log')