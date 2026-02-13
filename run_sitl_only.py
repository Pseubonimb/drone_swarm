#!/usr/bin/env python3
"""
Запуск двух экземпляров ArduPilot SITL без Webots (встроенная физика).

Цель: проверить, связана ли проблема «преследователь пролетает мимо лидера»
с симулятором Webots или с логикой/CoordsMonitor.

Использование:
  1. Из корня проекта: python run_sitl_only.py
  2. Дождаться запуска обоих SITL (порты 14551 и 14561).
  3. В другой консоли запустить тот же сценарий, что и с Webots, например:
     cd new_iteration_RC_OVERRIDE/experiment_stability
     python leader_forward_one_move.py
  4. Сравнить поведение: если без Webots преследователь не пролетает мимо —
     вероятно, виноват Webots; если пролетает — смотреть CoordsMonitor и регуляторы.
"""

import os
import subprocess
import time
import argparse
import sys

# Пути (как в run_sim.py)
project_root = os.path.dirname(os.path.abspath(__file__))
APM_HOME = os.path.join(project_root, "..", "ardupilot")
SIM_VEHICLE_PATH = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")


def start_sitl_instance(instance: int, udp_port: int, params=None):
    """
    Запуск одного SITL с встроенной физикой (без --model webots-python).
    MAVLink отдаётся на UDP для подключения Python-скрипта (CoordsMonitor, RC_OVERRIDE).
    """
    if not os.path.isfile(SIM_VEHICLE_PATH):
        print(f"Ошибка: не найден {SIM_VEHICLE_PATH}")
        print("Убедитесь, что ArduPilot клонирован в ../ardupilot относительно проекта.")
        sys.exit(1)

    args = [
        sys.executable,
        SIM_VEHICLE_PATH,
        "-v", "ArduCopter",
        "-w",
        # Без --model webots-python: используем встроенную физику SITL
        f"--instance={instance}",
        f"--sysid={instance + 1}",
        f"--out=127.0.0.1:{udp_port}",
        "--console",
    ]
    if params:
        args.extend(params)

    print(f"[SITL] Запуск instance={instance}, sysid={instance+1}, UDP MAVLink -> 127.0.0.1:{udp_port}")
    # sim_vehicle.py ожидает запуск из корня ArduPilot
    cwd = APM_HOME if os.path.isdir(APM_HOME) else project_root
    return subprocess.Popen(args, cwd=cwd)


def main():
    parser = argparse.ArgumentParser(
        description="Запуск двух SITL без Webots для теста изоляции (лидер/преследователь)."
    )
    parser.add_argument("--drones", type=int, default=2, help="Количество дронов (по умолчанию 2)")
    parser.add_argument("--param-file", type=str, default=None,
                        help="Файл параметров (например config/iris.parm)")
    args = parser.parse_args()

    # Порты должны совпадать с DRONES_CONFIG в leader_forward_one_move.py
    BASE_UDP_PORT = 14551
    extra_params = []
    if args.param_file:
        p = os.path.join(project_root, args.param_file)
        if os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")

    processes = []
    for i in range(args.drones):
        udp_port = BASE_UDP_PORT + i * 10
        proc = start_sitl_instance(i, udp_port, params=extra_params if extra_params else None)
        processes.append(proc)
        time.sleep(5)

    print("\n--- SITL запущены (без Webots). Подключите сценарий к UDP 14551 (лидер) и 14561 (преследователь). ---")
    print("Например: cd new_iteration_RC_OVERRIDE/experiment_stability && python leader_forward_one_move.py\n")
    try:
        for p in processes:
            p.wait()
    except KeyboardInterrupt:
        print("\nОстановка SITL...")
        for p in processes:
            p.terminate()
        for p in processes:
            p.wait(timeout=5)


if __name__ == "__main__":
    main()
