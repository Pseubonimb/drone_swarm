#!/usr/bin/env python3
"""
Универсальный лаунчер симуляции: SITL + Webots (опционально) + визуализатор + сценарий.

Использование:
  python launch_simulation.py                    # интерактивное меню
  python launch_simulation.py --help             # справка по аргументам
  python launch_simulation.py -s -v -c leader   # SITL-only, визуализатор, сценарий leader

Режимы симуляции:
  --webots, -w      Webots 3D + SITL (полная симуляция)
  --sitl-only, -s   Только SITL, без Webots (лёгкий режим)

Визуализация:
  --visualizer, -v  Запуск 2D-визуализатора позиций дронов (UDP)
"""
import os
import subprocess
import sys
import time
import argparse
import signal

project_root = os.path.dirname(os.path.abspath(__file__))
APM_HOME = os.path.join(project_root, "..", "ardupilot")
SIM_VEHICLE_PATH = os.path.join(APM_HOME, "Tools", "autotest", "sim_vehicle.py")

# Сценарии: (id, описание, путь к скрипту, рабочая директория)
SCENARIOS = [
    ("leader", "Лидер-преследователь (leader_forward_one_move)", 
     "leader_forward_one_move.py",
     os.path.join(project_root, "new_iteration_RC_OVERRIDE", "experiment_stability")),
    ("drone_following", "Преследование (drone_following)",
     "drone_following.py",
     os.path.join(project_root, "new_iteration_RC_OVERRIDE")),
    ("drone_following_stay", "Преследование, лидер на месте (drone_following_leader_stay)",
     "drone_following_leader_stay.py",
     os.path.join(project_root, "RC_OVERRIDE", "RC_Following")),
    ("square_free", "Квадрат FREE (two_drones_RC_OVERRIDE_FREE_square)",
     "two_drones_RC_OVERRIDE_FREE_square.py",
     os.path.join(project_root, "RC_OVERRIDE")),
    ("square_pid", "Квадрат PID (two_drones_RC_OVERRIDE_PID_square)",
     "two_drones_RC_OVERRIDE_PID_square.py",
     os.path.join(project_root, "RC_OVERRIDE")),
]

VISUALIZER_SCRIPT = os.path.join(project_root, "visualizer", "drone_position_visualizer.py")


def start_sitl_only(project_root, num_drones=2, param_file=None):
    """Запуск SITL без Webots (встроенная физика)."""
    if not os.path.isfile(SIM_VEHICLE_PATH):
        print(f"Ошибка: не найден {SIM_VEHICLE_PATH}")
        print("ArduPilot должен быть в ../ardupilot относительно проекта.")
        return []
    extra_params = []
    if param_file:
        p = os.path.join(project_root, param_file)
        if os.path.isfile(p):
            extra_params.append(f"--add-param-file={os.path.abspath(p)}")
    processes = []
    BASE_UDP_PORT = 14551
    cwd = APM_HOME if os.path.isdir(APM_HOME) else project_root
    for i in range(num_drones):
        udp_port = BASE_UDP_PORT + i * 10
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{udp_port}",
            "--console",
        ]
        if extra_params:
            args.extend(extra_params)
        print(f"[SITL] instance={i}, UDP -> 127.0.0.1:{udp_port}")
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        time.sleep(5)
    return processes


def start_sitl_webots(project_root, num_drones=2, param_file=None):
    """Запуск SITL с Webots (модель webots-python)."""
    if not os.path.isfile(SIM_VEHICLE_PATH):
        print(f"Ошибка: не найден {SIM_VEHICLE_PATH}")
        return [], None
    BASE_TCP = 5770
    BASE_UDP = 14551
    p = os.path.join(project_root, param_file) if param_file else None
    extra = [f"--add-param-file={os.path.abspath(p)}"] if p and os.path.isfile(p) else []
    processes = []
    cwd = APM_HOME if os.path.isdir(APM_HOME) else project_root
    for i in range(num_drones):
        tcp_port = BASE_TCP + i * 10
        udp_port = BASE_UDP + i * 10
        args = [
            sys.executable,
            SIM_VEHICLE_PATH,
            "-v", "ArduCopter", "-w",
            "--model", "webots-python",
            f"--instance={i}",
            f"--sysid={i + 1}",
            f"--out=127.0.0.1:{tcp_port}",
            f"--out=127.0.0.1:{udp_port}",
            "--console",
        ]
        if extra:
            args.extend(extra)
        print(f"[SITL] instance={i}, TCP={tcp_port}, UDP={udp_port}")
        proc = subprocess.Popen(args, cwd=cwd)
        processes.append(proc)
        time.sleep(5)
    return processes, None


def launch_webots(project_root, num_drones=2):
    """Запуск Webots."""
    WORLDS_DIR = os.path.join(project_root, "worlds")
    INPUT_WORLD = os.path.join(WORLDS_DIR, "irisAuto.wbt")
    OUTPUT_WORLD = os.path.join(WORLDS_DIR, "temp_world.wbt")
    if not os.path.isfile(INPUT_WORLD):
        print(f"Ошибка: мир не найден {INPUT_WORLD}")
        return None
    # Генерация мира (упрощённо — копируем логику run_sim)
    with open(INPUT_WORLD, "r") as f:
        content = f.read()
    insert_marker = "# Insert drones"
    pos = content.find(insert_marker)
    if pos == -1:
        print("Метка # Insert drones не найдена в мире")
        return None
    pos += len(insert_marker)
    dx, dy, z = 2.0, 2.0, 0.0549632125
    drones = []
    for i in range(num_drones):
        row, col = i % 10, i // 10
        x, y = col * dx, row * dy
        drones.append(f'\nIris {{\n  translation {x} {y} {z}\n  rotation 0 1 0 0\n  name "Iris_{i}"\n  controller "ardupilot_vehicle_controller"\n  controllerArgs [ "--instance" "{i}" "--motors" "m1_motor, m2_motor, m3_motor, m4_motor" ]\n}}\n')
    new_content = content[:pos] + "".join(drones) + content[pos:]
    with open(OUTPUT_WORLD, "w") as f:
        f.write(new_content)
    env = os.environ.copy()
    env["WEBOTS_PROTO_PATH"] = os.path.join(project_root, "protos")
    print("[Webots] Запуск...")
    return subprocess.Popen(["webots", OUTPUT_WORLD], env=env)


def run_interactive_menu():
    """Интерактивное меню выбора."""
    print("\n=== Лаунчер симуляции ===\n")
    print("1. Режим симуляции:")
    print("   1) Webots 3D + SITL")
    print("   2) Только SITL (без Webots)")
    mode = input("   Выбор [1/2, по умолчанию 2]: ").strip() or "2"
    use_webots = mode == "1"

    print("\n2. Визуализатор 2D (позиции дронов по UDP):")
    v = input("   Запустить? [y/n, по умолчанию y]: ").strip().lower() or "y"
    use_visualizer = v in ("y", "yes", "д", "да")

    print("\n3. Сценарий:")
    for i, (sid, desc, _, _) in enumerate(SCENARIOS, 1):
        print(f"   {i}) {sid}: {desc}")
    idx = input("   Номер [1-{}, по умолчанию 1]: ".format(len(SCENARIOS))).strip()
    idx = int(idx) if idx.isdigit() else 1
    scenario = SCENARIOS[min(idx - 1, 0)] if 1 <= idx <= len(SCENARIOS) else SCENARIOS[0]

    return use_webots, use_visualizer, scenario


def main():
    parser = argparse.ArgumentParser(
        description="Лаунчер симуляции: SITL + Webots/только SITL + визуализатор + сценарий",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры:
  python launch_simulation.py -s -v -c leader     # SITL-only + визуализатор + лидер-преследователь
  python launch_simulation.py -w -c leader        # Webots + лидер-преследователь
  python launch_simulation.py -s -c square_free   # SITL-only + квадрат FREE
        """,
    )
    g = parser.add_mutually_exclusive_group(required=False)
    g.add_argument("-w", "--webots", action="store_true", help="Webots 3D + SITL")
    g.add_argument("-s", "--sitl-only", action="store_true", help="Только SITL (без Webots)")
    parser.add_argument("-v", "--visualizer", action="store_true", help="Запустить 2D-визуализатор позиций")
    parser.add_argument("-c", "--scenario", type=str, help="ID сценария: " + ", ".join(s[0] for s in SCENARIOS))
    parser.add_argument("--drones", type=int, default=2, help="Количество дронов")
    parser.add_argument("--param-file", type=str, default="config/iris.parm", help="Файл параметров")
    args = parser.parse_args()

    # Определяем режим
    if args.webots or args.sitl_only:
        use_webots = args.webots
        use_visualizer = args.visualizer
        if args.scenario:
            scenario = next((s for s in SCENARIOS if s[0] == args.scenario), SCENARIOS[0])
        else:
            scenario = SCENARIOS[0]
    else:
        use_webots, use_visualizer, scenario = run_interactive_menu()

    scenario_id, scenario_desc, script_name, scenario_cwd = scenario
    script_path = os.path.join(scenario_cwd, script_name)
    if not os.path.isfile(script_path):
        print(f"Ошибка: сценарий не найден: {script_path}")
        sys.exit(1)

    processes = []

    # 1. Webots (если нужен)
    webots_proc = None
    if use_webots:
        webots_proc = launch_webots(project_root, args.drones)
        if webots_proc:
            processes.append(webots_proc)
            time.sleep(5)

    # 2. SITL
    if use_webots:
        sitl_procs, _ = start_sitl_webots(project_root, args.drones, args.param_file)
        processes.extend(sitl_procs)
    else:
        sitl_procs = start_sitl_only(project_root, args.drones, args.param_file)
        processes.extend(sitl_procs)

    if not processes:
        sys.exit(1)

    print("[Launcher] Ожидание готовности SITL (arm/takeoff)...")
    time.sleep(8)

    # 3. Визуализатор
    viz_proc = None
    if use_visualizer and os.path.isfile(VISUALIZER_SCRIPT):
        print("[Visualizer] Запуск 2D-визуализатора позиций...")
        viz_proc = subprocess.Popen(
            [sys.executable, VISUALIZER_SCRIPT],
            cwd=project_root,
        )
        processes.append(viz_proc)
        time.sleep(1)

    # 4. Сценарий
    print(f"[Scenario] Запуск: {scenario_desc}")
    scenario_proc = subprocess.Popen(
        [sys.executable, script_name],
        cwd=scenario_cwd,
        stdout=sys.stdout,
        stderr=sys.stderr,
    )
    processes.append(scenario_proc)

    def shutdown(signum=None, frame=None):
        print("\n[Launcher] Остановка процессов...")
        for p in processes:
            try:
                p.terminate()
            except Exception:
                pass
        time.sleep(1)
        for p in processes:
            try:
                p.wait(timeout=2)
            except subprocess.TimeoutExpired:
                p.kill()
        sys.exit(0)

    signal.signal(signal.SIGINT, shutdown)
    signal.signal(signal.SIGTERM, shutdown)

    try:
        scenario_proc.wait()
    except KeyboardInterrupt:
        pass
    finally:
        shutdown()


if __name__ == "__main__":
    main()
