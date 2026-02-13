#!/usr/bin/env python3
"""
Запуск одного из типовых экспериментов по docs/STRATEGY.md.
Не запускает SITL/Webots — только сценарий с нужными параметрами.
Использование:
  python run_experiment.py H1_kd0
  python run_experiment.py H1_kd10
  python run_experiment.py H2_no_limit
  python run_experiment.py H2_50hz
  python run_experiment.py H3_sitl --run-id my_sitl_run
"""
import subprocess
import sys
import os

# control_hz: 50 = rate limit 50 Hz; 0 = no rate limit (script default is 50)
EXPERIMENTS = {
    "H1_kd0":    {"kd": 0,   "control_hz": 50, "run_id": "H1_kd0"},
    "H1_kd10":   {"kd": 10,  "control_hz": 50, "run_id": "H1_kd10"},
    "H2_no_limit": {"kd": 10, "control_hz": 0, "run_id": "H2_no_limit"},
    "H2_50hz":   {"kd": 10,  "control_hz": 50, "run_id": "H2_50hz"},
    "H3_sitl":   {"kd": 10,  "control_hz": 50, "run_id": "sitl_only"},
    "H3_webots": {"kd": 10,  "control_hz": 50, "run_id": "webots"},
}

def main():
    if len(sys.argv) < 2 or sys.argv[1] not in EXPERIMENTS:
        print("Usage: python run_experiment.py <experiment_id> [--run-id SUFFIX]")
        print("Available:", ", ".join(EXPERIMENTS))
        sys.exit(1)
    exp_id = sys.argv[1]
    overrides = {}
    i = 2
    while i < len(sys.argv):
        if sys.argv[i] == "--run-id" and i + 1 < len(sys.argv):
            overrides["run_id"] = sys.argv[i + 1]
            i += 2
            continue
        i += 1

    cfg = {**EXPERIMENTS[exp_id], **overrides}
    cmd = [sys.executable, "leader_forward_one_move.py",
           "--kp", "8.0", "--ki", "0.0", "--kd", str(cfg["kd"]),
           "--run-id", cfg["run_id"]]
    if cfg.get("control_hz") is not None:
        cmd.extend(["--control-hz", str(cfg["control_hz"])])
    print("Running:", " ".join(cmd))
    os.chdir(os.path.dirname(os.path.abspath(__file__)))
    subprocess.run(cmd)

if __name__ == "__main__":
    main()
