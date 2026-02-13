#!/usr/bin/env python3
"""
Построение графиков координат по логу эксперимента: лидер и преследователь во времени.
Позволяет увидеть, отставал ли преследователь от лидера по осям X и Y (NED).

Использование:
  python plot_log_coords.py logs/two_drones_log.csv
  python plot_log_coords.py logs/two_drones_log.csv -o plot.png
"""
import sys
import csv
import argparse

def load_log(path):
    """Загрузка лога: возвращает dict с массивами по колонкам."""
    with open(path, newline="") as f:
        reader = csv.DictReader(f)
        rows = list(reader)
    if not rows:
        return None
    fieldnames = list(rows[0].keys())
    data = {k: [] for k in fieldnames}
    for row in rows:
        for k in fieldnames:
            try:
                data[k].append(float(row.get(k, 0)))
            except (ValueError, TypeError):
                data[k].append(0.0)
    return data

def main():
    parser = argparse.ArgumentParser(description="Plot leader/follower coordinates from log CSV")
    parser.add_argument("log_file", nargs="?", default="logs/two_drones_log.csv", help="Path to log CSV")
    parser.add_argument("-o", "--output", type=str, default=None, help="Save figure to file")
    parser.add_argument("--no-show", action="store_true", help="Do not show interactive window (only save)")
    args = parser.parse_args()

    try:
        data = load_log(args.log_file)
    except FileNotFoundError:
        print(f"File not found: {args.log_file}")
        sys.exit(1)
    if not data or not data.get("t"):
        print("Empty or invalid log")
        sys.exit(1)

    try:
        import matplotlib
        if args.no_show or args.output:
            matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("Install matplotlib: pip install matplotlib")
        sys.exit(1)

    t = data["t"]
    leader_x = data.get("leader_x", [0] * len(t))
    leader_y = data.get("leader_y", [0] * len(t))
    follower_x = data.get("follower_x", [0] * len(t))
    follower_y = data.get("follower_y", [0] * len(t))
    error_x = data.get("error_x", [0] * len(t))
    error_y = data.get("error_y", [0] * len(t))

    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    fig.suptitle("Leader vs Follower: coordinates and errors (NED)\nFollower lag → leader curve ahead in time", fontsize=11)

    # 1) X (North) vs time
    ax1 = axes[0, 0]
    ax1.plot(t, leader_x, label="Leader X (north)", color="C0", linewidth=1.5)
    ax1.plot(t, follower_x, label="Follower X (north)", color="C1", linestyle="--", linewidth=1)
    ax1.set_xlabel("Time (s)")
    ax1.set_ylabel("X (m, NED)")
    ax1.set_title("X (North) vs time — lag: follower curve to the right of leader")
    ax1.legend(loc="best", fontsize=8)
    ax1.grid(True, alpha=0.3)

    # 2) Y (East) vs time
    ax2 = axes[0, 1]
    ax2.plot(t, leader_y, label="Leader Y (east)", color="C0", linewidth=1.5)
    ax2.plot(t, follower_y, label="Follower Y (east)", color="C1", linestyle="--", linewidth=1)
    ax2.set_xlabel("Time (s)")
    ax2.set_ylabel("Y (m, NED)")
    ax2.set_title("Y (East) vs time — lag: follower curve to the right of leader")
    ax2.legend(loc="best", fontsize=8)
    ax2.grid(True, alpha=0.3)

    # 3) Error X and Y vs time (positive error = leader ahead)
    ax3 = axes[1, 0]
    ax3.plot(t, error_x, label="Error X (leader−follower)", color="C2", linewidth=1)
    ax3.plot(t, error_y, label="Error Y (leader−follower)", color="C3", linewidth=1)
    ax3.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax3.set_xlabel("Time (s)")
    ax3.set_ylabel("Error (m)")
    ax3.set_title("Position error vs time — positive = leader ahead (follower lagging)")
    ax3.legend(loc="best", fontsize=8)
    ax3.grid(True, alpha=0.3)

    # 4) Trajectory in XY plane
    ax4 = axes[1, 1]
    ax4.plot(leader_x, leader_y, label="Leader path", color="C0", linewidth=1.5)
    ax4.plot(follower_x, follower_y, label="Follower path", color="C1", linestyle="--", linewidth=1)
    ax4.set_xlabel("X (m, North)")
    ax4.set_ylabel("Y (m, East)")
    ax4.set_title("Trajectory (XY) — follower lag = follower curve inside/behind leader")
    ax4.legend(loc="best", fontsize=8)
    ax4.grid(True, alpha=0.3)
    ax4.axis("equal")

    plt.tight_layout()

    if args.output:
        plt.savefig(args.output, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.output}")
    if not args.no_show and not args.output:
        plt.show()

if __name__ == "__main__":
    main()
