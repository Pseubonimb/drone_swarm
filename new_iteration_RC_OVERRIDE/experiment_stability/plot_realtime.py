#!/usr/bin/env python3
"""
Визуализация движения дронов по координатам X и Y в реальном времени.
Читает лог-файл по мере записи (тот же файл пишет leader_forward_one_move.py) и обновляет график.

Режимы:
  --output FILE   сохранять кадр в файл (без окна; не требует Qt/tkinter).
                  Откройте файл в просмотрщике, при необходимости: feh -R 0.2 FILE
  без --output    попытка открыть окно (нужен PyQt5 или python3-tk; на Wayland может не работать).

  python plot_realtime.py logs/two_drones_log.csv --output logs/realtime_xy.png
  python plot_realtime.py logs/two_drones_log.csv
"""
import sys
import csv
import time
import argparse
import os

def read_log_rows(path, max_rows=None):
    """Читает все строки данных из CSV (без заголовка). Возвращает список dict."""
    if not os.path.isfile(path):
        return []
    rows = []
    try:
        with open(path, newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                try:
                    row = {k: float(v) for k, v in row.items()}
                    rows.append(row)
                    if max_rows and len(rows) >= max_rows:
                        break
                except (ValueError, TypeError):
                    continue
    except (IOError, csv.Error):
        pass
    return rows

def main():
    parser = argparse.ArgumentParser(description="Real-time X-Y plot of leader and follower from log CSV")
    parser.add_argument("log_file", nargs="?", default="logs/two_drones_log.csv", help="Path to log CSV")
    parser.add_argument("--trail", type=int, default=200, help="Number of last points to show as trail (0 = only current)")
    parser.add_argument("--interval", type=float, default=0.15, help="Update interval in seconds")
    parser.add_argument("--output", "-o", type=str, default=None, help="Save to image file (no window; use if Qt/tk fails)")
    args = parser.parse_args()

    import matplotlib
    use_window = not args.output
    if args.output:
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        outpath = args.output
        os.makedirs(os.path.dirname(outpath) or ".", exist_ok=True)
        print(f"Режим: сохранение в {outpath} каждые {args.interval} с. Откройте в просмотрщике (автообновление: feh -R 0.2 {outpath})")
    else:
        backend_ok = False
        for backend in ("Qt5Agg", "TkAgg", "QtAgg", "GTK4Agg", "GTK3Agg", "WXAgg"):
            try:
                matplotlib.use(backend)
                backend_ok = True
                break
            except Exception:
                continue
        if not backend_ok:
            print("Не найден GUI-бэкенд. Используйте режим без окна:")
            print("  python plot_realtime.py logs/two_drones_log.csv --output logs/realtime_xy.png")
            print("  затем откройте файл или: feh -R 0.2 logs/realtime_xy.png")
            sys.exit(1)
        try:
            import matplotlib.pyplot as plt
        except ImportError:
            print("Установите matplotlib: pip install matplotlib")
            sys.exit(1)

    print(f"Ожидание данных в {args.log_file} (запустите сценарий в другом терминале)...")
    while not os.path.isfile(args.log_file):
        time.sleep(0.5)
    while True:
        rows = read_log_rows(args.log_file)
        if rows:
            break
        time.sleep(0.5)

    fig, ax = plt.subplots(figsize=(9, 7))
    ax.set_xlabel("X (м, North, NED)")
    ax.set_ylabel("Y (м, East, NED)")
    ax.set_title("Движение дронов в реальном времени (X–Y)")
    ax.grid(True, alpha=0.3)
    ax.axis("equal")

    leader_line, = ax.plot([], [], "b-", linewidth=1.5, alpha=0.6, label="Лидер (след)")
    follower_line, = ax.plot([], [], "r-", linewidth=1.5, alpha=0.6, label="Преследователь (след)")
    leader_pt, = ax.plot([], [], "bo", markersize=10, label="Лидер")
    follower_pt, = ax.plot([], [], "rs", markersize=10, label="Преследователь")
    ax.legend(loc="upper left", fontsize=9)

    trail = max(0, args.trail)
    try:
        while True:
            rows = read_log_rows(args.log_file)
            if not rows:
                plt.pause(args.interval)
                continue

            t = [r["t"] for r in rows]
            lx = [r["leader_x"] for r in rows]
            ly = [r["leader_y"] for r in rows]
            fx = [r["follower_x"] for r in rows]
            fy = [r["follower_y"] for r in rows]

            if trail > 0 and len(rows) > trail:
                lx, ly = lx[-trail:], ly[-trail:]
                fx, fy = fx[-trail:], fy[-trail:]
            elif trail == 0:
                lx, ly = [lx[-1]], [ly[-1]]
                fx, fy = [fx[-1]], [fy[-1]]

            leader_line.set_data(lx, ly)
            follower_line.set_data(fx, fy)
            leader_pt.set_data([lx[-1]], [ly[-1]])
            follower_pt.set_data([fx[-1]], [fy[-1]])

            if len(rows) >= 2:
                margin = 2.0
                x_min = min(min(lx), min(fx)) - margin
                x_max = max(max(lx), max(fx)) + margin
                y_min = min(min(ly), min(fy)) - margin
                y_max = max(max(ly), max(fy)) + margin
                ax.set_xlim(x_min, x_max)
                ax.set_ylim(y_min, y_max)

            if use_window:
                fig.canvas.draw_idle()
                plt.pause(args.interval)
            else:
                fig.savefig(outpath, dpi=120, bbox_inches="tight")
                time.sleep(args.interval)

    except KeyboardInterrupt:
        print("\nВизуализация остановлена.")
    plt.close()

if __name__ == "__main__":
    main()
