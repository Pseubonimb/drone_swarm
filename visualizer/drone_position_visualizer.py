#!/usr/bin/env python3
"""
Визуализатор позиций двух дронов в реальном времени.

Получает координаты по UDP от скриптов симуляции (через position_publisher).
Координаты — NED (x=North, y=East, z=Down), те же что даёт CoordsMonitor.

Запуск:
  python drone_position_visualizer.py
  python drone_position_visualizer.py --port 15551

Порядок запуска не важен:
  • Сначала визуализатор, затем симуляция — график появится, когда придёт первая точка
  • Сначала симуляция, затем визуализатор — точки появятся сразу
"""
import argparse
import json
import socket
import sys
import threading
import warnings

# Suppress matplotlib aspect/adjustable warnings
warnings.filterwarnings("ignore", message=".*fixed.*aspect.*adjustable.*", category=UserWarning)

# Try matplotlib
try:
    import matplotlib
    matplotlib.use("TkAgg")
    try:
        matplotlib.set_loglevel("warning")
    except AttributeError:
        pass
    import matplotlib.pyplot as plt
    import matplotlib.animation as animation
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

DEFAULT_PORT = 15551
BUFFER_SIZE = 4096


def parse_args():
    parser = argparse.ArgumentParser(
        description="Real-time drone position visualizer (UDP, NED coordinates)"
    )
    parser.add_argument("--port", "-p", type=int, default=DEFAULT_PORT,
                        help=f"UDP port to listen on (default {DEFAULT_PORT})")
    parser.add_argument("--trail", type=int, default=30,
                        help="Number of last points to show as trail (0 = current only)")
    parser.add_argument("--interval", type=float, default=0.1,
                        help="Plot update interval in seconds")
    return parser.parse_args()


class PositionReceiver:
    """Слушает UDP и накапливает позиции дронов и частоты контуров (rates)."""
    MAX_HISTORY = 500  # cap to avoid unbounded growth and lag

    def __init__(self, port: int):
        self.port = port
        self.positions = {}
        self.history = {}
        self.rates = {}  # follower_hz, exchange_hz, webots_step_hz (float or None)
        self.lock = threading.Lock()
        self.running = False
        self.sock = None
    
    def start(self):
        self.running = True
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("0.0.0.0", self.port))
        self.sock.settimeout(0.5)
        t = threading.Thread(target=self._recv_loop, daemon=True)
        t.start()
    
    def stop(self):
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
    
    def _recv_loop(self):
        while self.running and self.sock:
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                obj = json.loads(data.decode("utf-8"))
                pos_dict = obj.get("positions", {})
                rates_dict = obj.get("rates")
                with self.lock:
                    for drone_id_str, p in pos_dict.items():
                        drone_id = int(drone_id_str) if drone_id_str.isdigit() else drone_id_str
                        x = float(p.get("x", 0))
                        y = float(p.get("y", 0))
                        z = float(p.get("z", 0))
                        self.positions[drone_id] = {"x": x, "y": y, "z": z}
                        if drone_id not in self.history:
                            self.history[drone_id] = []
                        h = self.history[drone_id]
                        h.append((x, y, z))
                        if len(h) > self.MAX_HISTORY:
                            self.history[drone_id] = h[-self.MAX_HISTORY:]
                    if rates_dict is not None:
                        self.rates = {
                            "follower_hz": rates_dict.get("follower_hz"),
                            "exchange_hz": rates_dict.get("exchange_hz"),
                            "webots_step_hz": rates_dict.get("webots_step_hz"),
                        }
            except socket.timeout:
                continue
            except (json.JSONDecodeError, KeyError, ValueError) as e:
                pass
            except Exception:
                if self.running:
                    raise
    
    def get_positions(self):
        with self.lock:
            return self.positions.copy(), {k: list(v) for k, v in self.history.items()}

    def get_rates(self):
        with self.lock:
            return self.rates.copy()


def run_visualizer(port: int, trail: int, interval: float):
    if not HAS_MATPLOTLIB:
        print("Error: matplotlib required. Install: pip install matplotlib")
        sys.exit(1)
    
    receiver = PositionReceiver(port)
    receiver.start()
    print(f"Listening for drone positions on UDP port {port}")
    print("Start simulation scripts (leader_forward_one_move.py, etc.) to see positions.")
    print("Press Ctrl+C to stop.\n")
    
    fig, (ax, ax_rates) = plt.subplots(2, 1, figsize=(9, 9), gridspec_kw={"height_ratios": [3, 1]})
    ax.set_xlabel("X (м, North, NED)")
    ax.set_ylabel("Y (м, East, NED)")
    ax.set_title("Позиции дронов в реальном времени (X–Y)")
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")
    ax_rates.set_axis_off()
    rates_text = ax_rates.text(0.5, 0.5, "", transform=ax_rates.transAxes, fontsize=10,
                               verticalalignment="center", horizontalalignment="center",
                               family="monospace")
    
    colors = {1: ("#2563eb", "Лидер"), 2: ("#dc2626", "Преследователь")}
    for i in range(3, 10):
        colors[i] = (f"C{i % 10}", f"Дрон {i}")
    
    lines = {}
    points = {}
    for did, (col, label) in colors.items():
        lines[did], = ax.plot([], [], "-", color=col, linewidth=1.5, alpha=0.6, label=label)
        points[did], = ax.plot([], [], "o", color=col, markersize=10)
    
    def _format_hz(v):
        if v is None:
            return "—"
        try:
            return f"{float(v):.1f}"
        except (TypeError, ValueError):
            return "—"
    
    def init():
        return list(lines.values()) + list(points.values())
    
    def animate(_):
        pos, hist = receiver.get_positions()
        rates = receiver.get_rates()
        vals = []
        if rates.get("follower_hz") is not None:
            vals.append(rates["follower_hz"])
        if rates.get("exchange_hz") is not None:
            vals.append(rates["exchange_hz"])
        if rates.get("webots_step_hz") is not None:
            vals.append(rates["webots_step_hz"])
        min_hz = min(vals) if vals else None
        rates_str = (
            f"follower_loop: {_format_hz(rates.get('follower_hz'))} Hz  |  "
            f"exchange_loop: {_format_hz(rates.get('exchange_hz'))} Hz  |  "
            f"Webots step: {_format_hz(rates.get('webots_step_hz'))} Hz  |  "
            f"Минимальная частота контуров: {_format_hz(min_hz)} Hz"
        )
        rates_text.set_text(rates_str)
        
        if not pos:
            return list(lines.values()) + list(points.values())
        
        trail_n = max(0, trail)
        for drone_id, pts in hist.items():
            if drone_id not in lines:
                col = colors.get(drone_id, (f"C{drone_id % 10}", f"Дрон {drone_id}"))[0]
                lines[drone_id], = ax.plot([], [], "-", color=col, linewidth=1.5, alpha=0.6)
                points[drone_id], = ax.plot([], [], "o", color=col, markersize=10)
            
            if trail_n > 0 and len(pts) > trail_n:
                pts = pts[-trail_n:]
            elif trail_n == 0 and pts:
                pts = pts[-1:]
            
            if pts:
                xs = [p[0] for p in pts]
                ys = [p[1] for p in pts]
                lines[drone_id].set_data(xs, ys)
                points[drone_id].set_data([xs[-1]], [ys[-1]])
        
        # Auto-scale: view follows drones, centered on current positions
        all_x, all_y = [], []
        for pts in hist.values():
            if pts:
                trail_pts = pts[-trail_n:] if trail_n > 0 else pts[-1:]
                all_x.extend(p[0] for p in trail_pts)
                all_y.extend(p[1] for p in trail_pts)
        if all_x and all_y:
            margin = 2.0
            x_min, x_max = min(all_x) - margin, max(all_x) + margin
            y_min, y_max = min(all_y) - margin, max(all_y) + margin
            # Keep equal aspect: use the larger range so view always contains all drones
            range_x = x_max - x_min
            range_y = y_max - y_min
            max_range = max(range_x, range_y, 1.0)
            x_center = (x_min + x_max) / 2
            y_center = (y_min + y_max) / 2
            half = max_range / 2
            ax.set_xlim(x_center - half, x_center + half)
            ax.set_ylim(y_center - half, y_center + half)
        
        return list(lines.values()) + list(points.values())
    
    anim = animation.FuncAnimation(
        fig, animate, init_func=init, interval=int(interval * 1000), blit=False
    )
    
    plt.legend(loc="upper left", fontsize=9)
    plt.tight_layout()
    try:
        plt.show()
    finally:
        receiver.stop()


def main():
    args = parse_args()
    try:
        run_visualizer(args.port, args.trail, args.interval)
    except KeyboardInterrupt:
        print("\nВизуализатор остановлен.")


if __name__ == "__main__":
    main()
