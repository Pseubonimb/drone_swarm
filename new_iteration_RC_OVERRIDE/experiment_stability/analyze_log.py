#!/usr/bin/env python3
"""
Анализ лога эксперимента для проверки гипотез (см. docs/STRATEGY.md, docs/HYPOTHESES_AND_VERIFICATION.md).
  python analyze_log.py logs/two_drones_log_sitl_only.csv
  python analyze_log.py --brief logs/two_drones_log_kd10.csv   # одна строка метрик для таблицы
"""
import sys
import csv
import math

def analyze(path):
    with open(path, newline="") as f:
        r = csv.DictReader(f)
        rows = list(r)
    if not rows:
        return None
    has_loop_dt = "loop_dt" in rows[0]
    has_vel = "follower_vx" in rows[0]
    has_err = "error_x" in rows[0]
    t = [float(row["t"]) for row in rows]
    duration = t[-1] - t[0] if len(t) > 1 else 0
    out = {"path": path, "rows": len(rows), "duration_s": duration}
    if has_err:
        err_x = [float(row["error_x"]) for row in rows]
        err_y = [float(row["error_y"]) for row in rows]
        out["max_err_x"] = max(abs(e) for e in err_x)
        out["max_err_y"] = max(abs(e) for e in err_y)
        out["overshoot_count"] = sum(1 for i in range(1, len(err_x)) if (err_x[i-1] * err_x[i]) < 0 and abs(err_x[i]) < 2.0)
    else:
        out["max_err_x"] = out["max_err_y"] = out["overshoot_count"] = None
    if has_loop_dt:
        loop_dt = [float(row["loop_dt"]) for row in rows if row.get("loop_dt") and float(row["loop_dt"]) > 0]
        if loop_dt:
            mean_dt = sum(loop_dt) / len(loop_dt)
            out["mean_loop_dt"] = mean_dt
            out["effective_hz"] = 1.0 / mean_dt if mean_dt > 0 else 0
            out["max_loop_dt"] = max(loop_dt)
        else:
            out["mean_loop_dt"] = out["effective_hz"] = out["max_loop_dt"] = None
    else:
        out["mean_loop_dt"] = out["effective_hz"] = out["max_loop_dt"] = None
    if has_vel:
        vx = [float(row.get("follower_vx", 0)) for row in rows]
        vy = [float(row.get("follower_vy", 0)) for row in rows]
        out["max_speed"] = max(math.hypot(a, b) for a, b in zip(vx, vy))
    else:
        out["max_speed"] = None
    return out

def main():
    brief = "--brief" in sys.argv
    args = [a for a in sys.argv[1:] if a != "--brief"]
    if len(args) < 1:
        print("Usage: python analyze_log.py [--brief] <path_to_csv>")
        print("  --brief  one-line metrics for docs/HYPOTHESES_AND_VERIFICATION.md")
        sys.exit(1)
    path = args[0]
    try:
        out = analyze(path)
    except FileNotFoundError:
        print(f"File not found: {path}")
        sys.exit(1)
    if out is None:
        print("Empty log")
        sys.exit(0)

    if brief:
        # One line: run_id can be from path; metrics for table
        max_ex = f"{out['max_err_x']:.3f}" if out.get('max_err_x') is not None else "—"
        max_ey = f"{out['max_err_y']:.3f}" if out.get('max_err_y') is not None else "—"
        osc = out.get('overshoot_count') if out.get('overshoot_count') is not None else "—"
        mdt = f"{out['mean_loop_dt']:.4f}" if out.get('mean_loop_dt') is not None else "—"
        hz = f"{out['effective_hz']:.0f}" if out.get('effective_hz') is not None else "—"
        print(f"max_err_x(m)={max_ex} max_err_y(m)={max_ey} overshoot_count={osc} mean_loop_dt(s)={mdt} effective_hz={hz}")
        return

    print(f"=== Log: {path} ===")
    print(f"Rows: {out['rows']}, duration: {out['duration_s']:.1f} s")
    if out.get("max_err_x") is not None:
        print(f"max |error_x|: {out['max_err_x']:.3f} m")
        print(f"max |error_y|: {out['max_err_y']:.3f} m")
        print(f"error_x sign changes (|err|<2m): {out['overshoot_count']} (rough overshoot count)")
    if out.get("mean_loop_dt") is not None:
        print(f"mean loop_dt: {out['mean_loop_dt']:.4f} s -> effective ~{out['effective_hz']:.0f} Hz")
        print(f"max loop_dt: {out['max_loop_dt']:.4f} s")
    if out.get("max_speed") is not None:
        print(f"follower max horizontal speed: {out['max_speed']:.3f} m/s")
    print()

if __name__ == "__main__":
    main()
