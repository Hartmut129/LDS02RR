"""Live 2D view of LDS02RR scans from the Arduino sketch (lds02rr_serial.ino).

Reads lines "S,<rpm>,d0,...,d359" (mm, 0 = invalid) from the serial port and
plots the points in X/Y, sensor at the origin, 0 deg pointing up.
Lines starting with '#' are status lines and shown in the window title.

    python lidar_view.py --port COM18
    python lidar_view.py --port COM18 --save scan.png   # one scan to PNG, no window
    ./lds02rr_linux /dev/ttyUSB0 | python3 lidar_view.py --port -   # from stdin

Requires: pip install pyserial matplotlib numpy
"""
import argparse
import sys
import threading
import time

import numpy as np
import serial

ANGLES = np.deg2rad(np.arange(360))


class ScanReader(threading.Thread):
    """Reads the serial port in the background, keeps the latest scan."""

    def __init__(self, port, baud):
        super().__init__(daemon=True)
        if port == "-":
            self.ser = sys.stdin.buffer      # e.g. piped from lds02rr_linux
        else:
            self.ser = serial.Serial(port, baud, timeout=1)
        self.lock = threading.Lock()
        self.scan = None        # (rpm, np.array of 360 distances in mm)
        self.scan_count = 0
        self.status = ""

    def run(self):
        while True:
            raw = self.ser.readline()
            if self.ser is sys.stdin.buffer and not raw:
                self.status = "end of input"
                return
            line = raw.decode("ascii", errors="ignore").strip()
            if line.startswith("#"):
                self.status = line[1:].strip()
            elif line.startswith("S,"):
                parts = line.split(",")
                if len(parts) != 362:
                    continue    # incomplete line
                try:
                    rpm = float(parts[1])
                    dist = np.array(parts[2:], dtype=float)
                except ValueError:
                    continue
                with self.lock:
                    self.scan = (rpm, dist)
                    self.scan_count += 1

    def latest(self):
        with self.lock:
            return self.scan, self.scan_count


def to_xy(dist_mm, ccw, offset_deg):
    """Polar -> cartesian in metres. 0 deg = +Y (up), clockwise unless ccw."""
    valid = dist_mm > 0
    a = ANGLES[valid] + np.deg2rad(offset_deg)
    if ccw:
        a = -a
    d = dist_mm[valid] / 1000.0
    return d * np.sin(a), d * np.cos(a), d


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default="COM18", help="serial port, or - for stdin")
    ap.add_argument("--baud", type=int, default=500000)
    ap.add_argument("--range", type=float, default=4.0, help="plot range in m")
    ap.add_argument("--offset", type=float, default=0.0,
                    help="rotate the plot by this many degrees (mounting offset)")
    ap.add_argument("--ccw", action="store_true",
                    help="angles increase counter-clockwise (mirror the plot)")
    ap.add_argument("--save", metavar="PNG",
                    help="wait for one complete scan, save it, and exit")
    args = ap.parse_args()

    if args.save:
        import matplotlib
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.animation import FuncAnimation

    reader = ScanReader(args.port, args.baud)
    reader.start()

    fig, ax = plt.subplots(figsize=(7, 7))
    ax.set_aspect("equal")
    ax.set_xlim(-args.range, args.range)
    ax.set_ylim(-args.range, args.range)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.grid(True, alpha=0.3)
    for r in np.arange(1, args.range + 0.01):
        ax.add_patch(plt.Circle((0, 0), r, fill=False, ls=":", lw=0.6, alpha=0.4))
    ax.plot(0, 0, "r^", ms=10)                       # sensor, 0 deg = up
    points = ax.scatter([], [], s=6, c=[], cmap="viridis",
                        vmin=0, vmax=args.range)
    fig.colorbar(points, ax=ax, shrink=0.75, label="distance [m]")
    title = ax.set_title("waiting for data...")

    def update(_frame=None):
        scan, count = reader.latest()
        if scan is None:
            title.set_text("waiting for data...  " + reader.status)
            return points, title
        rpm, dist = scan
        x, y, d = to_xy(dist, args.ccw, args.offset)
        points.set_offsets(np.column_stack((x, y)))
        points.set_array(d)
        title.set_text(f"scan {count}   {rpm:.0f} rpm   {len(d)}/360 valid")
        return points, title

    if args.save:
        # Opening the port resets the Mega; skip ~3 s of motor spin-up
        deadline = time.time() + 20
        while time.time() < deadline and reader.latest()[1] < 15:
            time.sleep(0.05)
        if reader.latest()[0] is None:
            raise SystemExit(f"no scan received on {args.port}")
        update()
        fig.savefig(args.save, dpi=110, bbox_inches="tight")
        print(f"saved {args.save}: {title.get_text()}")
        return

    anim = FuncAnimation(fig, update, interval=100, cache_frame_data=False)  # noqa: F841
    plt.show()


if __name__ == "__main__":
    main()
