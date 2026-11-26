#!/usr/bin/env python3
"""Record labeled IMU episodes from the Pico and plot them.

Flash the firmware with OUTPUT_CSV 1, close any serial monitor, then:

  python3 tools/imu_logger.py record              # record episodes (live plot)
  python3 tools/imu_logger.py plot                # plot the most recent recording
  python3 tools/imu_logger.py plot recordings/*palm-up*.csv --only pose finger palm still

While recording (click the plot window first):
  z       zero the heading: the way the fingers point now becomes "away"
  SPACE   drop a marker (vertical line, saved in the 'marker' column)
  q       stop and save this episode (closing the window does the same)

Episodes are saved to recordings/<date>-<time>_<label>.csv. After saving (and
when plotting) it prints which poses the hand was in while it was still - an
easy way to check that a sign lands in the zone you expect.
"""
import argparse
import csv
import glob
import re
import sys
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import matplotlib.pyplot as plt
import serial

ROOT = Path(__file__).resolve().parent.parent
REC_DIR = ROOT / "recordings"
AXIS_SUFFIXES = {"x", "y", "z", "r", "i", "j", "k", "palm", "fingers"}

# Must match the firmware (imu.h / imu.c).
POSE_NAMES = ["between", "up", "down", "away", "toward", "left", "right"]   # imu_dir_t order
POSE_MIN_COS = 0.8                                                          # imu.c
ROOM_AXES = {"x": "away", "y": "left", "z": "up"}    # finger_* / palm_* columns
ROOM_GROUPS = {"finger", "palm"}
CATEGORY_TICKS = {"pose": POSE_NAMES, "still": ["moving", "still"]}


def find_port():
    ports = sorted(glob.glob("/dev/tty.usbmodem*")) + sorted(glob.glob("/dev/ttyACM*"))
    if not ports:
        sys.exit("No Pico found (looked for /dev/tty.usbmodem*). "
                 "Is it plugged in, and is the serial monitor closed?")
    return ports[0]


def group_columns(cols):
    """accel_x, accel_y, accel_z -> {'accel': [...]}; anything else gets its own plot."""
    groups = {}
    for c in cols:
        prefix, _, suffix = c.rpartition("_")
        key = prefix if prefix and suffix in AXIS_SUFFIXES else c
        groups.setdefault(key, []).append(c)
    return groups


def make_figure(cols, title):
    groups = group_columns(cols)
    fig, axes = plt.subplots(len(groups), 1, sharex=True, squeeze=False,
                             figsize=(11, min(1.8 * len(groups) + 1, 12)))
    axes = axes[:, 0]
    lines = {}
    for ax, (name, members) in zip(axes, groups.items()):
        ticks = CATEGORY_TICKS.get(name)
        for c in members:
            label = c.rpartition("_")[2] if len(members) > 1 else c
            if name in ROOM_GROUPS:
                label = ROOM_AXES.get(label, label)
            (lines[c],) = ax.plot([], [], label=label,
                                  drawstyle="steps-post" if ticks else "default")
        if ticks:                                  # pose / still: show names, not numbers
            ax.set_yticks(range(len(ticks)), ticks)
            ax.set_ylim(-0.5, len(ticks) - 0.5)
            ax.set_autoscaley_on(False)
        elif name in ROOM_GROUPS:                  # unit directions; dotted = zone edges
            ax.set_ylim(-1.1, 1.1)
            ax.set_autoscaley_on(False)
            for y in (POSE_MIN_COS, -POSE_MIN_COS):
                ax.axhline(y, color="gray", ls=":", lw=1)
        ax.set_ylabel(name)
        ax.grid(alpha=0.3)
        ax.legend(loc="upper left", fontsize=8)
    axes[-1].set_xlabel("time (s)")
    fig.suptitle(title)
    fig.tight_layout()
    return fig, axes, lines


class Stream:
    """Reads H,/D, lines from the Pico. Other lines are printed as log messages."""

    def __init__(self, port):
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.cols = None

    bytes_seen = 0
    data_lines_seen = 0

    def lines(self):
        while self.ser.in_waiting:
            raw = self.ser.readline().decode(errors="replace").strip()
            self.bytes_seen += len(raw)
            if raw.startswith("D,"):
                self.data_lines_seen += 1
            if raw.startswith("H,"):
                cols = raw.split(",")[2:]          # drop 'H' and 't_ms'
                if self.cols is not None and cols != self.cols:
                    print("Column set changed (reflashed?). Starting a fresh episode is safest.")
                self.cols = cols
            elif raw.startswith("D,"):
                parts = raw.split(",")[1:]
                if self.cols is None or len(parts) != len(self.cols) + 1:
                    continue                       # no header yet, or partial line
                try:
                    yield [float(p) for p in parts]
                except ValueError:
                    continue
            elif raw:
                print(f"[pico] {raw}")

    def wait_for_header(self, timeout=20.0):
        """The Pico resends the header every second, but right after a flash or
        replug it can take several seconds to boot and find the IMU."""
        if self.cols is not None:
            return
        print("Waiting for the Pico to send its column header...", end="", flush=True)
        start = time.time()
        next_dot = start + 1
        while self.cols is None and time.time() - start < timeout:
            for _ in self.lines():
                pass
            if time.time() >= next_dot:
                print(".", end="", flush=True)
                next_dot += 1
            time.sleep(0.05)
        print()
        if self.cols is not None:
            return
        if self.bytes_seen == 0:
            sys.exit("Nothing received from the Pico at all. Is it running (not in BOOTSEL), "
                     "and is every other serial monitor closed?")
        if self.data_lines_seen:
            sys.exit("Getting data rows but no header. Reflash the latest firmware.")
        sys.exit("The Pico is talking but not sending CSV. Is OUTPUT_CSV set to 1 in the firmware?")


def record_episode(stream, label):
    stream.ser.reset_input_buffer()
    stream.ser.readline()          # throw away the half line left after clearing the buffer
    stream.wait_for_header()
    cols = stream.cols
    t, data, markers = [], {c: [] for c in cols}, []
    t0 = None
    state = {"stop": False}

    fig, axes, lines = make_figure(cols, f"{label}  (z = zero heading, SPACE = marker, q = stop & save)")

    def on_key(event):
        if event.key == " " and t:
            markers.append(len(t) - 1)
            for ax in axes:
                ax.axvline(t[-1], color="k", ls="--", lw=1)
        elif event.key == "z":
            stream.ser.write(b"z")     # firmware: the way the fingers point now = "away"
            print("Heading zeroed (fingers now point 'away').")
        elif event.key == "q":
            state["stop"] = True

    fig.canvas.mpl_connect("key_press_event", on_key)
    fig.canvas.mpl_connect("close_event", lambda e: state.update(stop=True))
    plt.show(block=False)

    try:
        while not state["stop"]:
            for row in stream.lines():
                if t0 is None:
                    t0 = row[0]
                t.append((row[0] - t0) / 1000.0)
                for c, v in zip(cols, row[1:]):
                    data[c].append(v)
            for c in cols:
                lines[c].set_data(t, data[c])
            for ax in axes:
                ax.relim()
                ax.autoscale_view()
            plt.pause(0.1)
    except KeyboardInterrupt:
        pass
    plt.close(fig)

    if not t:
        print("No data recorded, nothing saved.")
        return
    REC_DIR.mkdir(exist_ok=True)
    safe = re.sub(r"[^A-Za-z0-9_-]+", "-", label).strip("-") or "episode"
    path = REC_DIR / f"{datetime.now():%Y%m%d-%H%M%S}_{safe}.csv"
    marked = set(markers)
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["label", "t_s", *cols, "marker"])
        for i, ti in enumerate(t):
            w.writerow([label, f"{ti:.3f}", *(data[c][i] for c in cols), int(i in marked)])
    print(f"Saved {len(t)} samples ({t[-1]:.1f} s, {len(markers)} markers) -> {path.relative_to(ROOT)}")
    print_pose_summary(data)


def print_pose_summary(data):
    """Share of the STILL samples spent in each palm/finger pose."""
    if not {"pose_palm", "pose_fingers", "still"} <= data.keys():
        return
    poses = Counter((int(p), int(f))
                    for p, f, s in zip(data["pose_palm"], data["pose_fingers"], data["still"])
                    if s >= 0.5)
    total = sum(poses.values())
    if not total:
        print("Pose summary: the hand was never still.")
        return
    print(f"Pose while still ({total} samples):")
    for (p, f), n in poses.most_common():
        print(f"  {100 * n / total:5.1f}%  palm {POSE_NAMES[p]}, fingers {POSE_NAMES[f]}")


def cmd_record(args):
    stream = Stream(args.port or find_port())
    print(f"Connected to {stream.ser.port}")
    while True:
        try:
            label = input("\nEpisode label (Enter to quit): ").strip()
        except (EOFError, KeyboardInterrupt):
            break
        if not label:
            break
        record_episode(stream, label)


def cmd_plot(args):
    files = args.files or sorted(glob.glob(str(REC_DIR / "*.csv")))[-1:]
    if not files:
        sys.exit("No recordings found.")
    for path in files:
        with open(path) as f:
            rows = list(csv.DictReader(f))
        if not rows:
            continue
        all_cols = [c for c in rows[0] if c not in ("label", "t_s", "marker")]
        data = {c: [float(r[c]) for r in rows] for c in all_cols}
        cols = all_cols
        if args.only:
            cols = [c for c in cols if any(o in c for o in args.only)]
        t = [float(r["t_s"]) for r in rows]
        title = rows[0].get("label") or Path(path).stem
        print(f"\n{Path(path).name}")
        print_pose_summary(data)
        fig, axes, lines = make_figure(cols, f"{title}   ({Path(path).name})")
        for c in cols:
            lines[c].set_data(t, data[c])
        for ax in axes:
            for r, ti in zip(rows, t):
                if r.get("marker") == "1":
                    ax.axvline(ti, color="k", ls="--", lw=1)
            ax.relim()
            ax.autoscale_view()
    plt.show()


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest="cmd", required=True)
    r = sub.add_parser("record", help="record labeled episodes with a live plot")
    r.add_argument("--port", help="serial port (default: auto-detect)")
    r.set_defaults(func=cmd_record)
    pl = sub.add_parser("plot", help="plot saved recordings (default: the latest)")
    pl.add_argument("files", nargs="*")
    pl.add_argument("--only", nargs="+", help="only columns containing these words, e.g. --only pose finger palm")
    pl.set_defaults(func=cmd_plot)
    args = p.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
