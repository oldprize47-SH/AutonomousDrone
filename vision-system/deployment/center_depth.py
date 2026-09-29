#!/usr/bin/env python3
"""
RealSense center-depth probe for temporary altitude/landing distance checks on Uno Q.

This intentionally bypasses helipad detection/tracking and only reports the distance at
screen/camera center using the RealSense depth frame.  Default sampling is the COLOR
screen center projected into depth pixels, matching the existing helipad preview frame;
use --center-frame depth for the raw depth-image center.

Examples:
  python3 center_depth.py --fps 30 --log /home/arduino/center_depth.csv
  python3 center_depth.py --center-frame depth --window-radius 4 --print-hz 10
  python3 center_depth.py --frames 150 --print-hz 5
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import signal
import statistics
import sys
import time
from dataclasses import dataclass
from typing import Iterable, Optional, Sequence



@dataclass
class DepthSample:
    frame: int
    t: float
    center_frame: str
    center_x: float
    center_y: float
    depth_x: Optional[int]
    depth_y: Optional[int]
    range_m: Optional[float]
    valid_count: int
    fps: float
    proc_ms: float
    status: str


def positive_finite_values(values: Iterable[float], min_range: float, max_range: float) -> list[float]:
    """Return finite positive ranges within [min_range, max_range]."""
    out: list[float] = []
    for value in values:
        try:
            v = float(value)
        except (TypeError, ValueError):
            continue
        if math.isfinite(v) and min_range <= v <= max_range:
            out.append(v)
    return out


def summarize_ranges(values: Sequence[float], stat: str) -> Optional[float]:
    """Summarize valid range samples; returns meters or None."""
    if not values:
        return None
    if stat == "mean":
        return float(sum(values) / len(values))
    if stat == "min":
        return float(min(values))
    if stat == "center":
        # Caller should pass center sample first when stat=center.
        return float(values[0])
    return float(statistics.median(values))


def sample_depth_window(depth_frame, x: int, y: int, radius: int,
                        min_range: float, max_range: float, stat: str) -> tuple[Optional[float], int]:
    """Sample a square window around depth pixel (x,y) using depth_frame.get_distance()."""
    width = int(depth_frame.get_width())
    height = int(depth_frame.get_height())
    if x < 0 or y < 0 or x >= width or y >= height:
        return None, 0

    coords: list[tuple[int, int]] = [(x, y)]
    if radius > 0 and stat != "center":
        coords = []
        x0, x1 = max(0, x - radius), min(width - 1, x + radius)
        y0, y1 = max(0, y - radius), min(height - 1, y + radius)
        for yy in range(y0, y1 + 1):
            for xx in range(x0, x1 + 1):
                coords.append((xx, yy))

    values = positive_finite_values(
        (depth_frame.get_distance(xx, yy) for xx, yy in coords),
        min_range,
        max_range,
    )
    return summarize_ranges(values, stat), len(values)


def project_color_center_to_depth(rs, depth_frame, scale: float, dintr, cintr, c2d, d2c,
                                  center_x: float, center_y: float,
                                  min_range: float, max_range: float) -> Optional[tuple[int, int]]:
    """Project a color-frame pixel into depth-frame coordinates without full-frame align."""
    try:
        px = rs.rs2_project_color_pixel_to_depth_pixel(
            depth_frame.get_data(),
            scale,
            min_range,
            max_range,
            dintr,
            cintr,
            c2d,
            d2c,
            [float(center_x), float(center_y)],
        )
        dx, dy = int(round(px[0])), int(round(px[1]))
        if 0 <= dx < depth_frame.get_width() and 0 <= dy < depth_frame.get_height():
            return dx, dy
    except Exception:
        return None
    return None


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description="RealSense center depth only; no helipad detector/tracker.")
    p.add_argument("--width", type=int, default=640, help="color/depth stream width")
    p.add_argument("--height", type=int, default=480, help="color/depth stream height")
    p.add_argument("--fps", type=int, default=30, help="RealSense hardware FPS; D435 normally supports 30/60/90")
    p.add_argument("--center-frame", choices=["color", "depth"], default="color",
                   help="color=screen center projected to depth (default), depth=raw depth image center")
    p.add_argument("--center-x", type=float, default=None, help="override center x in selected center-frame pixels")
    p.add_argument("--center-y", type=float, default=None, help="override center y in selected center-frame pixels")
    p.add_argument("--window-radius", type=int, default=3,
                   help="median window radius around center depth pixel; 3 means 7x7")
    p.add_argument("--stat", choices=["median", "mean", "min", "center"], default="median",
                   help="how to summarize valid samples in the window")
    p.add_argument("--min-range", type=float, default=0.10, help="minimum accepted depth in meters")
    p.add_argument("--max-range", type=float, default=6.00, help="maximum accepted depth in meters")
    p.add_argument("--warmup", type=int, default=15, help="frames to discard after pipeline start")
    p.add_argument("--print-hz", type=float, default=5.0, help="stdout print rate; 0 disables periodic prints")
    p.add_argument("--log", default="", help="optional CSV log path")
    p.add_argument("--fallback-depth-center", action="store_true",
                   help="when color-center projection fails, sample raw depth-image center instead of reporting None")
    p.add_argument("--frames", type=int, default=0, help="stop after N frames; 0 runs until Ctrl+C/systemd stop")
    p.add_argument("--self-test", action="store_true", help="run local pure-Python self-test without RealSense")
    return p


def run_self_test() -> int:
    class FakeDepth:
        def __init__(self):
            nan = float("nan")
            self.data = [
                [0.0, 0.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 1.1, 0.0, 0.0],
                [0.0, 1.2, 1.3, 1.4, 0.0],
                [0.0, nan, 1.5, 9.0, 0.0],
                [0.0, 0.0, 0.0, 0.0, 0.0],
            ]

        def get_width(self):
            return len(self.data[0])

        def get_height(self):
            return len(self.data)

        def get_distance(self, x, y):
            return float(self.data[y][x])

    value, count = sample_depth_window(FakeDepth(), 2, 2, 1, 0.1, 6.0, "median")
    assert count == 6, (value, count)
    assert abs(value - 1.25) < 1e-9, value
    value_center, count_center = sample_depth_window(FakeDepth(), 2, 2, 1, 0.1, 6.0, "center")
    assert count_center == 1 and abs(value_center - 1.3) < 1e-9, (value_center, count_center)
    print("self-test OK")
    return 0


def run(args: argparse.Namespace) -> int:
    try:
        import pyrealsense2 as rs
    except Exception as exc:
        print(f"ERROR: pyrealsense2 import failed: {exc}", file=sys.stderr, flush=True)
        return 2

    if args.window_radius < 0:
        print("ERROR: --window-radius must be >= 0", file=sys.stderr, flush=True)
        return 2
    if args.min_range <= 0 or args.max_range <= args.min_range:
        print("ERROR: require 0 < --min-range < --max-range", file=sys.stderr, flush=True)
        return 2

    stop = False

    def _stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGTERM, _stop)
    signal.signal(signal.SIGINT, _stop)

    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    if args.center_frame == "color":
        cfg.enable_stream(rs.stream.color, args.width, args.height, rs.format.bgr8, args.fps)

    logf = None
    logw = None
    samples: list[float] = []
    fps_buf: list[float] = []

    try:
        profile = pipe.start(cfg)
        device = profile.get_device()
        depth_sensor = device.first_depth_sensor()
        scale = float(depth_sensor.get_depth_scale())
        dprof = profile.get_stream(rs.stream.depth).as_video_stream_profile()
        dintr = dprof.get_intrinsics()
        cintr = c2d = d2c = None
        if args.center_frame == "color":
            cprof = profile.get_stream(rs.stream.color).as_video_stream_profile()
            cintr = cprof.get_intrinsics()
            c2d = cprof.get_extrinsics_to(dprof)
            d2c = dprof.get_extrinsics_to(cprof)
            center_x = args.center_x if args.center_x is not None else (cintr.width - 1) / 2.0
            center_y = args.center_y if args.center_y is not None else (cintr.height - 1) / 2.0
        else:
            center_x = args.center_x if args.center_x is not None else (dintr.width - 1) / 2.0
            center_y = args.center_y if args.center_y is not None else (dintr.height - 1) / 2.0

        for _ in range(max(0, args.warmup)):
            pipe.wait_for_frames()

        if args.log:
            os.makedirs(os.path.dirname(os.path.abspath(args.log)), exist_ok=True)
            logf = open(args.log, "w", newline="")
            logw = csv.writer(logf)
            logw.writerow([
                "frame", "t", "center_frame", "center_x", "center_y", "depth_x", "depth_y",
                "range_m", "valid_count", "fps", "proc_ms", "status",
            ])

        print(
            "CENTER_DEPTH_START "
            f"center_frame={args.center_frame} center=({center_x:.1f},{center_y:.1f}) "
            f"stream={args.width}x{args.height}@{args.fps}Hz window_radius={args.window_radius} "
            f"stat={args.stat} min={args.min_range} max={args.max_range} depth_scale={scale}",
            flush=True,
        )

        frame_idx = 0
        last_print = 0.0
        while not stop:
            loop_t0 = time.perf_counter()
            frames = pipe.wait_for_frames()
            depth = frames.get_depth_frame()
            if not depth:
                continue

            if args.center_frame == "color":
                projected = project_color_center_to_depth(
                    rs, depth, scale, dintr, cintr, c2d, d2c, center_x, center_y,
                    args.min_range, args.max_range,
                )
                if projected is None:
                    if args.fallback_depth_center:
                        dx, dy = int(round((dintr.width - 1) / 2.0)), int(round((dintr.height - 1) / 2.0))
                        status = "PROJECT_FAIL_DEPTH_CENTER_FALLBACK"
                    else:
                        dx = dy = None
                        status = "PROJECT_FAIL_NO_SAMPLE"
                else:
                    dx, dy = projected
                    status = "OK"
            else:
                dx, dy = int(round(center_x)), int(round(center_y))
                status = "OK"

            if dx is None or dy is None:
                rng, valid_count = None, 0
            else:
                rng, valid_count = sample_depth_window(
                    depth, dx, dy, args.window_radius, args.min_range, args.max_range, args.stat,
                )
            if rng is None:
                status = "NO_VALID_DEPTH" if status == "OK" else status + "+NO_VALID_DEPTH"
            else:
                samples.append(rng)

            proc_ms = (time.perf_counter() - loop_t0) * 1000.0
            # Regulate only if processing was faster than target period. wait_for_frames already blocks,
            # but this keeps CPU modest if a backend returns early.
            sleep_s = (1.0 / args.fps) - (time.perf_counter() - loop_t0)
            if sleep_s > 0:
                time.sleep(sleep_s)
            dt = time.perf_counter() - loop_t0
            fps = 1.0 / max(dt, 1e-9)
            fps_buf.append(fps)
            if len(fps_buf) > max(3, args.fps):
                fps_buf.pop(0)
            avg_fps = sum(fps_buf) / len(fps_buf)

            sample = DepthSample(
                frame=frame_idx,
                t=time.perf_counter(),
                center_frame=args.center_frame,
                center_x=center_x,
                center_y=center_y,
                depth_x=dx,
                depth_y=dy,
                range_m=rng,
                valid_count=valid_count,
                fps=avg_fps,
                proc_ms=proc_ms,
                status=status,
            )

            if logw:
                logw.writerow([
                    sample.frame, f"{sample.t:.3f}", sample.center_frame,
                    f"{sample.center_x:.1f}", f"{sample.center_y:.1f}",
                    sample.depth_x, sample.depth_y,
                    f"{sample.range_m:.3f}" if sample.range_m is not None else "",
                    sample.valid_count, f"{sample.fps:.1f}", f"{sample.proc_ms:.1f}", sample.status,
                ])
                if frame_idx % max(1, args.fps) == 0:
                    logf.flush()

            now = time.perf_counter()
            if args.print_hz > 0 and (now - last_print) >= (1.0 / args.print_hz):
                last_print = now
                range_text = f"{sample.range_m:.3f}" if sample.range_m is not None else "None"
                print(
                    "CENTER_DEPTH "
                    f"frame={sample.frame} range_m={range_text} valid={sample.valid_count} "
                    f"depth_px=({sample.depth_x},{sample.depth_y}) status={sample.status} "
                    f"fps={sample.fps:.1f} proc_ms={sample.proc_ms:.1f}",
                    flush=True,
                )

            frame_idx += 1
            if args.frames > 0 and frame_idx >= args.frames:
                break

        if samples:
            sorted_samples = sorted(samples)
            p50 = sorted_samples[len(sorted_samples) // 2]
            p95 = sorted_samples[min(len(sorted_samples) - 1, int(len(sorted_samples) * 0.95))]
            print(
                "CENTER_DEPTH_SUMMARY "
                f"frames={frame_idx} valid={len(samples)} invalid={frame_idx - len(samples)} "
                f"range_m_min={min(samples):.3f} range_m_p50={p50:.3f} "
                f"range_m_p95={p95:.3f} range_m_max={max(samples):.3f} "
                f"fps_avg={(sum(fps_buf)/len(fps_buf)) if fps_buf else 0:.1f}",
                flush=True,
            )
        else:
            print(f"CENTER_DEPTH_SUMMARY frames={frame_idx} valid=0 invalid={frame_idx} range_m=None", flush=True)
        return 0
    finally:
        try:
            pipe.stop()
        except Exception:
            pass
        if logf:
            logf.close()


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()
    if args.self_test:
        return run_self_test()
    return run(args)


if __name__ == "__main__":
    raise SystemExit(main())
