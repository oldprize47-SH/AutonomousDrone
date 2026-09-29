"""RealSense 메인 루프의 단계별 지연을 계측한다.

`wait_for_frames`, depth-to-color 정렬, NumPy 복사, grayscale 변환의 p50/p95/max를
분리해 Uno Q 배포 병목을 확인한다. 결과에는 환경 변수나 자격 증명을 출력하지 않는다.
"""

import argparse
import time

import cv2
import numpy as np
import pyrealsense2 as rs


def percentile_summary(values):
    ordered = sorted(values)
    return {
        "p50": ordered[len(ordered) // 2],
        "p95": ordered[min(int(len(ordered) * 0.95), len(ordered) - 1)],
        "max": max(ordered),
    }


def print_summary(name, values):
    summary = percentile_summary(values)
    print(
        f"{name}: p50={summary['p50']:.1f}ms  "
        f"p95={summary['p95']:.1f}ms  max={summary['max']:.1f}ms"
    )


def run_benchmark(args):
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(
        rs.stream.color, args.color_width, args.color_height, rs.format.bgr8, args.fps,
    )
    config.enable_stream(
        rs.stream.depth, args.depth_width, args.depth_height, rs.format.z16, args.fps,
    )
    pipeline.start(config)
    align = rs.align(rs.stream.color)

    waits, aligns, copies, grayscale = [], [], [], []
    try:
        for _ in range(args.warmup_frames):
            pipeline.wait_for_frames()

        for _ in range(args.frames):
            start = time.perf_counter()
            frames = pipeline.wait_for_frames()
            after_wait = time.perf_counter()
            aligned_frames = align.process(frames)
            after_align = time.perf_counter()
            color = np.asanyarray(aligned_frames.get_color_frame().get_data())
            np.asanyarray(aligned_frames.get_depth_frame().get_data())
            after_copy = time.perf_counter()
            cv2.cvtColor(color, cv2.COLOR_BGR2GRAY)
            after_grayscale = time.perf_counter()

            waits.append((after_wait - start) * 1000)
            aligns.append((after_align - after_wait) * 1000)
            copies.append((after_copy - after_align) * 1000)
            grayscale.append((after_grayscale - after_copy) * 1000)
    finally:
        pipeline.stop()

    print_summary("wait_for_frames", waits)
    print_summary("align.process", aligns)
    print_summary("numpy copy x2", copies)
    print_summary("cvtColor gray", grayscale)


def parse_args():
    parser = argparse.ArgumentParser(description="RealSense 캡처/정렬 지연 벤치마크")
    parser.add_argument("--color-width", type=int, default=640)
    parser.add_argument("--color-height", type=int, default=480)
    parser.add_argument("--depth-width", type=int, default=480)
    parser.add_argument("--depth-height", type=int, default=270)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    parser.add_argument("--frames", type=int, default=150)
    args = parser.parse_args()
    if args.frames <= 0:
        parser.error("--frames must be positive")
    return args


if __name__ == "__main__":
    run_benchmark(parse_args())
