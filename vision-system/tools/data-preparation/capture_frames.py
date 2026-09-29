"""
RealSense D435 캡처 스크립트 (1-class detection 학습용 데이터 수집)

사용법:
    python tools/data-preparation/capture_frames.py --output-dir ./data

키 조작:
    SPACE  : 현재 프레임 저장 (RGB + depth)
    B      : 자동 연속 캡처 ON/OFF (0.3초마다 1장)
    R      : 마지막 저장 사진 삭제
    ESC/Q  : 종료

권장 사항:
    - 다양한 각도, 거리, 조명, 배경에서 100~300장 수집
    - 객체를 화면 가장자리/중앙 다양한 위치에 두기
    - 일부는 객체 없는 사진 (negative samples)도 포함
"""

import sys
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

import argparse
import os
import time
from pathlib import Path
import numpy as np
import cv2
import pyrealsense2 as rs

# === 기본 설정 ===
REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_OUTPUT_DIR = REPO_ROOT / "data"
WIDTH, HEIGHT, FPS = 640, 480, 30
AUTO_BURST_INTERVAL = 0.3  # 자동 캡처 간격 (초)


def get_next_index(img_dir):
    """기존 파일들 중 가장 큰 번호 다음을 반환"""
    files = [f for f in os.listdir(img_dir) if f.startswith("img_") and f.endswith(".jpg")]
    if not files:
        return 0
    nums = []
    for f in files:
        try:
            nums.append(int(f[4:-4]))
        except ValueError:
            pass
    return (max(nums) + 1) if nums else 0


def select_depth_frame(frames, save_depth):
    """Depth 저장이 켜진 경우에만 현재 depth frame을 요청한다."""
    return frames.get_depth_frame() if save_depth else None


def main():
    parser = argparse.ArgumentParser(description="RealSense RGB/depth 데이터 수집")
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help="images/depth/camera_intrinsics.txt를 저장할 데이터 루트")
    parser.add_argument("--no-depth", action="store_true", help="depth 프레임을 저장하지 않음")
    parser.add_argument("--burst-interval", type=float, default=AUTO_BURST_INTERVAL,
                        help="자동 연속 캡처 간격(초)")
    args = parser.parse_args()

    save_dir = args.output_dir.resolve()
    img_dir = save_dir / "images"
    depth_dir = save_dir / "depth"
    save_depth = not args.no_depth
    img_dir.mkdir(parents=True, exist_ok=True)
    if save_depth:
        depth_dir.mkdir(parents=True, exist_ok=True)

    # RealSense 파이프라인 설정
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, WIDTH, HEIGHT, rs.format.bgr8, FPS)
    if save_depth:
        cfg.enable_stream(rs.stream.depth, WIDTH, HEIGHT, rs.format.z16, FPS)
    profile = pipe.start(cfg)

    # depth/color 정렬 (depth를 color 좌표계로 매핑)
    align = rs.align(rs.stream.color) if save_depth else None

    # 카메라 intrinsics 출력 + 저장 (시선각 계산용)
    color_stream = profile.get_stream(rs.stream.color).as_video_stream_profile()
    intr = color_stream.get_intrinsics()
    print(f"\n=== 카메라 Intrinsics (시선각 계산용으로 저장됨) ===")
    print(f"  width:  {intr.width}")
    print(f"  height: {intr.height}")
    print(f"  fx, fy: {intr.fx:.2f}, {intr.fy:.2f}")
    print(f"  cx, cy: {intr.ppx:.2f}, {intr.ppy:.2f}")
    print(f"  model:  {intr.model}")
    print(f"  coeffs: {intr.coeffs}")

    intr_path = save_dir / "camera_intrinsics.txt"
    with open(intr_path, "w") as f:
        f.write(f"width {intr.width}\n")
        f.write(f"height {intr.height}\n")
        f.write(f"fx {intr.fx}\n")
        f.write(f"fy {intr.fy}\n")
        f.write(f"cx {intr.ppx}\n")
        f.write(f"cy {intr.ppy}\n")
        f.write(f"model {intr.model}\n")
        f.write(f"coeffs {' '.join(map(str, intr.coeffs))}\n")
    print(f"  -> saved to {intr_path}")

    idx = get_next_index(img_dir)
    print(f"\n다음 저장 번호: img_{idx:05d}")
    print("키: SPACE=저장 / B=자동연속 / R=마지막삭제 / ESC=종료\n")

    auto_burst = False
    last_burst_time = 0
    last_saved_idx = -1
    saved_count_session = 0
    start_time = time.time()

    try:
        while True:
            frames = pipe.wait_for_frames()
            if align is not None:
                frames = align.process(frames)

            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            color = np.asanyarray(color_frame.get_data())

            depth_frame = select_depth_frame(frames, save_depth)

            # 프리뷰에 오버레이
            display = color.copy()
            # 중앙 십자선
            h, w = display.shape[:2]
            cv2.line(display, (w//2-15, h//2), (w//2+15, h//2), (0,255,255), 1)
            cv2.line(display, (w//2, h//2-15), (w//2, h//2+15), (0,255,255), 1)

            elapsed = time.time() - start_time
            fps_actual = (saved_count_session + 1) / elapsed if elapsed > 0 else 0

            # 상태 표시
            info = [
                f"Saved this session: {saved_count_session}  (next: img_{idx:05d})",
                f"AUTO BURST: {'ON' if auto_burst else 'off'}  | depth: {'on' if save_depth else 'off'}",
                f"SPACE=save  B=burst  R=undo  ESC=quit",
            ]
            for i, t in enumerate(info):
                y = 20 + i * 22
                cv2.putText(display, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,0), 3)
                cv2.putText(display, t, (8, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

            if auto_burst:
                cv2.circle(display, (w-20, 20), 8, (0,0,255), -1)

            cv2.imshow("RealSense capture", display)

            save_now = False
            key = cv2.waitKey(1) & 0xFF

            if key == 27 or key == ord('q'):  # ESC or Q
                break
            elif key == ord(' '):
                save_now = True
            elif key == ord('b'):
                auto_burst = not auto_burst
                last_burst_time = time.time()
                print(f"AUTO BURST: {'ON' if auto_burst else 'off'}")
            elif key == ord('r'):
                if last_saved_idx >= 0:
                    img_path = img_dir / f"img_{last_saved_idx:05d}.jpg"
                    dep_path = depth_dir / f"img_{last_saved_idx:05d}.png"
                    for p in (img_path, dep_path):
                        if os.path.exists(p):
                            os.remove(p)
                    print(f"  Deleted img_{last_saved_idx:05d}")
                    idx = last_saved_idx
                    last_saved_idx = -1
                    saved_count_session = max(0, saved_count_session - 1)

            # auto burst
            if auto_burst and (time.time() - last_burst_time) >= args.burst_interval:
                save_now = True
                last_burst_time = time.time()

            if save_now:
                img_path = img_dir / f"img_{idx:05d}.jpg"
                # 한글 경로 대응: cv2.imwrite는 비-ASCII 경로 실패하므로 imencode + open() 사용
                ok_jpg, enc = cv2.imencode(".jpg", color, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if ok_jpg:
                    with open(img_path, "wb") as f:
                        f.write(enc.tobytes())
                if depth_frame is not None:
                    depth_np = np.asanyarray(depth_frame.get_data())
                    dep_path = depth_dir / f"img_{idx:05d}.png"
                    ok_dep, enc_d = cv2.imencode(".png", depth_np)
                    if ok_dep:
                        with open(dep_path, "wb") as f:
                            f.write(enc_d.tobytes())
                # 실제 파일이 생성됐는지 확인
                if os.path.exists(img_path) and os.path.getsize(img_path) > 0:
                    print(f"  saved img_{idx:05d}  ({os.path.getsize(img_path)//1024} KB)")
                    last_saved_idx = idx
                    idx += 1
                    saved_count_session += 1
                else:
                    print(f"  FAILED img_{idx:05d}  path={img_path}")

    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        print(f"\n=== 종료 ===")
        print(f"이번 세션 저장: {saved_count_session}장")
        print(f"전체 저장된 이미지: {len(os.listdir(img_dir))}장")
        print(f"위치: {img_dir}")


if __name__ == "__main__":
    main()
