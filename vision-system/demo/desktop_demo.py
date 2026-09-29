"""
PC 실시간 검출: RealSense 카메라로 헬리패드를 실시간 검출 + depth 거리 표시.
입력 크기는 train.py 의 INPUT_SIZE 를 따름(현재 512). ESC 종료.

사용:
    python demo/desktop_demo.py --weights ./weights/best.pth
"""

import sys
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

import argparse
import time
from pathlib import Path
import cv2
import numpy as np
import torch
import pyrealsense2 as rs

REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))
from training.train import get_model, NUM_CLASSES, INPUT_SIZE

DEFAULT_WEIGHTS = REPO_ROOT / "weights" / "best.pth"


def load_model(weights, device):
    model = get_model(NUM_CLASSES)
    state = torch.load(str(weights), map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device)
    model.eval()
    print(f"Model loaded: {weights} on {device}")
    return model


def setup_realsense(width, height, fps, warmup_frames):
    pipe = rs.pipeline()
    cfg = rs.config()
    cfg.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
    cfg.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
    pipe.start(cfg)
    for _ in range(warmup_frames):
        pipe.wait_for_frames()
    print(f"RealSense ready ({width}x{height} @ {fps} FPS)")
    return pipe


def preprocess(frame_bgr):
    rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)
    t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
    t = torch.nn.functional.interpolate(
        t.unsqueeze(0), size=(INPUT_SIZE, INPUT_SIZE),
        mode="bilinear", align_corners=False
    ).squeeze(0)
    return t


def main():
    parser = argparse.ArgumentParser(description="RealSense SSDLite 데스크톱 검출 데모")
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--width", type=int, default=640)
    parser.add_argument("--height", type=int, default=480)
    parser.add_argument("--fps", type=int, default=30)
    parser.add_argument("--warmup-frames", type=int, default=30)
    args = parser.parse_args()

    weights = args.weights.resolve()
    if not weights.is_file():
        parser.error("--weights file does not exist")
    device = torch.device(args.device)
    model = load_model(weights, device)
    pipe = setup_realsense(args.width, args.height, args.fps, args.warmup_frames)

    print(f"\n=== {INPUT_SIZE}px model visual check ===")
    print("Press ESC to quit\n")

    fps_list = []
    try:
        while True:
            t0 = time.perf_counter()
            frames = pipe.wait_for_frames()
            color = frames.get_color_frame()
            depth = frames.get_depth_frame()
            if not color: continue

            frame = np.asanyarray(color.get_data())
            h, w = frame.shape[:2]

            inp = preprocess(frame).to(device)
            with torch.no_grad():
                preds = model([inp])

            if preds:
                det = preds[0]
                boxes = det["boxes"].cpu().numpy()
                scores = det["scores"].cpu().numpy()
                labels = det["labels"].cpu().numpy()
                n_total = int((scores >= args.confidence).sum())
                max_score = float(scores.max()) if scores.size else 0.0

                for box, score, label in zip(boxes, scores, labels):
                    if score < args.confidence or label == 0:
                        continue
                    # Scale model input coordinates -> original frame
                    x1 = int(box[0] * w / INPUT_SIZE)
                    y1 = int(box[1] * h / INPUT_SIZE)
                    x2 = int(box[2] * w / INPUT_SIZE)
                    y2 = int(box[3] * h / INPUT_SIZE)
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    text = f"HELIPAD {score:.2f}"
                    cv2.putText(frame, text, (x1, max(y1 - 6, 12)),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    cx_, cy_ = (x1 + x2) // 2, (y1 + y2) // 2
                    cv2.circle(frame, (cx_, cy_), 5, (0, 0, 255), -1)
                    d = depth.get_distance(min(max(cx_, 0), w-1), min(max(cy_, 0), h-1))
                    if d > 0:
                        cv2.putText(frame, f"{d:.2f}m", (cx_ + 6, cy_),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

                cv2.putText(frame, f"dets:{n_total} max_score:{max_score:.2f}",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

            elapsed = time.perf_counter() - t0
            fps_list.append(1.0 / elapsed)
            if len(fps_list) > 30: fps_list.pop(0)
            avg = sum(fps_list) / len(fps_list)
            cv2.putText(frame, f"FPS: {avg:.0f}  Model: {INPUT_SIZE}x{INPUT_SIZE} V3-Small",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

            cv2.imshow(f"Helipad {INPUT_SIZE} - Visual check", frame)
            if cv2.waitKey(1) == 27:
                break
    finally:
        pipe.stop()
        cv2.destroyAllWindows()
        if fps_list:
            print(f"Avg FPS: {sum(fps_list)/len(fps_list):.1f}")


if __name__ == "__main__":
    main()
