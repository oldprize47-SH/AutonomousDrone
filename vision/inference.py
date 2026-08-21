"""
Helipad Detection on Uno Q via NCNN — 512x512 model post-processing and V4L2 diagnostic path.

Pipeline: RealSense (V4L2 /dev/video6) -> NCNN CPU FP16 -> bbox overlay.

Usage:
  python3 inference_224.py                # GUI, 32 FPS
  python3 inference_224.py --headless     # no display
  python3 inference_224.py --fps 25       # lower target
  python3 inference_224.py --conf 0.5
"""
import os
import sys
import time
import argparse
import numpy as np
import cv2
import ncnn

INPUT_SIZE = 512  # [Phase2] train.py 와 동일 (320→512)


# ---- Anchor generation (train.py 의 DefaultBoxGenerator 와 정확히 일치) ----
def generate_anchors():
    """[Phase2] train.py 의 anchor 설정과 bit 단위로 일치해야 함.
    셋 중 하나라도 train 과 다르면 NCNN 박스가 깨진다:
      - feature_maps: 512 입력 → [32,16,8,4,2,1]
      - aspect_ratios: [[2]]*6 (helipad 원형이라 ratio 3 제거)
      - scales: train DefaultBoxGenerator(scales=[...]) 와 동일한 명시값(선형 아님)"""
    feature_maps = [32, 16, 8, 4, 2, 1]
    aspect_ratios_per_layer = [[2]] * 6
    scales = [0.040, 0.080, 0.160, 0.260, 0.400, 0.600, 0.850]  # 7개 = 6 fm + 1
    anchors = []
    for layer_idx, fm_size in enumerate(feature_maps):
        sk = scales[layer_idx]
        sk1 = scales[layer_idx + 1]
        for i in range(fm_size):
            for j in range(fm_size):
                cy = (i + 0.5) / fm_size
                cx = (j + 0.5) / fm_size
                anchors.append([cx, cy, sk, sk])
                extra_s = np.sqrt(sk * sk1)
                anchors.append([cx, cy, extra_s, extra_s])
                for ar in aspect_ratios_per_layer[layer_idx]:
                    w = sk * np.sqrt(ar)
                    h = sk / np.sqrt(ar)
                    anchors.append([cx, cy, w, h])
                    anchors.append([cx, cy, h, w])
    return np.array(anchors, dtype=np.float32)


def decode_boxes(box_preds, anchors):
    cx = box_preds[:, 0] / 10.0 * anchors[:, 2] + anchors[:, 0]
    cy = box_preds[:, 1] / 10.0 * anchors[:, 3] + anchors[:, 1]
    w = np.exp(box_preds[:, 2] / 5.0) * anchors[:, 2]
    h = np.exp(box_preds[:, 3] / 5.0) * anchors[:, 3]
    x1 = np.clip(cx - w / 2, 0, 1)
    y1 = np.clip(cy - h / 2, 0, 1)
    x2 = np.clip(cx + w / 2, 0, 1)
    y2 = np.clip(cy + h / 2, 0, 1)
    return np.stack([x1, y1, x2, y2], axis=1)


def nms(boxes, scores, iou_thresh=0.45, max_dets=5):
    if len(boxes) == 0: return []
    x1, y1, x2, y2 = boxes[:, 0], boxes[:, 1], boxes[:, 2], boxes[:, 3]
    areas = (x2 - x1) * (y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size > 0 and len(keep) < max_dets:
        i = order[0]
        keep.append(i)
        xx1 = np.maximum(x1[i], x1[order[1:]])
        yy1 = np.maximum(y1[i], y1[order[1:]])
        xx2 = np.minimum(x2[i], x2[order[1:]])
        yy2 = np.minimum(y2[i], y2[order[1:]])
        inter = np.maximum(0, xx2 - xx1) * np.maximum(0, yy2 - yy1)
        iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
        order = order[np.where(iou <= iou_thresh)[0] + 1]
    return keep


class CameraV4L2:
    def __init__(self, w, h, fps, device=6):
        self.cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, h)
        self.cap.set(cv2.CAP_PROP_FPS, fps)
        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open /dev/video{device}")
        for _ in range(5): self.cap.read()
    def read(self):
        ret, frame = self.cap.read()
        return frame if ret else None
    def stop(self): self.cap.release()


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--param", default="ssdlite_v3small_helipad.param")
    p.add_argument("--bin", default="ssdlite_v3small_helipad.bin")
    p.add_argument("--conf", type=float, default=0.5)
    p.add_argument("--fps", type=int, default=32, help="Target FPS")
    p.add_argument("--threads", type=int, default=4)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--v4l2-dev", type=int, default=6)
    p.add_argument("--headless", action="store_true")
    args = p.parse_args()

    here = os.path.dirname(os.path.abspath(__file__))
    param_path = args.param if os.path.isabs(args.param) else os.path.join(here, args.param)
    bin_path = args.bin if os.path.isabs(args.bin) else os.path.join(here, args.bin)

    print(f"Param: {param_path}")
    print(f"Bin:   {bin_path}")
    print(f"Target {args.fps} FPS = {1_000_000//args.fps} us/frame")

    # NCNN load
    net = ncnn.Net()
    net.opt.use_vulkan_compute = False
    net.opt.use_fp16_packed = True
    net.opt.use_fp16_storage = True
    net.opt.use_fp16_arithmetic = True
    net.opt.num_threads = args.threads
    net.load_param(param_path)
    net.load_model(bin_path)

    anchors = generate_anchors()
    print(f"Anchors: {len(anchors)} (512 model)")

    FRAME_INTERVAL_S = 1.0 / args.fps
    # PyTorch SSD uses softmax across classes. For 2-class:
    # p(helipad) = sigmoid(logit_helipad - logit_bg) > conf
    # <=> logit_helipad - logit_bg > logit(conf)
    logit_diff_thresh = np.log(args.conf / (1.0 - args.conf))

    # Set camera to 60 FPS to avoid V4L2 blocking at 30 FPS
    cam = CameraV4L2(args.width, args.height, 60, device=args.v4l2_dev)
    print(f"Camera: /dev/video{args.v4l2_dev} ({args.width}x{args.height} @ 60 FPS)")

    fps_buf = []
    frame_count = 0

    try:
        while True:
            t0 = time.perf_counter()
            frame = cam.read()
            if frame is None: continue
            h, w = frame.shape[:2]

            # Preprocess: resize -> 224, BGR -> RGB
            inp = cv2.resize(frame, (INPUT_SIZE, INPUT_SIZE))
            inp = cv2.cvtColor(inp, cv2.COLOR_BGR2RGB)

            mat_in = ncnn.Mat.from_pixels(inp, ncnn.Mat.PixelType.PIXEL_RGB, INPUT_SIZE, INPUT_SIZE)
            # ImageNet normalization: pixels are 0-255 from from_pixels
            # Need: (pixel/255 - mean) / std = pixel * (1/(255*std)) - mean/std
            # substract_mean_normalize does: (pixel - mean) * norm
            # So: mean_arg = 255 * imagenet_mean, norm_arg = 1 / (255 * imagenet_std)
            mat_in.substract_mean_normalize(
                [0.485*255, 0.456*255, 0.406*255],
                [1.0/(0.229*255), 1.0/(0.224*255), 1.0/(0.225*255)]
            )

            # Inference
            ex = net.create_extractor()
            ex.input("input", mat_in)
            ret_b, mat_boxes = ex.extract("boxes")
            ret_s, mat_scores = ex.extract("scores")

            # NCNN already gives (N, C) format - no reshape needed!
            box_preds = np.array(mat_boxes)    # (1602, 4)
            score_preds = np.array(mat_scores) # (1602, 2)

            # Softmax: p(helipad) = sigmoid(logit_helipad - logit_bg)
            logit_diff = score_preds[:, 1] - score_preds[:, 0]
            mask = logit_diff > logit_diff_thresh

            keep = []
            if mask.sum() > 0:
                fl = logit_diff[mask]
                fs = 1.0 / (1.0 + np.exp(-fl))
                fb = box_preds[mask]
                fa = anchors[mask]
                decoded = decode_boxes(fb, fa)
                keep = nms(decoded, fs, max_dets=5)

                if not args.headless:
                    for idx in keep:
                        x1 = int(decoded[idx, 0] * w)
                        y1 = int(decoded[idx, 1] * h)
                        x2 = int(decoded[idx, 2] * w)
                        y2 = int(decoded[idx, 3] * h)
                        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.putText(frame, f"HELIPAD {fs[idx]:.2f}",
                                    (x1, max(y1-6, 12)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                        cx = (x1 + x2) // 2
                        cy = (y1 + y2) // 2
                        cv2.circle(frame, (cx, cy), 4, (0, 0, 255), -1)

            # Frame timing
            elapsed = time.perf_counter() - t0
            sleep_s = FRAME_INTERVAL_S - elapsed
            if sleep_s > 0: time.sleep(sleep_s)
            actual = time.perf_counter() - t0
            fps = 1.0 / max(actual, 1e-6)
            fps_buf.append(fps)
            if len(fps_buf) > 30: fps_buf.pop(0)
            avg_fps = sum(fps_buf) / len(fps_buf)
            min_fps = min(fps_buf)
            color = (0, 255, 255) if min_fps >= args.fps * 0.95 else (0, 0, 255)
            cv2.putText(frame, f"FPS:{avg_fps:.0f} min:{min_fps:.0f} target:{args.fps}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            if elapsed > FRAME_INTERVAL_S:
                cv2.putText(frame, f"OVERRUN +{(elapsed-FRAME_INTERVAL_S)*1000:.1f}ms",
                            (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            frame_count += 1
            if args.headless:
                if frame_count % args.fps == 0:
                    flag = ' OVERRUN' if elapsed > FRAME_INTERVAL_S else ''
                    print(f"[{frame_count}] FPS={avg_fps:.1f} min={min_fps:.1f} infer={elapsed*1000:.1f}ms dets={len(keep)}{flag}")
            else:
                cv2.imshow("Helipad 224", frame)
                if cv2.waitKey(1) == 27: break
    except KeyboardInterrupt:
        pass
    finally:
        cam.stop()
        if not args.headless: cv2.destroyAllWindows()
        if fps_buf:
            print(f"\n=== Summary ===")
            print(f"Frames: {frame_count}")
            print(f"Avg FPS: {sum(fps_buf)/len(fps_buf):.1f}")
            print(f"Min FPS: {min(fps_buf):.1f}")
            print(f"Target:  {args.fps} FPS")
            if min(fps_buf) >= args.fps * 0.99:
                print("[OK] Min FPS target met!")
            else:
                print(f"[!] Min FPS short by {args.fps - min(fps_buf):.1f}")


if __name__ == "__main__":
    main()
