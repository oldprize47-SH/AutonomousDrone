"""
4-point click labeling tool for helipad marker.
Click 4 points on the circle edges (top/bottom/left/right) → auto bbox.
SPACE=save & next, R=redo points, S=skip, ESC=quit.
Saves YOLO format labels.
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
from pathlib import Path
import cv2

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_IMAGE_DIR = REPO_ROOT / "data" / "images"
DEFAULT_LABEL_DIR = REPO_ROOT / "data" / "labels"

CLASS_ID = 0
points = []


def mouse_cb(event, x, y, flags, param):
    global points
    if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
        points.append((x, y))


def main():
    global points

    parser = argparse.ArgumentParser(description="헬리패드 4점 클릭 라벨링 도구")
    parser.add_argument("--image-dir", type=Path, default=DEFAULT_IMAGE_DIR)
    parser.add_argument("--label-dir", type=Path, default=DEFAULT_LABEL_DIR)
    args = parser.parse_args()
    image_dir = args.image_dir.resolve()
    label_dir = args.label_dir.resolve()
    label_dir.mkdir(parents=True, exist_ok=True)

    images = sorted([f for f in os.listdir(image_dir) if f.lower().endswith(".jpg")])
    if not images:
        print("No images found")
        return

    unlabeled = []
    labeled = 0
    for f in images:
        label_f = f.replace(".jpg", ".txt")
        if os.path.exists(label_dir / label_f):
            labeled += 1
        else:
            unlabeled.append(f)

    if not unlabeled:
        unlabeled = images
        print(f"All {len(images)} already labeled. Re-labeling mode.")
    else:
        print(f"Labeled: {labeled}, Remaining: {len(unlabeled)}")

    win = "Label Tool"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.setMouseCallback(win, mouse_cb)

    idx = 0
    while idx < len(unlabeled):
        fname = unlabeled[idx]
        img = cv2.imread(str(image_dir / fname))
        if img is None:
            idx += 1
            continue

        h, w = img.shape[:2]
        points = []

        while True:
            display = img.copy()

            prog = f"[{idx+1}/{len(unlabeled)}] {fname}  pts:{len(points)}/4"
            cv2.putText(display, prog, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,0,0), 3)
            cv2.putText(display, prog, (8, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 1)

            guide = "Click 4 pts on circle edge | SPACE=save R=redo S=skip ESC=quit"
            cv2.putText(display, guide, (8, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,0), 3)
            cv2.putText(display, guide, (8, h-12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255,255,255), 1)

            for i, (px, py) in enumerate(points):
                cv2.circle(display, (px, py), 5, (0, 0, 255), -1)
                cv2.putText(display, str(i+1), (px+8, py-4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0,0,255), 1)

            if len(points) >= 4:
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                x1, y1 = min(xs), min(ys)
                x2, y2 = max(xs), max(ys)
                # add small padding
                pad = int(max(x2-x1, y2-y1) * 0.05)
                x1 = max(0, x1 - pad)
                y1 = max(0, y1 - pad)
                x2 = min(w, x2 + pad)
                y2 = min(h, y2 + pad)
                cv2.rectangle(display, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(display, "helipad - SPACE to save", (x1, y1-8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow(win, display)
            key = cv2.waitKey(30) & 0xFF

            if key == 27:  # ESC
                cv2.destroyAllWindows()
                print(f"\nDone. Labeled {idx} this session.")
                return

            elif key == ord(' ') and len(points) >= 4:  # SPACE = save
                xs = [p[0] for p in points]
                ys = [p[1] for p in points]
                x1, y1 = min(xs), min(ys)
                x2, y2 = max(xs), max(ys)
                pad = int(max(x2-x1, y2-y1) * 0.05)
                x1 = max(0, x1 - pad)
                y1 = max(0, y1 - pad)
                x2 = min(w, x2 + pad)
                y2 = min(h, y2 + pad)
                bw = x2 - x1
                bh = y2 - y1
                if bw > 5 and bh > 5:
                    cx = (x1 + bw/2) / w
                    cy = (y1 + bh/2) / h
                    nw = bw / w
                    nh = bh / h
                    label_path = label_dir / (Path(fname).stem + ".txt")
                    with open(label_path, "w") as f:
                        f.write(f"{CLASS_ID} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
                    print(f"  saved {fname}")
                idx += 1
                break

            elif key == ord('r'):  # R = redo
                points = []

            elif key == ord('s'):  # S = skip
                print(f"  skipped {fname}")
                idx += 1
                break

    cv2.destroyAllWindows()
    total = len([f for f in os.listdir(label_dir) if f.endswith(".txt")])
    print(f"\nAll done. Total labels: {total}")


if __name__ == "__main__":
    main()
