"""
학습된 모델로 새 이미지를 자동 라벨링.

best.pth를 사용해 data/images/ 의 라벨 없는 이미지들에 대해
추론을 돌려서 YOLO 형식 라벨을 자동 생성한다.
검수는 review_labels.py 로 수행.

사용:
  python tools/data-preparation/auto_label.py --weights ./weights/best.pth
  python tools/data-preparation/auto_label.py --weights ./weights/best.pth --conf 0.5
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
import torch
import cv2
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT))
from training.train import get_model, NUM_CLASSES, INPUT_SIZE

DEFAULT_IMG_DIR = REPO_ROOT / "data" / "images"
DEFAULT_LABEL_DIR = REPO_ROOT / "data" / "labels"
DEFAULT_WEIGHTS = REPO_ROOT / "weights" / "best.pth"
CLASS_ID = 0  # helipad


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--img-dir", type=Path, default=DEFAULT_IMG_DIR)
    p.add_argument("--label-dir", type=Path, default=DEFAULT_LABEL_DIR)
    p.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    p.add_argument("--conf", type=float, default=0.7, help="신뢰도 임계값 (높을수록 보수적)")
    p.add_argument("--overwrite", action="store_true", help="이미 라벨이 있는 이미지도 덮어쓰기")
    args = p.parse_args()

    args.label_dir.mkdir(parents=True, exist_ok=True)

    # 모델 로드
    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = get_model(NUM_CLASSES)
    state = torch.load(str(args.weights), map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device).eval()
    print(f"모델 로드: {args.weights} on {device}")
    print(f"이미지 폴더: {args.img_dir}")
    print(f"라벨 폴더:   {args.label_dir}")
    print(f"신뢰도 임계값: {args.conf}")
    print()

    images = sorted([f for f in os.listdir(args.img_dir) if f.lower().endswith(".jpg")])
    if not images:
        print(f"이미지 없음: {args.img_dir}")
        return

    stats = {"labeled": 0, "skipped": 0, "no_detect": 0, "already": 0}
    processed = []  # 자동 라벨링 시도한 파일 (검출 성공 + 실패 모두)

    for fname in images:
        label_path = args.label_dir / (Path(fname).stem + ".txt")

        if os.path.exists(label_path) and not args.overwrite:
            stats["already"] += 1
            continue

        img = cv2.imread(str(args.img_dir / fname))
        if img is None:
            stats["skipped"] += 1
            continue
        h, w = img.shape[:2]

        # 추론
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        t = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        t = torch.nn.functional.interpolate(
            t.unsqueeze(0), size=(INPUT_SIZE, INPUT_SIZE),
            mode="bilinear", align_corners=False
        ).squeeze(0).to(device)
        with torch.no_grad():
            preds = model([t])

        det = preds[0]
        boxes = det["boxes"].cpu().numpy()
        scores = det["scores"].cpu().numpy()
        labels = det["labels"].cpu().numpy()

        # conf 이상의 helipad만
        valid = []
        for box, score, lab in zip(boxes, scores, labels):
            if score < args.conf or lab == 0:
                continue
            x1 = box[0] * w / INPUT_SIZE
            y1 = box[1] * h / INPUT_SIZE
            x2 = box[2] * w / INPUT_SIZE
            y2 = box[3] * h / INPUT_SIZE
            valid.append((x1, y1, x2, y2, float(score)))

        if not valid:
            stats["no_detect"] += 1
            processed.append(fname)
            print(f"  [없음] {fname}")
            continue

        # 가장 높은 점수의 박스 하나만 저장 (헬리패드는 보통 한 개)
        valid.sort(key=lambda v: -v[4])
        x1, y1, x2, y2, score = valid[0]

        cx = (x1 + x2) / 2 / w
        cy = (y1 + y2) / 2 / h
        nw = (x2 - x1) / w
        nh = (y2 - y1) / h

        with open(label_path, "w") as f:
            f.write(f"{CLASS_ID} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")
        stats["labeled"] += 1
        processed.append(fname)
        print(f"  [라벨] {fname}  score={score:.3f}")

    # 처리한 파일 목록 저장 (검수 도구가 이것만 보여줌)
    log_path = args.img_dir.parent / ".last_auto_labeled.txt"
    with open(log_path, "w", encoding="utf-8") as f:
        for fname in processed:
            f.write(fname + "\n")

    print()
    print(f"=== 결과 ===")
    print(f"  라벨 생성: {stats['labeled']}")
    print(f"  검출 없음: {stats['no_detect']}  ← 검수에서 수동 라벨링 필요")
    print(f"  기존 라벨 유지: {stats['already']}")
    print(f"  읽기 실패: {stats['skipped']}")
    print()
    print(f"검수 대상: {len(processed)}장 (자동 라벨링 시도한 사진들)")
    print(f"기록 파일: {log_path}")
    print()
    print("다음: python tools/data-preparation/review_labels.py 로 검수")


if __name__ == "__main__":
    main()
