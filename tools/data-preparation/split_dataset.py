"""
데이터셋 분리 자동화 (validation 15% / training 85% 기본값).

기존 train/val에 흩어진 이미지를 모두 모은 뒤, 전체의 15%를 val,
나머지를 train으로 무작위 재분리한다.

사용:
  python tools/data-preparation/split_dataset.py --seed 42
  python tools/data-preparation/split_dataset.py --val-ratio 0.2 --seed 42

매번 실행할 때마다 무작위로 다시 섞이므로, 새 데이터 추가 시 그냥 다시 돌리면 된다.
"""

import argparse
import os
import random
import shutil
import sys
from pathlib import Path

# Windows cmd에서 한글 출력 문제 방지
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_DATA_DIR = REPO_ROOT / "data"


def build_dirs(data_dir):
    data_dir = Path(data_dir)
    return {
        "img_pool": data_dir / "images",
        "lbl_pool": data_dir / "labels",
        "train_img": data_dir / "train" / "images",
        "train_lbl": data_dir / "train" / "labels",
        "val_img": data_dir / "val" / "images",
        "val_lbl": data_dir / "val" / "labels",
    }


def ensure_dirs(dirs):
    for directory in dirs.values():
        directory.mkdir(parents=True, exist_ok=True)


def move_files_to_pool(dirs):
    """train/val에 있던 이미지·라벨을 data/images, data/labels 풀로 모은다."""
    moved = 0
    for src_img, src_lbl in [(dirs["train_img"], dirs["train_lbl"]), (dirs["val_img"], dirs["val_lbl"])]:
        if not os.path.isdir(src_img):
            continue
        for f in os.listdir(src_img):
            if not f.lower().endswith(".jpg"):
                continue
            shutil.move(src_img / f, dirs["img_pool"] / f)
            moved += 1
            lbl = f.replace(".jpg", ".txt")
            lbl_path = os.path.join(src_lbl, lbl)
            if os.path.exists(lbl_path):
                shutil.move(lbl_path, dirs["lbl_pool"] / lbl)
    return moved


def split(dirs, val_ratio: float, seed: int | None):
    imgs = [f for f in os.listdir(dirs["img_pool"]) if f.lower().endswith(".jpg")]
    total = len(imgs)
    if total == 0:
        print("이미지가 없습니다. data/images/와 train/val 폴더를 확인하세요.")
        return

    n_val = max(1, round(total * val_ratio))
    if seed is not None:
        random.seed(seed)
    random.shuffle(imgs)

    val_set = imgs[:n_val]
    train_set = imgs[n_val:]

    # val로 이동
    for f in val_set:
        shutil.move(dirs["img_pool"] / f, dirs["val_img"] / f)
        lbl = f.replace(".jpg", ".txt")
        lbl_path = dirs["lbl_pool"] / lbl
        if os.path.exists(lbl_path):
            shutil.move(lbl_path, dirs["val_lbl"] / lbl)

    # train으로 이동
    for f in train_set:
        shutil.move(dirs["img_pool"] / f, dirs["train_img"] / f)
        lbl = f.replace(".jpg", ".txt")
        lbl_path = dirs["lbl_pool"] / lbl
        if os.path.exists(lbl_path):
            shutil.move(lbl_path, dirs["train_lbl"] / lbl)

    return total, n_val, len(train_set)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR,
                   help="images/labels/train/val을 포함하는 데이터 루트")
    p.add_argument("--val-ratio", type=float, default=0.15, help="검증셋 비율 (기본 0.15 = 15%%)")
    p.add_argument("--seed", type=int, default=None, help="무작위 시드 (재현 가능한 분리하려면 지정)")
    args = p.parse_args()

    print(f"검증셋 비율: {args.val_ratio*100:.0f}%")
    if args.seed is not None:
        print(f"무작위 시드: {args.seed}")

    dirs = build_dirs(args.data_dir.resolve())
    ensure_dirs(dirs)
    print("\n[1/2] 기존 train/val 데이터를 풀로 모으는 중...")
    moved = move_files_to_pool(dirs)
    print(f"  -> {moved}장 풀로 이동")

    print("\n[2/2] 무작위 재분리 중...")
    result = split(dirs, args.val_ratio, args.seed)
    if result is None:
        return
    total, n_val, n_train = result

    print(f"\n=== 분리 완료 ===")
    print(f"  전체:    {total}장")
    print(f"  훈련셋:  {n_train}장 ({(n_train/total)*100:.1f}%)")
    print(f"  검증셋:  {n_val}장 ({(n_val/total)*100:.1f}%)")
    print("\n다음 단계: python training/train.py --data-dir ./data")


if __name__ == "__main__":
    main()
