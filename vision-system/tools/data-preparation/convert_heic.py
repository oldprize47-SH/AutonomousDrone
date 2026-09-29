"""
HEIC (iPhone 사진) → JPG 변환.

휴대전화에서 찍은 HEIC 파일들을 data/images/에 JPG로 변환해 넣는다.
기존 이미지와 파일명 충돌하지 않게 번호를 자동으로 매김.
EXIF 회전 정보도 자동 보정.

사용:
  python tools/data-preparation/convert_heic.py
  python tools/data-preparation/convert_heic.py --src ./photos --dst ./data/images
  python tools/data-preparation/convert_heic.py --max-size 1920
"""

import sys
if sys.stdout.encoding and sys.stdout.encoding.lower() != "utf-8":
    try:
        sys.stdout.reconfigure(encoding="utf-8")
        sys.stderr.reconfigure(encoding="utf-8")
    except Exception:
        pass

import argparse
import glob
import os
from pathlib import Path
from PIL import Image, ImageOps

try:
    import pillow_heif
    pillow_heif.register_heif_opener()
except ImportError:
    print("pillow-heif 가 설치되지 않았습니다. 다음 명령으로 설치:")
    print("  pip install pillow-heif")
    sys.exit(1)

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_SRC = REPO_ROOT / "data" / "heic-input"
DEFAULT_DST = REPO_ROOT / "data" / "images"


def next_index(dst_dir):
    """images/, train/images/, val/images/ 전체에서 가장 큰 img_NNNNN 다음 번호.

    split_dataset.py를 다시 돌리면 train/val 이미지가 images/ 풀로 도로
    모이므로, dst 폴더만 보고 번호를 매기면 기존 라벨 데이터와 파일명이
    충돌해 덮어써진다. 세 폴더 전체의 최댓값 다음 번호부터 매긴다.
    """
    data_root = os.path.dirname(dst_dir)  # .../dataset
    search_dirs = [
        dst_dir,
        os.path.join(data_root, "train", "images"),
        os.path.join(data_root, "val", "images"),
    ]
    max_n = -1
    for d in search_dirs:
        for p in glob.glob(os.path.join(d, "img_*.jpg")):
            name = os.path.basename(p)
            try:
                n = int(name[4:].split(".")[0])
                max_n = max(max_n, n)
            except ValueError:
                continue
    return max_n + 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--src", type=Path, default=DEFAULT_SRC, help="HEIC 파일이 들어있는 폴더")
    p.add_argument("--dst", type=Path, default=DEFAULT_DST, help="JPG 출력 폴더")
    p.add_argument("--max-size", type=int, default=1920,
                   help="너무 큰 사진은 줄임 (긴 변 기준 픽셀, 기본 1920)")
    p.add_argument("--quality", type=int, default=92,
                   help="JPG 품질 (기본 92)")
    args = p.parse_args()

    if not args.src.is_dir():
        print(f"HEIC 폴더가 없습니다: {args.src}")
        print(f"폴더 만들고 HEIC 파일들 거기에 넣으세요.")
        args.src.mkdir(parents=True, exist_ok=True)
        print(f"빈 폴더 생성: {args.src}")
        return

    args.dst.mkdir(parents=True, exist_ok=True)

    # Windows glob 은 대소문자를 구분하지 않아 *.HEIC 와 *.heic 가 같은 파일을
    # 둘 다 잡는다. set 으로 중복 제거해 한 장이 두 번 변환되는 것을 막는다.
    heic_files = sorted(set(
        glob.glob(os.path.join(args.src, "*.HEIC")) +
        glob.glob(os.path.join(args.src, "*.heic"))
    ))
    if not heic_files:
        print(f"HEIC 파일 없음: {args.src}")
        return

    start_idx = next_index(args.dst)
    print(f"HEIC 폴더:    {args.src}")
    print(f"JPG 폴더:     {args.dst}")
    print(f"HEIC 파일 수: {len(heic_files)}")
    print(f"시작 번호:    img_{start_idx:05d}.jpg 부터")
    print()

    ok = 0
    fail = 0
    for i, src_path in enumerate(heic_files):
        try:
            img = Image.open(src_path)
            img = ImageOps.exif_transpose(img)  # 회전 정보 자동 보정
            img = img.convert("RGB")

            # 너무 크면 줄임 (긴 변 기준)
            w, h = img.size
            long_side = max(w, h)
            if long_side > args.max_size:
                scale = args.max_size / long_side
                img = img.resize((int(w * scale), int(h * scale)), Image.LANCZOS)

            idx = start_idx + ok
            dst_name = f"img_{idx:05d}.jpg"
            dst_path = os.path.join(args.dst, dst_name)
            img.save(dst_path, "JPEG", quality=args.quality)
            ok += 1
            print(f"  [{ok}/{len(heic_files)}] {os.path.basename(src_path)} → {dst_name} ({img.size[0]}x{img.size[1]})")
        except Exception as e:
            fail += 1
            print(f"  실패: {os.path.basename(src_path)} - {e}")

    print()
    print(f"=== 변환 완료 ===")
    print(f"  성공: {ok}")
    print(f"  실패: {fail}")
    print(f"  저장 위치: {args.dst}")
    print()
    print(f"다음 단계:")
    print("  1. python tools/data-preparation/auto_label.py --weights ./weights/best.pth")
    print("  2. python tools/data-preparation/review_labels.py")


if __name__ == "__main__":
    main()
