"""
자동 라벨 검수 + 수동 재라벨링 도구.

기본 동작:
  auto_label.py가 처리한 사진들만 검수한다
  (`data/.last_auto_labeled.txt` 목록 참조).
  학습에 쓰였던 train/val 사진들은 검수 대상에서 제외.

키:
  - y / Space  : 라벨 OK, 다음 이미지로
  - n          : 라벨 잘못됨 → 4점 클릭으로 수동 라벨링
  - d          : 라벨 삭제 (이 이미지엔 헬리패드 없음)
  - b          : 이전 이미지로 되돌아가기
  - ESC        : 종료

수동 라벨링 모드 (n 누른 후):
  - 헬리패드 네 변에 점 4개 클릭 (위/아래/왼쪽/오른쪽)
  - u          : 마지막 점 되돌리기
  - r          : 점 다 지우고 다시 그리기
  - y / Space  : 4점 다 찍힌 후 저장
  - ESC        : 취소

옵션:
  python tools/data-preparation/review_labels.py
  python tools/data-preparation/review_labels.py --all
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
import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_IMAGE_DIR = REPO_ROOT / "data" / "images"
DEFAULT_LABEL_DIR = REPO_ROOT / "data" / "labels"
DEFAULT_AUTO_LOG = REPO_ROOT / "data" / ".last_auto_labeled.txt"
CLASS_ID = 0

WIN = "Label Review"


def load_label(path, img_w, img_h):
    if not os.path.exists(path):
        return None
    with open(path) as f:
        line = f.readline().strip()
    if not line:
        return None
    parts = line.split()
    if len(parts) < 5:
        return None
    cx, cy, nw, nh = map(float, parts[1:5])
    x1 = int((cx - nw / 2) * img_w)
    y1 = int((cy - nh / 2) * img_h)
    x2 = int((cx + nw / 2) * img_w)
    y2 = int((cy + nh / 2) * img_h)
    return (x1, y1, x2, y2)


def save_label(path, bbox, img_w, img_h):
    x1, y1, x2, y2 = bbox
    cx = (x1 + x2) / 2 / img_w
    cy = (y1 + y2) / 2 / img_h
    nw = abs(x2 - x1) / img_w
    nh = abs(y2 - y1) / img_h
    with open(path, "w") as f:
        f.write(f"{CLASS_ID} {cx:.6f} {cy:.6f} {nw:.6f} {nh:.6f}\n")


HEADER_H = 50
FOOTER_H = 30
PADDING_RATIO = 0.03  # 수동 라벨링 시 박스 크기의 3% padding 추가


def draw_review_overlay(img, bbox, header, stats_text):
    """이미지 위/아래에 별도 영역을 추가해서 텍스트 표시 (이미지를 가리지 않음)"""
    h, w = img.shape[:2]
    show = img.copy()

    if bbox:
        x1, y1, x2, y2 = bbox
        cv2.rectangle(show, (x1, y1), (x2, y2), (0, 255, 0), 2)

    # 검은색 헤더/푸터 영역을 이미지 위·아래에 덧붙임 (원본 이미지 영역 그대로 보존)
    final = np.zeros((h + HEADER_H + FOOTER_H, w, 3), dtype=np.uint8)
    final[HEADER_H : HEADER_H + h, :, :] = show

    # 상단 텍스트
    cv2.putText(final, header, (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.putText(final, stats_text, (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 255, 200), 1)

    # 하단 텍스트 (이미지 아래 영역)
    help_text = "y / Space = OK   n = re-label   d = delete   b = back   ESC = quit"
    cv2.putText(final, help_text, (10, HEADER_H + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)

    return final


def manual_label(img, fname):
    """수동 4점 클릭 라벨링 모드.
    클릭 좌표는 합성 이미지 기준이지만 헤더 오프셋 보정 필요."""
    points = []  # 원본 이미지 기준 좌표
    h, w = img.shape[:2]

    def on_mouse(event, x, y, flags, param):
        # y는 합성 이미지 좌표 → 헤더 빼서 원본 이미지 좌표로 변환
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 4:
            img_y = y - HEADER_H
            if 0 <= img_y < h:  # 이미지 영역 안의 클릭만 받음
                points.append((x, img_y))

    cv2.setMouseCallback(WIN, on_mouse)

    while True:
        show = img.copy()

        # 점 표시 (원본 이미지 기준)
        for i, p in enumerate(points):
            cv2.circle(show, p, 5, (0, 0, 255), -1)
            cv2.putText(show, str(i + 1), (p[0] + 8, p[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 4점 다 찍히면 박스 미리보기
        if len(points) == 4:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
            cv2.rectangle(show, (x1, y1), (x2, y2), (0, 255, 0), 2)

        # 헤더/푸터 덧붙임
        final = np.zeros((h + HEADER_H + FOOTER_H, w, 3), dtype=np.uint8)
        final[HEADER_H : HEADER_H + h, :, :] = show

        cv2.putText(final, f"Manual labeling: {fname}", (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        cv2.putText(final, f"Click 4 points (top/bottom/left/right). {len(points)}/4", (10, 42), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 1)

        help_text = "u = undo   r = reset   y/Space = save   ESC = cancel"
        if len(points) < 4:
            help_text = f"Click {4 - len(points)} more point(s).  " + help_text
        cv2.putText(final, help_text, (10, HEADER_H + h + 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 255), 1)

        cv2.imshow(WIN, final)
        key = cv2.waitKey(20) & 0xFF
        if key == 27:  # ESC
            return None
        elif key == ord("u") and points:
            points.pop()
        elif key == ord("r"):
            points = []
        elif key in (ord(" "), ord("y")) and len(points) == 4:
            xs = [p[0] for p in points]
            ys = [p[1] for p in points]
            x1, y1, x2, y2 = min(xs), min(ys), max(xs), max(ys)
            # 박스 크기의 PADDING_RATIO 만큼 padding (이미지 경계 안에서)
            bw, bh = x2 - x1, y2 - y1
            pad_x = int(bw * PADDING_RATIO)
            pad_y = int(bh * PADDING_RATIO)
            x1 = max(0, x1 - pad_x)
            y1 = max(0, y1 - pad_y)
            x2 = min(w, x2 + pad_x)
            y2 = min(h, y2 + pad_y)
            return (x1, y1, x2, y2)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--all", action="store_true", help="image-dir의 모든 사진 검수 (기본: 자동 라벨링한 것만)")
    p.add_argument("--image-dir", type=Path, default=DEFAULT_IMAGE_DIR)
    p.add_argument("--label-dir", type=Path, default=DEFAULT_LABEL_DIR)
    p.add_argument("--auto-log", type=Path, default=DEFAULT_AUTO_LOG)
    args = p.parse_args()
    image_dir = args.image_dir.resolve()
    label_dir = args.label_dir.resolve()
    auto_log = args.auto_log.resolve()
    if not image_dir.is_dir():
        print(f"image-dir가 없습니다: {image_dir}")
        return
    label_dir.mkdir(parents=True, exist_ok=True)

    # 검수 대상 결정
    all_images = sorted([f for f in os.listdir(image_dir) if f.lower().endswith(".jpg")])
    if not all_images:
        print("image-dir에 이미지가 없습니다.")
        return

    if args.all:
        images = all_images
        print(f"전체 검수 모드: {len(images)}장")
    else:
        if not auto_log.exists():
            print(f"자동 라벨링 기록이 없습니다: {auto_log}")
            print("먼저 auto_label.py를 실행하세요.")
            print("아니면 --all 옵션으로 전체를 검수하세요.")
            return
        with auto_log.open(encoding="utf-8") as f:
            target = set(line.strip() for line in f if line.strip())
        images = [f for f in all_images if f in target]
        if not images:
            print("자동 라벨링 대상이 없습니다.")
            return
        print(f"자동 라벨링한 {len(images)}장만 검수합니다.")
        print(f"(전체 {len(all_images)}장 중. --all 옵션으로 전체 검수 가능)")

    # WINDOW_NORMAL + WINDOW_KEEPRATIO: 비율 유지하면서 사용자가 창 크기 조절 가능
    cv2.namedWindow(WIN, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO | cv2.WINDOW_GUI_EXPANDED)

    stats = {"ok": 0, "fixed": 0, "deleted": 0, "skipped": 0}
    idx = 0

    while 0 <= idx < len(images):
        fname = images[idx]
        img_path = image_dir / fname
        label_path = label_dir / (Path(fname).stem + ".txt")

        img = cv2.imread(str(img_path))
        if img is None:
            print(f"읽기 실패: {fname}")
            idx += 1
            continue
        h, w = img.shape[:2]

        bbox = load_label(label_path, w, h)
        header = f"[{idx+1}/{len(images)}] {fname}  " + ("LABELED" if bbox else "NO LABEL")
        stats_text = f"OK:{stats['ok']}  Fixed:{stats['fixed']}  Deleted:{stats['deleted']}"
        show = draw_review_overlay(img, bbox, header, stats_text)
        cv2.imshow(WIN, show)
        key = cv2.waitKey(0) & 0xFF

        if key == 27:  # ESC
            print("종료")
            break
        elif key in (ord("y"), ord(" ")):
            stats["ok"] += 1
            idx += 1
        elif key == ord("n"):
            # 수동 재라벨링
            new_bbox = manual_label(img, fname)
            if new_bbox is not None:
                save_label(label_path, new_bbox, w, h)
                stats["fixed"] += 1
                print(f"  재라벨: {fname}")
                idx += 1
            # ESC면 그 자리 머무름
        elif key == ord("d"):
            # 라벨 삭제 (헬리패드 없음)
            if os.path.exists(label_path):
                os.remove(label_path)
            stats["deleted"] += 1
            print(f"  삭제: {fname}")
            idx += 1
        elif key == ord("b"):
            idx = max(0, idx - 1)
        else:
            stats["skipped"] += 1
            idx += 1

    cv2.destroyAllWindows()
    print()
    print("=== 검수 결과 ===")
    print(f"  OK (그대로):     {stats['ok']}")
    print(f"  재라벨링:        {stats['fixed']}")
    print(f"  삭제:            {stats['deleted']}")
    print(f"  스킵/기타:       {stats['skipped']}")


if __name__ == "__main__":
    main()
