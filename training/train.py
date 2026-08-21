"""
Train SSDLite + MobileNetV3-Small helipad detector (드론 자율착륙용).
입력 512 + 작은 객체용 custom anchor (멀리서 작게 포착 대응). detection은
정확도 우선으로 설계했고, 배포 런타임의 LK optical-flow tracking과 역할을 분리했다.

기본 데이터/출력 경로는 저장소 내부의 gitignored `data/`, `weights/`이며 CLI로 변경할 수 있다.
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
import shutil
import datetime
from pathlib import Path
import torch
from torchvision.models.detection.ssd import SSD
from torchvision.models.detection.anchor_utils import DefaultBoxGenerator
from torchvision.models.detection.ssdlite import (
    SSDLiteHead, SSDLiteFeatureExtractorMobileNet,
)
from torchvision.models import mobilenet_v3_small, MobileNet_V3_Small_Weights
from torch.utils.data import Dataset, DataLoader
import cv2
import numpy as np
import torch.nn as nn
import random
import albumentations as A
from tqdm import tqdm

# ─── Config ───────────────────────────────────────────────────────────────────
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_DATA_DIR = REPO_ROOT / "data"
DEFAULT_SAVE_DIR = REPO_ROOT / "weights"
NUM_CLASSES = 2
BATCH_SIZE = 16
NUM_EPOCHS = 80
LR = 0.008
MOMENTUM = 0.9
WEIGHT_DECAY = 0.0005
INPUT_SIZE = 512
DEVICE = torch.device("cuda" if torch.cuda.is_available() else "cpu")


# ─── Augmentation pipeline (albumentations) ───────────────────────────────────
# 학습용 (회전 + 좌우반전 + 밝기/대비 + 리사이즈)
TRAIN_TRANSFORM = A.Compose([
    A.HorizontalFlip(p=0.5),
    A.RandomBrightnessContrast(brightness_limit=0.25, contrast_limit=0.25, p=0.5),
    A.RandomRotate90(p=0.3),                      # 90/180/270도 무작위 회전
    A.Rotate(limit=20, p=0.4,                     # 임의 각도 -20~+20도 회전
             border_mode=cv2.BORDER_CONSTANT, fill=0),
    A.Resize(INPUT_SIZE, INPUT_SIZE),
], bbox_params=A.BboxParams(format='pascal_voc', label_fields=['labels'], min_visibility=0.3))

# 검증용 (리사이즈만)
VAL_TRANSFORM = A.Compose([
    A.Resize(INPUT_SIZE, INPUT_SIZE),
], bbox_params=A.BboxParams(format='pascal_voc', label_fields=['labels'], min_visibility=0.3))


# ─── Dataset with albumentations augmentation ────────────────────────────────
class HelipadDataset(Dataset):
    def __init__(self, img_dir, lbl_dir, augment=False):
        self.img_dir = img_dir
        self.lbl_dir = lbl_dir
        self.augment = augment
        self.transform = TRAIN_TRANSFORM if augment else VAL_TRANSFORM
        self.imgs = sorted([f for f in os.listdir(img_dir) if f.endswith(".jpg")])

    def __len__(self):
        return len(self.imgs)

    def __getitem__(self, idx):
        fname = self.imgs[idx]
        img_path = os.path.join(self.img_dir, fname)
        lbl_path = os.path.join(self.lbl_dir, fname.replace(".jpg", ".txt"))

        img = cv2.imread(img_path)
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w = img.shape[:2]

        boxes, labels = [], []
        if os.path.exists(lbl_path):
            with open(lbl_path) as f:
                for line in f:
                    parts = line.strip().split()
                    if len(parts) < 5: continue
                    cx, cy, bw, bh = map(float, parts[1:5])
                    x1, y1 = (cx - bw/2) * w, (cy - bh/2) * h
                    x2, y2 = (cx + bw/2) * w, (cy + bh/2) * h
                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(w, x2), min(h, y2)
                    if x2 > x1 and y2 > y1:
                        boxes.append([x1, y1, x2, y2])
                        labels.append(1)

        # 라벨 없으면 더미
        if len(boxes) == 0:
            boxes = [[0, 0, 1, 1]]
            labels = [0]

        # Albumentations 변환 (이미지 + bbox 동시에)
        try:
            result = self.transform(image=img, bboxes=boxes, labels=labels)
            img = result['image']
            boxes = result['bboxes']
            labels = result['labels']
        except Exception as e:
            # 회전 후 박스가 너무 잘리는 등 변환 실패 시 검증 transform 사용
            result = VAL_TRANSFORM(image=img, bboxes=boxes, labels=labels)
            img = result['image']
            boxes = result['bboxes']
            labels = result['labels']

        # 회전으로 박스가 사라진 경우 더미 처리
        if len(boxes) == 0:
            boxes = [[0, 0, 1, 1]]
            labels = [0]

        boxes = torch.as_tensor(boxes, dtype=torch.float32)
        labels = torch.as_tensor(labels, dtype=torch.int64)
        area = (boxes[:, 3] - boxes[:, 1]) * (boxes[:, 2] - boxes[:, 0])
        iscrowd = torch.zeros(len(labels), dtype=torch.int64)

        # numpy HWC -> torch CHW
        img = torch.from_numpy(img).permute(2, 0, 1).float() / 255.0

        target = {
            "boxes": boxes, "labels": labels,
            "area": area, "iscrowd": iscrowd,
            "image_id": torch.tensor([idx]),
        }

        return img, target


def collate_fn(batch):
    return tuple(zip(*batch))


# ─── Model ────────────────────────────────────────────────────────────────────
def get_model(num_classes, input_size=INPUT_SIZE):
    backbone_model = mobilenet_v3_small(weights=MobileNet_V3_Small_Weights.IMAGENET1K_V1)
    backbone = SSDLiteFeatureExtractorMobileNet(
        backbone=backbone_model.features, c4_pos=7, norm_layer=nn.BatchNorm2d,
    )
    # 작은 객체가 여러 feature-map level에 걸리도록 명시적인 scale을 사용한다.
    # min_ratio/max_ratio 선형배치는 작은 객체가 첫 feature map 하나에만 몰려 부족.
    # 명시 scale 로 512 기준 L0~L1(20~58px)에 작은 헬리패드를 여러 level 이 받게 함.
    # helipad 는 원형/정사각이라 aspect ratio 는 [2] 만 (ratio 3 제거 → anchor 수↓,
    # 혼동 anchor 감소로 confidence separation 유리).
    anchor_generator = DefaultBoxGenerator(
        [[2], [2], [2], [2], [2], [2]],
        scales=[0.040, 0.080, 0.160, 0.260, 0.400, 0.600, 0.850],
        clip=True,
    )
    backbone.eval()
    dummy = torch.randn(1, 3, input_size, input_size)
    with torch.no_grad():
        features = backbone(dummy)
    in_channels = [f.shape[1] for f in features.values()]
    fm_shapes = [(f.shape[2], f.shape[3]) for f in features.values()]
    print(f"Feature maps @ {input_size}x{input_size}: {fm_shapes}, channels: {in_channels}")
    num_anchors = anchor_generator.num_anchors_per_location()
    backbone.train()

    head = SSDLiteHead(
        in_channels=in_channels, num_anchors=num_anchors,
        num_classes=num_classes, norm_layer=nn.BatchNorm2d,
    )
    model = SSD(
        backbone=backbone, anchor_generator=anchor_generator,
        size=(input_size, input_size), num_classes=num_classes, head=head,
        score_thresh=0.3, nms_thresh=0.45, detections_per_img=10,
        # 작은 박스의 positive anchor를 확보하기 위해 IoU와 positive 비율을 조정했다.
        iou_thresh=0.4, positive_fraction=0.33,
        image_mean=[0.485, 0.456, 0.406],
        image_std=[0.229, 0.224, 0.225],
    )
    return model


def auto_backup_existing(save_dir):
    """학습 시작 시 기존 best.pth 가 있으면 backups/ 폴더로 timestamp 백업."""
    save_dir = Path(save_dir)
    src = save_dir / "best.pth"
    if not src.exists():
        return None
    backup_dir = save_dir / "backups"
    backup_dir.mkdir(parents=True, exist_ok=True)
    ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    dst = backup_dir / f"best_{ts}.pth"
    shutil.copy2(src, dst)
    return dst


def log_version(
    save_dir, train_n, val_n, best_loss, elapsed_min, backup_path,
    augment_summary, num_epochs, input_size,
):
    """학습 종료 후 weights/version_log.txt 에 자동 기록 (append)."""
    log_path = Path(save_dir) / "version_log.txt"
    ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
    device_name = (torch.cuda.get_device_name(0)
                   if torch.cuda.is_available() else "CPU")
    backup_name = os.path.basename(backup_path) if backup_path else "(이전 모델 없음, 첫 학습)"

    entry = (
        f"\n{'=' * 60}\n"
        f"[{ts}]\n"
        f"  데이터:        훈련 {train_n}장 / 검증 {val_n}장 (총 {train_n + val_n}장)\n"
        f"  Best val_loss: {best_loss:.4f}\n"
        f"  학습 시간:     {elapsed_min:.1f}분\n"
        f"  Epochs:        {num_epochs}\n"
        f"  Input size:    {input_size}x{input_size}\n"
        f"  Device:        {device_name}\n"
        f"  Augmentation:  {augment_summary}\n"
        f"  이전 모델 백업: {backup_name}\n"
    )
    with open(log_path, "a", encoding="utf-8") as f:
        f.write(entry)
    return log_path


def print_system_info(device):
    """학습 시작 전 시스템 환경 정보 출력."""
    print("=" * 60)
    print("  학습 환경 정보")
    print("=" * 60)
    print(f"  PyTorch:          {torch.__version__}")
    print(f"  Albumentations:   {A.__version__}")
    print(f"  CUDA 사용 가능:   {torch.cuda.is_available()}")
    if device.type == "cuda":
        print(f"  CUDA 버전:        {torch.version.cuda}")
        print(f"  GPU 장치:         {torch.cuda.get_device_name(0)}")
        vram_total = torch.cuda.get_device_properties(0).total_memory / 1024**3
        print(f"  GPU VRAM:         {vram_total:.1f} GB")
        print(f"  사용 디바이스:    {device} (GPU 가속)")
    else:
        import platform
        print(f"  사용 디바이스:    {device} ({platform.processor()})")
        print(f"  주의: GPU 없이 학습 시 시간이 오래 걸립니다")
    print("=" * 60)


def print_train_config(train_n, val_n, save_dir, batch_size, num_epochs, learning_rate):
    print("\n  학습 설정")
    print("-" * 60)
    print(f"  데이터셋:         훈련 {train_n}장 / 검증 {val_n}장")
    print(f"  입력 해상도:      {INPUT_SIZE}x{INPUT_SIZE}")
    print(f"  배치 크기:        {batch_size}")
    print(f"  에포크:           {num_epochs}")
    print(f"  학습률:           {learning_rate} (Cosine Annealing)")
    print(f"  Optimizer:        SGD (momentum={MOMENTUM}, wd={WEIGHT_DECAY})")
    print(f"  Augmentation:     좌우반전 + 밝기/대비 + 회전 (90도 + 임의각)")
    print(f"  저장 경로:        {save_dir}")
    print("-" * 60)


def train(
    data_dir=DEFAULT_DATA_DIR,
    save_dir=DEFAULT_SAVE_DIR,
    *,
    batch_size=BATCH_SIZE,
    num_epochs=NUM_EPOCHS,
    learning_rate=LR,
    num_workers=2,
    device=DEVICE,
):
    data_dir = Path(data_dir).resolve()
    save_dir = Path(save_dir).resolve()
    save_dir.mkdir(parents=True, exist_ok=True)

    print_system_info(device)

    # 기존 best.pth 자동 백업
    backup_path = auto_backup_existing(save_dir)
    if backup_path:
        print(f"\n  이전 모델 자동 백업: {os.path.basename(backup_path)}")
        print(f"  (위치: {os.path.dirname(backup_path)})")
    else:
        print(f"\n  (이전 best.pth 없음 — 첫 학습)")

    train_ds = HelipadDataset(
        data_dir / "train" / "images",
        data_dir / "train" / "labels",
        augment=True,
    )
    val_ds = HelipadDataset(
        data_dir / "val" / "images",
        data_dir / "val" / "labels",
        augment=False,
    )
    train_loader = DataLoader(
        train_ds, batch_size=batch_size, shuffle=True,
        num_workers=num_workers, collate_fn=collate_fn, pin_memory=device.type == "cuda",
        drop_last=True,
    )
    val_loader = DataLoader(
        val_ds, batch_size=batch_size, shuffle=False,
        num_workers=num_workers, collate_fn=collate_fn, pin_memory=device.type == "cuda",
    )

    print_train_config(
        len(train_ds), len(val_ds), save_dir, batch_size, num_epochs, learning_rate,
    )

    print("\n  모델 빌드 중...")
    model = get_model(NUM_CLASSES)
    model.to(device)

    params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(
        params, lr=learning_rate, momentum=MOMENTUM, weight_decay=WEIGHT_DECAY,
    )
    # 3-epoch linear warmup 뒤 cosine schedule을 적용한다.
    WARMUP_EPOCHS = 3
    warmup = torch.optim.lr_scheduler.LinearLR(optimizer, start_factor=0.1, total_iters=WARMUP_EPOCHS)
    cosine = torch.optim.lr_scheduler.CosineAnnealingLR(optimizer, T_max=num_epochs - WARMUP_EPOCHS)
    lr_scheduler = torch.optim.lr_scheduler.SequentialLR(
        optimizer, schedulers=[warmup, cosine], milestones=[WARMUP_EPOCHS])

    print("\n  학습 시작\n" + "=" * 60)

    train_start = time.time()
    best_loss = float("inf")
    for epoch in range(num_epochs):
        model.train()
        epoch_loss = 0.0
        t0 = time.time()

        # 훈련 루프 (배치별 진행률 바)
        train_pbar = tqdm(train_loader,
                          desc=f"Epoch {epoch+1:>3}/{num_epochs} [Train]",
                          ncols=100, leave=False)
        for batch_idx, (images, targets) in enumerate(train_pbar):
            images = [img.to(device) for img in images]
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]
            loss_dict = model(images, targets)
            losses = sum(loss for loss in loss_dict.values())
            optimizer.zero_grad()
            losses.backward()
            optimizer.step()
            epoch_loss += losses.item()
            # 진행률 바에 현재 loss 표시
            train_pbar.set_postfix(loss=f"{losses.item():.3f}")
        lr_scheduler.step()
        avg_loss = epoch_loss / len(train_loader)
        elapsed = time.time() - t0

        # 검증 루프 (배치별 진행률 바)
        model.train()
        val_loss = 0.0
        val_pbar = tqdm(val_loader,
                        desc=f"Epoch {epoch+1:>3}/{num_epochs} [Val]  ",
                        ncols=100, leave=False)
        with torch.no_grad():
            for images, targets in val_pbar:
                images = [img.to(device) for img in images]
                targets = [{k: v.to(device) for k, v in t.items()} for t in targets]
                loss_dict = model(images, targets)
                batch_loss = sum(loss.item() for loss in loss_dict.values())
                val_loss += batch_loss
                val_pbar.set_postfix(loss=f"{batch_loss:.3f}")
        avg_val_loss = val_loss / len(val_loader)

        # 에포크 요약 (한 줄)
        marker = "★" if avg_val_loss < best_loss else " "
        print(f"{marker} Epoch [{epoch+1:>3}/{num_epochs}] "
              f"train={avg_loss:.4f}  val={avg_val_loss:.4f}  "
              f"lr={optimizer.param_groups[0]['lr']:.6f}  t={elapsed:.1f}s")

        if avg_val_loss < best_loss:
            best_loss = avg_val_loss
            torch.save(model.state_dict(), save_dir / "best.pth")
            print(f"    → best.pth 저장 (val_loss={best_loss:.4f})")

    torch.save(model.state_dict(), save_dir / "final.pth")
    elapsed_total_min = (time.time() - train_start) / 60.0

    # 버전 로그 자동 기록
    aug_summary = "flip + brightness + 90deg rot + random rot(-20~+20)"
    log_path = log_version(
        save_dir, len(train_ds), len(val_ds), best_loss,
        elapsed_total_min, backup_path, aug_summary, num_epochs, INPUT_SIZE,
    )

    print("\n" + "=" * 60)
    print(f"  학습 완료. Best val_loss = {best_loss:.4f}")
    print(f"  학습 시간:  {elapsed_total_min:.1f}분")
    print(f"  최종 모델:  {save_dir / 'best.pth'}")
    if backup_path:
        print(f"  백업 파일:  {os.path.basename(backup_path)}")
    print(f"  버전 로그:  {log_path}")
    print("=" * 60)
    print("\n  다음 단계: 모델을 NCNN 배포 형식으로 변환한 뒤 vision/ 런타임에서 검증하세요.")


def parse_args():
    parser = argparse.ArgumentParser(description="SSDLite 512 helipad detector 학습")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_SAVE_DIR)
    parser.add_argument("--batch-size", type=int, default=BATCH_SIZE)
    parser.add_argument("--epochs", type=int, default=NUM_EPOCHS)
    parser.add_argument("--learning-rate", type=float, default=LR)
    parser.add_argument("--num-workers", type=int, default=2)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    if args.epochs <= 3:
        parser.error("--epochs must be greater than the 3-epoch warmup")
    return args


def main():
    args = parse_args()
    random.seed(args.seed)
    np.random.seed(args.seed)
    torch.manual_seed(args.seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(args.seed)
    train(
        args.data_dir,
        args.output_dir,
        batch_size=args.batch_size,
        num_epochs=args.epochs,
        learning_rate=args.learning_rate,
        num_workers=args.num_workers,
        device=torch.device(args.device),
    )


if __name__ == "__main__":
    main()
