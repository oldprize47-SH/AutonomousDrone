"""SSDLite 512 헬리패드 검출기의 로컬 검증 지표와 그래프를 생성한다.

검증 데이터와 모델 가중치는 저장소에 포함하지 않는다. 이 스크립트는 YOLO 형식의
단일 클래스 라벨을 읽어 confidence sweep, VOC2010+ 보간 AP, mAP@0.5:0.95를 계산한다.
사람이 포함될 수 있는 원본 이미지 montage는 의도적으로 생성하지 않는다.
"""

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import matplotlib.pyplot as plt
import numpy as np
import torch


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT))

from training.train import INPUT_SIZE, NUM_CLASSES, get_model  # noqa: E402


DEFAULT_DATA_DIR = REPO_ROOT / "data"
DEFAULT_WEIGHTS = REPO_ROOT / "weights" / "best.pth"
DEFAULT_OUTPUT_DIR = REPO_ROOT / "output" / "evaluation"
IOU_THRESHOLDS = (0.3, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85, 0.9, 0.95)


def load_gt(label_path, width, height):
    boxes = []
    if label_path.exists():
        with label_path.open(encoding="utf-8") as handle:
            for line in handle:
                parts = line.strip().split()
                if len(parts) < 5:
                    continue
                cx, cy, box_width, box_height = map(float, parts[1:5])
                boxes.append([
                    (cx - box_width / 2) * width,
                    (cy - box_height / 2) * height,
                    (cx + box_width / 2) * width,
                    (cy + box_height / 2) * height,
                ])
    return boxes


def box_iou(a, b):
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    intersection = max(0.0, ix2 - ix1) * max(0.0, iy2 - iy1)
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    return intersection / max(area_a + area_b - intersection, 1e-9)


def voc_ap(recall, precision):
    """VOC2010+ continuous/interpolated average precision."""
    mrec = np.concatenate(([0.0], recall, [1.0]))
    mpre = np.concatenate(([0.0], precision, [0.0]))
    for index in range(mpre.size - 1, 0, -1):
        mpre[index - 1] = max(mpre[index - 1], mpre[index])
    changed = np.where(mrec[1:] != mrec[:-1])[0]
    return float(np.sum((mrec[changed + 1] - mrec[changed]) * mpre[changed + 1]))


def eval_ap(records, iou_threshold):
    total_gt = sum(len(record["gt"]) for record in records)
    detections = []
    for image_index, record in enumerate(records):
        for score, box in record["dets"]:
            detections.append((float(score), image_index, box))
    detections.sort(key=lambda item: item[0], reverse=True)

    matched = [set() for _ in records]
    true_positives = []
    false_positives = []
    for _score, image_index, box in detections:
        best_iou, best_gt_index = 0.0, -1
        for gt_index, gt in enumerate(records[image_index]["gt"]):
            if gt_index in matched[image_index]:
                continue
            iou = box_iou(box, gt)
            if iou > best_iou:
                best_iou, best_gt_index = iou, gt_index
        if best_gt_index >= 0 and best_iou > iou_threshold:
            true_positives.append(1.0)
            false_positives.append(0.0)
            matched[image_index].add(best_gt_index)
        else:
            true_positives.append(0.0)
            false_positives.append(1.0)

    if not detections:
        return {"ap": 0.0, "recall": [], "precision": [], "scores": []}

    tp_cumulative = np.cumsum(true_positives)
    fp_cumulative = np.cumsum(false_positives)
    recall = tp_cumulative / max(total_gt, 1)
    precision = tp_cumulative / np.maximum(tp_cumulative + fp_cumulative, 1e-9)
    return {
        "ap": voc_ap(recall, precision),
        "recall": recall.tolist(),
        "precision": precision.tolist(),
        "scores": [detection[0] for detection in detections],
    }


def eval_conf(records, confidence, iou_threshold):
    total_gt = sum(len(record["gt"]) for record in records)
    detection_count = true_positive_count = false_positive_count = 0
    matched_total = 0
    matched_ious = []

    for record in records:
        matched = set()
        detections = [item for item in record["dets"] if item[0] >= confidence]
        detection_count += len(detections)
        for _score, box in sorted(detections, key=lambda item: item[0], reverse=True):
            best_iou, best_gt_index = 0.0, -1
            for gt_index, gt in enumerate(record["gt"]):
                if gt_index in matched:
                    continue
                iou = box_iou(box, gt)
                if iou > best_iou:
                    best_iou, best_gt_index = iou, gt_index
            if best_gt_index >= 0 and best_iou > iou_threshold:
                true_positive_count += 1
                matched.add(best_gt_index)
                matched_ious.append(best_iou)
            else:
                false_positive_count += 1
        matched_total += len(matched)

    false_negative_count = total_gt - matched_total
    precision = true_positive_count / max(detection_count, 1)
    recall = true_positive_count / max(total_gt, 1)
    f1 = 2 * precision * recall / max(precision + recall, 1e-9)
    return {
        "conf": confidence,
        "iou_thr": iou_threshold,
        "gt": total_gt,
        "det": detection_count,
        "tp": true_positive_count,
        "fp": false_positive_count,
        "fn": false_negative_count,
        "precision": precision,
        "recall": recall,
        "f1": f1,
        "mean_iou": float(np.mean(matched_ious)) if matched_ious else 0.0,
    }


def run_inference(image_dir, label_dir, weights, device):
    model = get_model(NUM_CLASSES)
    state = torch.load(str(weights), map_location=device, weights_only=True)
    model.load_state_dict(state)
    model.to(device).eval()
    # AP ranking용으로만 낮은 score도 보존한다. 실시간 게이트 임계값과는 별개다.
    if hasattr(model, "score_thresh"):
        model.score_thresh = 0.001
    if hasattr(model, "detections_per_img"):
        model.detections_per_img = 100

    records = []
    image_files = sorted(path for path in image_dir.iterdir() if path.suffix.lower() == ".jpg")
    for image_path in image_files:
        image = cv2.imread(str(image_path))
        if image is None:
            continue
        height, width = image.shape[:2]
        gt = load_gt(label_dir / f"{image_path.stem}.txt", width, height)
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        tensor = torch.from_numpy(rgb).permute(2, 0, 1).float() / 255.0
        tensor = torch.nn.functional.interpolate(
            tensor.unsqueeze(0),
            size=(INPUT_SIZE, INPUT_SIZE),
            mode="bilinear",
            align_corners=False,
        ).squeeze(0).to(device)
        with torch.no_grad():
            prediction = model([tensor])[0]

        boxes = prediction["boxes"].detach().cpu().numpy()
        scores = prediction["scores"].detach().cpu().numpy()
        labels = prediction["labels"].detach().cpu().numpy()
        detections = []
        for box, score, label in zip(boxes, scores, labels):
            if int(label) == 0:
                continue
            detections.append((float(score), [
                float(box[0] * width / INPUT_SIZE),
                float(box[1] * height / INPUT_SIZE),
                float(box[2] * width / INPUT_SIZE),
                float(box[3] * height / INPUT_SIZE),
            ]))
        records.append({"file": image_path.name, "gt": gt, "dets": detections})
    return records


def make_plots(output_dir, summary, ap_results, sweep):
    plt.rcParams["font.family"] = "DejaVu Sans"

    figure, axis = plt.subplots(figsize=(6.4, 4.6), dpi=180)
    for threshold, label in ((0.3, "AP@0.3"), (0.5, "AP@0.5")):
        result = ap_results[str(threshold)]
        axis.plot(result["recall"], result["precision"], linewidth=2,
                  label=f"{label} = {result['ap'] * 100:.2f}%")
    axis.set(xlim=(0, 1.02), ylim=(0, 1.02), xlabel="Recall", ylabel="Precision")
    axis.grid(True, alpha=0.3)
    axis.legend(loc="lower left")
    figure.tight_layout()
    figure.savefig(output_dir / "precision-recall.png", bbox_inches="tight")
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(6.4, 4.3), dpi=180)
    thresholds = [result["conf"] for result in sweep]
    axis.plot(thresholds, [result["precision"] * 100 for result in sweep], marker="o", label="Precision")
    axis.plot(thresholds, [result["recall"] * 100 for result in sweep], marker="s", label="Recall")
    axis.set(xlabel="Confidence threshold", ylabel="Score [%]", ylim=(0, 101.5))
    axis.grid(True, alpha=0.3)
    axis.legend()
    figure.tight_layout()
    figure.savefig(output_dir / "confidence-sweep.png", bbox_inches="tight")
    plt.close(figure)

    figure, axis = plt.subplots(figsize=(6.4, 4.1), dpi=180)
    names = ["Precision", "Recall", "F1", "AP@0.3", "AP@0.5", "mAP@0.5:0.95"]
    values = [
        summary["precision"] * 100,
        summary["recall"] * 100,
        summary["f1"] * 100,
        ap_results["0.3"]["ap"] * 100,
        ap_results["0.5"]["ap"] * 100,
        summary["map_50_95"] * 100,
    ]
    bars = axis.bar(names, values, color=["#4C78A8", "#72B7B2", "#59A14F", "#F28E2B", "#E15759", "#B07AA1"])
    axis.set(ylim=(0, 105), ylabel="Score [%]")
    axis.tick_params(axis="x", rotation=20)
    axis.grid(axis="y", alpha=0.25)
    for bar, value in zip(bars, values):
        axis.text(bar.get_x() + bar.get_width() / 2, value + 1, f"{value:.1f}", ha="center", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_dir / "metric-summary.png", bbox_inches="tight")
    plt.close(figure)


def make_training_history_plot(output_dir, csv_path):
    rows = []
    with csv_path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            rows.append((row["label"], float(row["best_val_loss"])))
    if not rows:
        return
    figure, axis = plt.subplots(figsize=(6.2, 3.8), dpi=180)
    axis.plot([row[0] for row in rows], [row[1] for row in rows], marker="o", linewidth=2)
    axis.set(ylabel="Best validation loss", title="Training summary")
    axis.grid(True, alpha=0.3)
    for index, (_label, value) in enumerate(rows):
        axis.text(index, value + 0.05, f"{value:.4f}", ha="center", fontsize=8)
    figure.tight_layout()
    figure.savefig(output_dir / "training-validation-loss.png", bbox_inches="tight")
    plt.close(figure)


def parse_args():
    parser = argparse.ArgumentParser(description="SSDLite helipad detector 로컬 평가")
    parser.add_argument("--data-dir", type=Path, default=DEFAULT_DATA_DIR)
    parser.add_argument("--weights", type=Path, default=DEFAULT_WEIGHTS)
    parser.add_argument("--output-dir", type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument("--device", default="cuda" if torch.cuda.is_available() else "cpu")
    parser.add_argument("--confidence", type=float, default=0.5)
    parser.add_argument("--match-iou", type=float, default=0.3)
    parser.add_argument("--training-history", type=Path,
                        help="선택 사항: label,best_val_loss 열을 가진 CSV")
    return parser.parse_args()


def main():
    args = parse_args()
    data_dir = args.data_dir.resolve()
    image_dir = data_dir / "val" / "images"
    label_dir = data_dir / "val" / "labels"
    weights = args.weights.resolve()
    output_dir = args.output_dir.resolve()
    if not image_dir.is_dir() or not label_dir.is_dir():
        raise SystemExit("검증 images/labels 폴더가 없습니다. --data-dir를 확인하세요.")
    if not weights.is_file():
        raise SystemExit("모델 가중치가 없습니다. --weights를 확인하세요.")
    output_dir.mkdir(parents=True, exist_ok=True)

    device = torch.device(args.device)
    records = run_inference(image_dir, label_dir, weights, device)
    if not records:
        raise SystemExit("읽을 수 있는 검증 JPG가 없습니다.")

    ap_results = {str(threshold): eval_ap(records, threshold) for threshold in IOU_THRESHOLDS}
    map_50_95 = float(np.mean([
        ap_results[str(threshold)]["ap"] for threshold in IOU_THRESHOLDS if threshold >= 0.5
    ]))
    summary_at_confidence = eval_conf(records, args.confidence, args.match_iou)
    sweep = [eval_conf(records, confidence, args.match_iou) for confidence in np.arange(0.3, 1.0, 0.1)]
    summary = {
        **summary_at_confidence,
        "image_count": len(records),
        "total_gt": sum(len(record["gt"]) for record in records),
        "ap_03": ap_results["0.3"]["ap"],
        "map_50": ap_results["0.5"]["ap"],
        "map_50_95": map_50_95,
        "weight_file": weights.name,
        "input_size": INPUT_SIZE,
        "device": str(device),
        "note": (
            "One-class local validation. The model postprocess threshold is lowered only "
            "to preserve detections for AP ranking; it is not the flight-runtime threshold."
        ),
    }

    with (output_dir / "eval-summary.json").open("w", encoding="utf-8") as handle:
        json.dump(
            {"summary": summary, "confidence_sweep": sweep,
             "ap": {key: {"ap": value["ap"]} for key, value in ap_results.items()}},
            handle,
            ensure_ascii=False,
            indent=2,
        )
    with (output_dir / "confidence-sweep.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(sweep[0].keys()))
        writer.writeheader()
        writer.writerows(sweep)
    with (output_dir / "eval-summary-table.csv").open("w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value"])
        for key in (
            "image_count", "total_gt", "det", "tp", "fp", "fn", "precision",
            "recall", "f1", "mean_iou", "ap_03", "map_50", "map_50_95",
        ):
            writer.writerow([key, summary[key]])

    make_plots(output_dir, summary, ap_results, sweep)
    if args.training_history:
        make_training_history_plot(output_dir, args.training_history.resolve())
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    print(f"output_dir={output_dir}")


if __name__ == "__main__":
    main()
