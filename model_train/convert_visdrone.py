"""
VisDrone -> YOLO converter
--------------------------
- Filters to person classes (1=pedestrian, 2=person)
- Image-level gate: average box must exceed ratio thresholds
- Official splits used: train -> train, val -> val, test-dev -> test
- No sequence splitting needed (VisDrone splits are already sequence-separated)
"""
from pathlib import Path
import shutil

TRAIN_ROOT = Path(r"C:\Users\batuh\Desktop\datasets\VisDrone2019-DET-train")
VAL_ROOT   = Path(r"C:\Users\batuh\Desktop\datasets\VisDrone2019-DET-val")
TEST_ROOT  = Path(r"C:\Users\batuh\Desktop\datasets\VisDrone2019-DET-test-dev")

OUTPUT_ROOT = Path(r"C:\Users\batuh\Desktop\combined_dataset")

PERSON_CLASSES = {1, 2}

MIN_KEEP_WIDTH_RATIO  = 0.0
MIN_KEEP_HEIGHT_RATIO = 0.08

DATASET_PREFIX = "visdrone"


def cleanup(output_root: Path) -> None:
    removed = 0
    for kind in ["images", "labels"]:
        for split in ["train", "val", "test"]:
            split_dir = output_root / kind / split
            if not split_dir.exists():
                continue
            for p in split_dir.iterdir():
                if p.is_file() and p.name.startswith(f"{DATASET_PREFIX}_"):
                    p.unlink()
                    removed += 1
    print(f"[visdrone] Removed {removed} existing files")


def ensure_dirs(output_root: Path) -> None:
    for kind in ["images", "labels"]:
        for split in ["train", "val", "test"]:
            (output_root / kind / split).mkdir(parents=True, exist_ok=True)


def get_image_size(image_path: Path):
    from PIL import Image
    with Image.open(image_path) as im:
        return im.size  # (width, height)


def load_person_boxes(annotation_path: Path):
    boxes = []
    with annotation_path.open("r", encoding="utf-8") as f:
        for line_idx, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            parts = [x.strip() for x in line.split(",") if x.strip()]
            if len(parts) != 8:
                continue
            try:
                left, top, width, height, score, category, truncation, occlusion = map(int, parts)
            except ValueError:
                continue
            if score != 1:
                continue
            if category not in PERSON_CLASSES:
                continue
            if width <= 0 or height <= 0:
                continue
            boxes.append((left, top, width, height))
    return boxes


def qualifies_image(boxes, img_w: int, img_h: int) -> bool:
    if not boxes:
        return False
    avg_w = sum(w for _, _, w, h in boxes) / len(boxes)
    avg_h = sum(h for _, _, w, h in boxes) / len(boxes)
    return (avg_w / img_w) > MIN_KEEP_WIDTH_RATIO and (avg_h / img_h) > MIN_KEEP_HEIGHT_RATIO


def xywh_to_yolo(left, top, width, height, img_w, img_h):
    x_center = min(max((left + width  / 2.0) / img_w, 0.0), 1.0)
    y_center = min(max((top  + height / 2.0) / img_h, 0.0), 1.0)
    w_norm   = min(max(width  / img_w, 0.0), 1.0)
    h_norm   = min(max(height / img_h, 0.0), 1.0)
    return x_center, y_center, w_norm, h_norm


def convert_split(split_root: Path, split_name: str, output_root: Path):
    images_dir      = split_root / "images"
    annotations_dir = split_root / "annotations"

    if not images_dir.exists():
        raise FileNotFoundError(f"Missing: {images_dir}")
    if not annotations_dir.exists():
        raise FileNotFoundError(f"Missing: {annotations_dir}")

    out_img_dir = output_root / "images" / split_name
    out_lbl_dir = output_root / "labels" / split_name

    annotation_files = sorted(annotations_dir.glob("*.txt"))
    kept_images = kept_boxes = skipped_no_person = skipped_too_small = 0

    for ann_path in annotation_files:
        img_path = images_dir / f"{ann_path.stem}.jpg"
        if not img_path.exists():
            continue

        boxes = load_person_boxes(ann_path)
        if not boxes:
            skipped_no_person += 1
            continue

        img_w, img_h = get_image_size(img_path)
        if not qualifies_image(boxes, img_w, img_h):
            skipped_too_small += 1
            continue

        new_stem = f"{DATASET_PREFIX}_{split_name}_{img_path.stem}"
        shutil.copy2(img_path, out_img_dir / f"{new_stem}.jpg")

        with (out_lbl_dir / f"{new_stem}.txt").open("w", encoding="utf-8") as f:
            for left, top, width, height in boxes:
                x, y, w, h = xywh_to_yolo(left, top, width, height, img_w, img_h)
                f.write(f"0 {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")
                kept_boxes += 1

        kept_images += 1

    print(f"[visdrone/{split_name}] kept {kept_images} images, {kept_boxes} boxes "
          f"(skipped: {skipped_no_person} no-person, {skipped_too_small} too-small)")


if __name__ == "__main__":
    ensure_dirs(OUTPUT_ROOT)
    cleanup(OUTPUT_ROOT)
    convert_split(TRAIN_ROOT, "train", OUTPUT_ROOT)
    convert_split(VAL_ROOT,   "val",   OUTPUT_ROOT)
    convert_split(TEST_ROOT,  "test",  OUTPUT_ROOT)
    print("[visdrone] Done.")