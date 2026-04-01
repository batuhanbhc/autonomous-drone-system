"""
CrowdHuman -> YOLO converter
-----------------------------
CrowdHuman annotation format: .odgt (JSONL — one JSON object per line)

Each line:
{
  "ID": "273271,c9db000d5146c15",
  "gtboxes": [
    {
      "fbox": [x, y, w, h],   <- full body box  (USE THIS)
      "vbox": [x, y, w, h],   <- visible body box
      "hbox": [x, y, w, h],   <- head box
      "tag": "person",
      "extra": {"ignore": 0}
    },
    ...
  ]
}

We use fbox (full body) and skip boxes tagged "ignore" or with tag != "person".

Download:
  Images : https://www.crowdhuman.org/download.html
           CrowdHuman_train01.zip ... CrowdHuman_train03.zip
           CrowdHuman_val.zip
  Annotations:
           annotation_train.odgt
           annotation_val.odgt

Expected folder layout:
  BASE_PATH/
    Images/          <- unzipped train + val images all in one folder (jpg files)
    annotation_train.odgt
    annotation_val.odgt

Output goes to OUTPUT_ROOT (combined dataset).
MAX_TRAIN_IMAGES sampled from train, MAX_VAL_IMAGES sampled from val — same
~30% box ratio target applied to both splits.
"""
from __future__ import annotations

import json
import random
import shutil
from pathlib import Path

from PIL import Image


BASE_PATH   = Path(r"C:\Users\batuh\Desktop\datasets\CrowdHuman")
OUTPUT_ROOT = Path(r"C:\Users\batuh\Desktop\combined_dataset")

DATASET_PREFIX   = "crowdhuman"
MAX_TRAIN_IMAGES = 1000 
MAX_VAL_IMAGES   = 200 
SEED             = 42


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def ensure_dirs(output_root: Path) -> None:
    for kind in ["images", "labels"]:
        for split in ["train", "val"]:
            (output_root / kind / split).mkdir(parents=True, exist_ok=True)


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
    print(f"[crowdhuman] Removed {removed} existing files")


# ---------------------------------------------------------------------------
# Annotation parsing
# ---------------------------------------------------------------------------

def load_odgt(odgt_path: Path) -> list[dict]:
    """Load all records from a .odgt file. Returns list of dicts."""
    records = []
    with odgt_path.open("r", encoding="utf-8") as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            records.append(json.loads(line))
    return records


def extract_fboxes(record: dict) -> list[tuple[int, int, int, int]]:
    """
    Return list of (x, y, w, h) full-body boxes for persons.
    Skips: non-person tags, ignored boxes, zero/negative size boxes.
    """
    boxes = []
    for gt in record.get("gtboxes", []):
        if gt.get("tag") != "person":
            continue
        extra = gt.get("extra", {})
        if extra.get("ignore", 0) == 1:
            continue
        fbox = gt.get("fbox")
        if fbox is None:
            continue
        x, y, w, h = fbox
        if w <= 0 or h <= 0:
            continue
        boxes.append((int(x), int(y), int(w), int(h)))
    return boxes


# ---------------------------------------------------------------------------
# Coordinate conversion
# ---------------------------------------------------------------------------

def xywh_to_yolo(x, y, w, h, img_w, img_h):
    # Clamp box to image bounds first
    x2 = min(x + w, img_w)
    y2 = min(y + h, img_h)
    x  = max(x, 0)
    y  = max(y, 0)
    w  = x2 - x
    h  = y2 - y
    if w <= 0 or h <= 0:
        return None
    x_center = (x + w / 2.0) / img_w
    y_center = (y + h / 2.0) / img_h
    w_norm   = w / img_w
    h_norm   = h / img_h
    return (
        min(max(x_center, 0.0), 1.0),
        min(max(y_center, 0.0), 1.0),
        min(max(w_norm,   0.0), 1.0),
        min(max(h_norm,   0.0), 1.0),
    )


# ---------------------------------------------------------------------------
# Core conversion
# ---------------------------------------------------------------------------

def find_image(images_dir: Path, image_id: str) -> Path | None:
    """CrowdHuman image filenames are {ID}.jpg"""
    p = images_dir / f"{image_id}.jpg"
    return p if p.exists() else None


def convert_records(
    records: list[dict],
    images_dir: Path,
    split: str,
    output_root: Path,
    max_images: int | None = None,
) -> tuple[int, int]:
    """
    Convert a list of odgt records into YOLO format.
    Returns (images_written, boxes_written).
    """
    if max_images is not None and len(records) > max_images:
        rng = random.Random(SEED)
        records = rng.sample(records, max_images)
        print(f"[crowdhuman/{split}] Sampled {max_images} from {len(records) + max_images - max_images} records")

    out_img_dir = output_root / "images" / split
    out_lbl_dir = output_root / "labels" / split

    written_images = written_boxes = skipped = 0

    for record in records:
        image_id = record.get("ID", "")
        img_path = find_image(images_dir, image_id)
        if img_path is None:
            skipped += 1
            continue

        boxes = extract_fboxes(record)
        if not boxes:
            skipped += 1
            continue

        try:
            with Image.open(img_path) as im:
                img_w, img_h = im.size
        except Exception as e:
            print(f"  Could not open {img_path}: {e}")
            skipped += 1
            continue

        new_stem     = f"{DATASET_PREFIX}_{split}_{image_id}"
        out_img_path = out_img_dir / f"{new_stem}.jpg"
        out_lbl_path = out_lbl_dir / f"{new_stem}.txt"

        shutil.copy2(img_path, out_img_path)

        valid_count = 0
        with out_lbl_path.open("w", encoding="utf-8") as f:
            for x, y, w, h in boxes:
                yolo = xywh_to_yolo(x, y, w, h, img_w, img_h)
                if yolo is None:
                    continue
                xc, yc, wn, hn = yolo
                f.write(f"0 {xc:.6f} {yc:.6f} {wn:.6f} {hn:.6f}\n")
                valid_count += 1

        if valid_count == 0:
            out_img_path.unlink(missing_ok=True)
            out_lbl_path.unlink(missing_ok=True)
            skipped += 1
            continue

        written_images += 1
        written_boxes  += valid_count

    print(f"[crowdhuman/{split}] {written_images} images, {written_boxes} boxes "
          f"(avg {written_boxes/written_images:.1f}/img)"
          f"  |  skipped: {skipped}")
    return written_images, written_boxes


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def convert_crowdhuman(base_path: Path, output_root: Path) -> None:
    images_dir  = base_path / "Images"
    train_odgt  = base_path / "annotation_train.odgt"
    val_odgt    = base_path / "annotation_val.odgt"

    if not images_dir.exists():
        raise FileNotFoundError(f"Missing images folder: {images_dir}")
    if not train_odgt.exists():
        raise FileNotFoundError(f"Missing: {train_odgt}")
    if not val_odgt.exists():
        raise FileNotFoundError(f"Missing: {val_odgt}")

    print(f"[crowdhuman] Loading train annotations from {train_odgt.name} ...")
    train_records = load_odgt(train_odgt)
    print(f"[crowdhuman] {len(train_records)} train records loaded")

    print(f"[crowdhuman] Loading val annotations from {val_odgt.name} ...")
    val_records = load_odgt(val_odgt)
    print(f"[crowdhuman] {len(val_records)} val records loaded")

    convert_records(train_records, images_dir, "train", output_root, max_images=MAX_TRAIN_IMAGES)
    convert_records(val_records,   images_dir, "val",   output_root, max_images=MAX_VAL_IMAGES)


if __name__ == "__main__":
    ensure_dirs(OUTPUT_ROOT)
    cleanup(OUTPUT_ROOT)
    convert_crowdhuman(BASE_PATH, OUTPUT_ROOT)
    print("[crowdhuman] Done.")