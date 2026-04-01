"""
SOMPT22 -> YOLO converter
--------------------------
- Converts train sequences, splitting 20% by sequence into val
- Converts test sequences into test split (NEW)
- keep_modulo controls frame subsampling (every Nth frame)
- Split is by sequence, never by frame, to avoid data leakage
"""
from __future__ import annotations

import csv
import random
import shutil
from collections import defaultdict
from pathlib import Path

from PIL import Image


BASE_PATH   = Path(r"C:\Users\batuh\Desktop\datasets\sompt22")
OUTPUT_ROOT = Path(r"C:\Users\batuh\Desktop\combined_dataset")

DATASET_PREFIX   = "sompt22"
KEEP_MODULO      = 30   
VAL_RATIO        = 0.2   # 20% of train sequences held out for val
SEED             = 42


# ---------------------------------------------------------------------------
# I/O helpers
# ---------------------------------------------------------------------------

def ensure_dirs(output_root: Path) -> None:
    for kind in ["images", "labels"]:
        for split in ["train", "val", "test"]:
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
    print(f"[sompt22] Removed {removed} existing files")


# ---------------------------------------------------------------------------
# Annotation loading
# ---------------------------------------------------------------------------

def load_mot_annotations(gt_txt_path: Path) -> dict[int, list[tuple[float, float, float, float]]]:
    """
    MOT format: frame, id, bb_left, bb_top, bb_width, bb_height, conf, x, y, z
    Returns frame_id -> list of (left, top, width, height)
    """
    frame_to_boxes: dict[int, list] = defaultdict(list)
    with gt_txt_path.open("r", newline="") as f:
        for row_idx, row in enumerate(csv.reader(f), start=1):
            row = [item.strip() for item in row if item.strip()]
            if not row or len(row) < 6:
                continue
            frame_id = int(float(row[0]))
            left, top, width, height = float(row[2]), float(row[3]), float(row[4]), float(row[5])
            if width > 0 and height > 0:
                frame_to_boxes[frame_id].append((left, top, width, height))
    return frame_to_boxes


def xywh_to_yolo(left, top, width, height, img_w, img_h):
    x_center = min(max((left + width  / 2.0) / img_w, 0.0), 1.0)
    y_center = min(max((top  + height / 2.0) / img_h, 0.0), 1.0)
    w_norm   = min(max(width  / img_w, 0.0), 1.0)
    h_norm   = min(max(height / img_h, 0.0), 1.0)
    return x_center, y_center, w_norm, h_norm


# ---------------------------------------------------------------------------
# Sequence conversion
# ---------------------------------------------------------------------------

def convert_sequence(sequence_dir: Path, split: str, output_root: Path) -> int:
    img_dir = sequence_dir / "img1"
    gt_txt  = sequence_dir / "gt" / "gt.txt"

    if not img_dir.exists():
        raise FileNotFoundError(f"Missing img1: {img_dir}")
    if not gt_txt.exists():
        raise FileNotFoundError(f"Missing gt.txt: {gt_txt}")

    frame_to_boxes = load_mot_annotations(gt_txt)
    out_img_dir = output_root / "images" / split
    out_lbl_dir = output_root / "labels" / split
    converted = 0

    for img_path in sorted(img_dir.glob("*.jpg")):
        frame_id = int(img_path.stem)
        if frame_id % KEEP_MODULO != 0:
            continue

        new_stem     = f"{DATASET_PREFIX}_{sequence_dir.name}_{img_path.stem}"
        out_img_path = out_img_dir / f"{new_stem}.jpg"
        out_lbl_path = out_lbl_dir / f"{new_stem}.txt"

        shutil.copy2(img_path, out_img_path)

        with Image.open(img_path) as im:
            img_w, img_h = im.size

        boxes = frame_to_boxes.get(frame_id, [])
        with out_lbl_path.open("w") as f:
            for left, top, width, height in boxes:
                x, y, w, h = xywh_to_yolo(left, top, width, height, img_w, img_h)
                f.write(f"0 {x:.6f} {y:.6f} {w:.6f} {h:.6f}\n")

        converted += 1

    return converted


# ---------------------------------------------------------------------------
# Split selection
# ---------------------------------------------------------------------------

def choose_val_sequences(sequences: list[Path], val_ratio: float, seed: int):
    if not sequences or val_ratio <= 0:
        return sequences, []
    rng = random.Random(seed)
    seqs = sorted(sequences, key=lambda p: p.name)
    rng.shuffle(seqs)
    n_val = max(1, round(len(seqs) * val_ratio)) if len(seqs) > 1 else 0
    val_seqs   = sorted(seqs[:n_val],  key=lambda p: p.name)
    train_seqs = sorted(seqs[n_val:],  key=lambda p: p.name)
    if not train_seqs and val_seqs:
        train_seqs = [val_seqs.pop()]
    return train_seqs, val_seqs


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------

def convert_sompt22(base_path: Path, output_root: Path) -> None:
    train_root = base_path / "train"
    test_root  = base_path / "test"

    # --- train + val ---
    if not train_root.exists():
        raise FileNotFoundError(f"Missing: {train_root}")

    all_train_seqs = [p for p in train_root.iterdir() if p.is_dir()]
    train_seqs, val_seqs = choose_val_sequences(all_train_seqs, VAL_RATIO, SEED)

    print(f"[sompt22] train sequences : {len(train_seqs)}")
    print(f"[sompt22] val sequences   : {len(val_seqs)}")

    total_train = sum(convert_sequence(s, "train", output_root) for s in train_seqs)
    total_val   = sum(convert_sequence(s, "val",   output_root) for s in val_seqs)

    print(f"[sompt22/train] {total_train} images")
    print(f"[sompt22/val]   {total_val} images")

    # --- test ---
    # SOMPT22 test sequences do not include gt.txt (annotations withheld).
    # We skip them entirely since VisDrone and Okutama already cover the test split.
    if test_root.exists():
        test_seqs = [p for p in test_root.iterdir() if p.is_dir()]
        annotated = [s for s in test_seqs if (s / "gt" / "gt.txt").exists()]
        skipped   = len(test_seqs) - len(annotated)
        if skipped:
            print(f"[sompt22/test]  Skipping {skipped} sequences with no gt.txt "
                  f"(annotations withheld)")
        if annotated:
            total_test = sum(convert_sequence(s, "test", output_root) for s in annotated)
            print(f"[sompt22/test]  {total_test} images  ({len(annotated)} sequences)")
        else:
            print(f"[sompt22/test]  No annotated test sequences found, skipping.")
    else:
        print(f"[sompt22] No test folder found at {test_root}, skipping.")


if __name__ == "__main__":
    ensure_dirs(OUTPUT_ROOT)
    cleanup(OUTPUT_ROOT)
    convert_sompt22(BASE_PATH, OUTPUT_ROOT)
    print("[sompt22] Done.")