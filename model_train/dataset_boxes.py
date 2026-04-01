from pathlib import Path
from collections import defaultdict

DATASET_ROOT = Path(r"C:\Users\batuh\Desktop\combined_dataset")


def count_split(split: str) -> tuple[int, int]:
    labels_dir = DATASET_ROOT / "labels" / split
    if not labels_dir.exists():
        return 0, 0

    total_images = 0
    total_boxes = 0

    for lbl_path in labels_dir.glob("*.txt"):
        lines = [l.strip() for l in lbl_path.read_text().splitlines() if l.strip()]
        if lines:  # skip empty label files (background images)
            total_images += 1
            total_boxes += len(lines)

    return total_images, total_boxes


def count_by_prefix(split: str) -> dict[str, tuple[int, int]]:
    """Break down image/box counts by dataset prefix (visdrone_, sompt22_, etc.)"""
    labels_dir = DATASET_ROOT / "labels" / split
    if not labels_dir.exists():
        return {}

    prefix_stats: dict[str, list[int, int]] = defaultdict(lambda: [0, 0])

    for lbl_path in labels_dir.glob("*.txt"):
        lines = [l.strip() for l in lbl_path.read_text().splitlines() if l.strip()]
        if not lines:
            continue

        # Extract prefix (everything before the first underscore group)
        # e.g. "visdrone_train_...", "sompt22_...", "okutama_..."
        name = lbl_path.stem
        prefix = name.split("_")[0]

        prefix_stats[prefix][0] += 1
        prefix_stats[prefix][1] += len(lines)

    return {k: tuple(v) for k, v in prefix_stats.items()}


def main():
    splits = ["train", "val", "test"]

    print("=" * 55)
    print(f"{'Dataset':30s} {'Images':>8} {'Boxes':>10}")
    print("=" * 55)

    grand_images = 0
    grand_boxes = 0

    for split in splits:
        images, boxes = count_split(split)
        if images == 0:
            continue

        grand_images += images
        grand_boxes += boxes

        print(f"\n[{split.upper()}]  {images} images,  {boxes} boxes")

        by_prefix = count_by_prefix(split)
        for prefix, (n_img, n_box) in sorted(by_prefix.items()):
            avg = n_box / n_img if n_img else 0
            print(f"  {prefix:28s} {n_img:>8} images  {n_box:>10} boxes  (avg {avg:.1f}/img)")

    print("=" * 55)
    print(f"  {'TOTAL':28s} {grand_images:>8} images  {grand_boxes:>10} boxes")
    print(f"  {'avg boxes/image':28s} {grand_boxes/grand_images:.1f}" if grand_images else "")
    print("=" * 55)


if __name__ == "__main__":
    main()