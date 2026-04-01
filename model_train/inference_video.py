"""
inference_video.py
──────────────────
Run your trained YOLO person-detector on an .mp4 file and save
an annotated output video.

Usage:
    python inference_video.py

Edit the CONFIG block below to point at your weights and video.
"""

from __future__ import annotations

from pathlib import Path

import cv2
from ultralytics import YOLO

# ─────────────────────────── CONFIG ────────────────────────────────────────
MODEL_PATH  = r"C:\Users\batuh\Desktop\runs\person_detector12\weights\last.pt"
VIDEO_PATH  = r"C:\Users\batuh\Desktop\process_dataset\proper.mp4"
OUTPUT_PATH = r""           # leave empty → saves next to VIDEO_PATH as *_annotated.mp4

CONF_THRESH  = 0.5         # detection confidence threshold
IOU_THRESH   = 0.7         # NMS IoU threshold
IMGSZ        = 800          # inference image size (must match training)
DEVICE       = 0            # 0 = first GPU, "cpu" = CPU

# Drawing style
BOX_COLOR    = (0, 255, 0)  # BGR  — green
TEXT_COLOR   = (0, 255, 0)
BOX_THICKNESS = 2
FONT          = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE    = 0.55
FONT_THICK    = 1

SHOW_WINDOW   = True        # set True to pop up a live preview window (needs display)
# ───────────────────────────────────────────────────────────────────────────


def build_output_path(video_path: str, output_path: str) -> Path:
    if output_path:
        return Path(output_path)
    p = Path(video_path)
    return p.with_stem(p.stem + "_annotated")


def draw_detections(frame, boxes, confs):
    for (x1, y1, x2, y2), conf in zip(boxes, confs):
        cv2.rectangle(frame, (x1, y1), (x2, y2), BOX_COLOR, BOX_THICKNESS)
        label = f"person {conf:.2f}"
        (tw, th), baseline = cv2.getTextSize(label, FONT, FONT_SCALE, FONT_THICK)
        # filled label background
        cv2.rectangle(frame, (x1, y1 - th - baseline - 4), (x1 + tw + 2, y1), BOX_COLOR, -1)
        cv2.putText(frame, label, (x1 + 1, y1 - baseline - 2),
                    FONT, FONT_SCALE, (0, 0, 0), FONT_THICK, cv2.LINE_AA)
    return frame


def run_inference():
    video_path  = Path(VIDEO_PATH)
    output_path = build_output_path(VIDEO_PATH, OUTPUT_PATH)

    if not video_path.exists():
        raise FileNotFoundError(f"Video not found: {video_path}")
    if not Path(MODEL_PATH).exists():
        raise FileNotFoundError(f"Model weights not found: {MODEL_PATH}")

    print(f"Model  : {MODEL_PATH}")
    print(f"Video  : {video_path}")
    print(f"Output : {output_path}")

    model = YOLO(MODEL_PATH)

    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open video: {video_path}")

    width  = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps    = cap.get(cv2.CAP_PROP_FPS) or 25.0
    total  = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))

    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(str(output_path), fourcc, fps, (width, height))

    print(f"Resolution: {width}×{height}  |  FPS: {fps:.1f}  |  Frames: {total}")
    print("Running inference … (press Q in preview window to quit early)")

    frame_idx = 0
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        results = model.predict(
            source=frame,
            conf=CONF_THRESH,
            iou=IOU_THRESH,
            imgsz=IMGSZ,
            device=DEVICE,
            verbose=False,
        )

        # Extract boxes (xyxy) and confidences
        res = results[0]
        if res.boxes is not None and len(res.boxes):
            xyxy  = res.boxes.xyxy.cpu().numpy().astype(int)
            confs = res.boxes.conf.cpu().numpy()
            boxes = [(x1, y1, x2, y2) for x1, y1, x2, y2 in xyxy]
        else:
            boxes, confs = [], []

        annotated = draw_detections(frame, boxes, confs)

        # Frame counter overlay
        cv2.putText(annotated, f"frame {frame_idx+1}/{total}  det:{len(boxes)}",
                    (8, 24), FONT, 0.65, (255, 255, 255), 2, cv2.LINE_AA)
        cv2.putText(annotated, f"frame {frame_idx+1}/{total}  det:{len(boxes)}",
                    (8, 24), FONT, 0.65, (30, 30, 30), 1, cv2.LINE_AA)

        writer.write(annotated)

        if SHOW_WINDOW:
            cv2.imshow("Inference", annotated)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                print("Early exit via Q key.")
                break

        frame_idx += 1
        if frame_idx % 50 == 0:
            pct = frame_idx / total * 100 if total else 0
            print(f"  {frame_idx}/{total}  ({pct:.1f}%)")

    cap.release()
    writer.release()
    if SHOW_WINDOW:
        cv2.destroyAllWindows()

    print(f"\nDone. Annotated video saved to:\n  {output_path}")


if __name__ == "__main__":
    run_inference()