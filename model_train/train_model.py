"""
train.py — YOLO11 person detector, fully configurable
══════════════════════════════════════════════════════
Edit the CONFIG sections below, then run:
    python train.py
"""

from ultralytics import YOLO
from pathlib import Path


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  PATHS                                                                   ║
# ╚══════════════════════════════════════════════════════════════════════════╝
MODEL_WEIGHTS   = "yolo11s.pt"          # yolo11n / s / m / l / x  .pt
DATA_YAML       = r"C:\Users\batuh\Desktop\combined_dataset\data.yaml"
PROJECT_DIR     = r"C:\Users\batuh\Desktop\runs"
RUN_NAME        = "person_detector"

RESUME          = False    # True → resume from last interrupted run


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  HARDWARE                                                                ║
# ╚══════════════════════════════════════════════════════════════════════════╝
DEVICE          = 0        # int GPU index | "cpu" | "0,1" for multi-GPU
WORKERS         = 4        # dataloader worker threads
BATCH           = 16       # -1 = auto-batch (uses 60 % of GPU VRAM)
IMGSZ           = 800


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TRAINING SCHEDULE                                                       ║
# ╚══════════════════════════════════════════════════════════════════════════╝
EPOCHS          = 100
PATIENCE        = 0       # early-stop if no mAP50 improvement for N epochs
                            # set 0 to disable early stopping

WARMUP_EPOCHS   = 3.0      # epochs of LR/momentum warmup (float ok)
WARMUP_MOMENTUM = 0.8      # initial momentum during warmup
WARMUP_BIAS_LR  = 0.1      # bias LR during warmup

CLOSE_MOSAIC    = 20       # disable mosaic for last N epochs (stabilises training)
                            # set 0 to keep mosaic on throughout

FREEZE          = 10

# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  OPTIMIZER                                                               ║
# ╚══════════════════════════════════════════════════════════════════════════╝
OPTIMIZER       = "SGD"    # SGD | Adam | AdamW | NAdam | RAdam | RMSProp | auto

LR0             = 0.001   # initial learning rate
LRF             = 0.01     # final LR = LR0 * LRF  (cosine annealing target)
MOMENTUM        = 0.937    # SGD momentum / Adam beta1
WEIGHT_DECAY    = 0.0005   # L2 regularisation — try 0.0005 if overfitting is worse
# NOTE: nesterov is NOT a valid Ultralytics arg — it's always on internally for SGD


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  LOSS WEIGHTS                                                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝
# Increase BOX_LOSS to improve localisation on small objects
BOX_LOSS        = 7.5
CLS_LOSS        = 0.5
DFL_LOSS        = 1.5


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  AUGMENTATION                                                            ║
# ╚══════════════════════════════════════════════════════════════════════════╝
# ── Spatial ──────────────────────────────────────────────────────────────
MOSAIC          = 0.9      # mosaic probability  [0.0 – 1.0]
MIXUP           = 0.0      # mixup probability   [0.0 – 1.0]  try 0.1
COPY_PASTE      = 0.0      # copy-paste prob     [0.0 – 1.0]  try 0.3 for crowds

DEGREES         = 5.0      # random rotation ± degrees
TRANSLATE       = 0.15      # random translation ± fraction of image
SCALE           = 0.5      # random scale       ± fraction  (0.5 → 0.5×–1.5×)
SHEAR           = 1.0      # random shear       ± degrees
PERSPECTIVE     = 0.00025      # perspective warp   [0.0 – 0.001]
FLIPUD          = 0.0      # vertical flip probability
FLIPLR          = 0.5      # horizontal flip probability

# ── Colour / photometric ─────────────────────────────────────────────────
HSV_H           = 0.02     # hue shift          ± fraction of 180°
HSV_S           = 0.7      # saturation shift   ± fraction
HSV_V           = 0.4      # value (brightness) ± fraction

ERASING         = 0.4      # random erasing probability (occlusion simulation)


# ── Multi-scale training ─────────────────────────────────────────────────
MULTI_SCALE     = False    # vary imgsz ±50% each batch — helps with drone footage


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  INFERENCE / NMS (used during val inside training)                       ║
# ╚══════════════════════════════════════════════════════════════════════════╝
CONF            = 0.001    # minimum confidence to keep a prediction  (keep low for val)
IOU             = 0.7      # NMS IoU threshold
MAX_DET         = 300      # max detections per image


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  MISC / LOGGING                                                          ║
# ╚══════════════════════════════════════════════════════════════════════════╝
SEED            = 42
SAVE_PERIOD     = 10       # save checkpoint every N epochs  (-1 = only best/last)
EXIST_OK        = False    # True → overwrite existing run folder
PLOTS           = True
VERBOSE         = True


# ╔══════════════════════════════════════════════════════════════════════════╗
# ║  TEST EVAL & EXPORT                                                      ║
# ╚══════════════════════════════════════════════════════════════════════════╝
RUN_TEST_EVAL   = True

EXPORT_FORMAT   = "onnx"   # onnx | torchscript | engine (TensorRT) | coreml | …
EXPORT_OPSET    = 11       # ONNX opset version
EXPORT_SIMPLIFY = True
EXPORT_DYNAMIC  = False    # True = dynamic batch axis in ONNX graph
EXPORT_HALF     = False    # FP16 export (TensorRT / CoreML only)
EXPORT_INT8     = False    # INT8 quantisation (TensorRT only, needs calib data)


# ══════════════════════════════════════════════════════════════════════════
if __name__ == "__main__":

    model = YOLO(MODEL_WEIGHTS)

    results = model.train(
        # ── paths ────────────────────────────────────────────────────────
        data            = DATA_YAML,
        project         = PROJECT_DIR,
        name            = RUN_NAME,
        resume          = RESUME,
        exist_ok        = EXIST_OK,

        # ── hardware ─────────────────────────────────────────────────────
        device          = DEVICE,
        workers         = WORKERS,
        batch           = BATCH,
        imgsz           = IMGSZ,

        # ── schedule ─────────────────────────────────────────────────────
        epochs          = EPOCHS,
        patience        = PATIENCE,
        warmup_epochs   = WARMUP_EPOCHS,
        warmup_momentum = WARMUP_MOMENTUM,
        warmup_bias_lr  = WARMUP_BIAS_LR,
        close_mosaic    = CLOSE_MOSAIC,
        freeze          = FREEZE,

        # ── optimiser ────────────────────────────────────────────────────
        optimizer       = OPTIMIZER,
        lr0             = LR0,
        lrf             = LRF,
        momentum        = MOMENTUM,
        weight_decay    = WEIGHT_DECAY,

        # ── loss ─────────────────────────────────────────────────────────
        box             = BOX_LOSS,
        cls             = CLS_LOSS,
        dfl             = DFL_LOSS,

        # ── augmentation ─────────────────────────────────────────────────
        mosaic          = MOSAIC,
        mixup           = MIXUP,
        copy_paste      = COPY_PASTE,
        degrees         = DEGREES,
        translate       = TRANSLATE,
        scale           = SCALE,
        shear           = SHEAR,
        perspective     = PERSPECTIVE,
        flipud          = FLIPUD,
        fliplr          = FLIPLR,
        hsv_h           = HSV_H,
        hsv_s           = HSV_S,
        hsv_v           = HSV_V,
        erasing         = ERASING,
        multi_scale     = MULTI_SCALE,

        # ── nms / inference ──────────────────────────────────────────────
        conf            = CONF,
        iou             = IOU,
        max_det         = MAX_DET,

        # ── misc ─────────────────────────────────────────────────────────
        seed            = SEED,
        save            = True,
        save_period     = SAVE_PERIOD,
        val             = True,
        plots           = PLOTS,
        verbose         = VERBOSE,
    )

    # ── test-set evaluation (both last and best) ──────────────────────────
    if RUN_TEST_EVAL:
        import json, datetime

        data_yaml = Path(DATA_YAML)

        for tag in ("last", "best"):
            weights      = Path(results.save_dir) / "weights" / f"{tag}.pt"
            test_out_dir = Path(results.save_dir) / f"test_eval_{tag}"

            print(f"\n── Test Set Results [{tag}] ───────────────────")

            m = YOLO(str(weights))
            metrics = m.val(
                data        = str(data_yaml),
                split       = "test",
                imgsz       = IMGSZ,
                batch       = BATCH,
                device      = DEVICE,
                conf        = 0.001,
                iou         = IOU,
                plots       = True,
                save_dir    = str(test_out_dir),
            )

            print(f"  mAP50    : {metrics.box.map50:.4f}")
            print(f"  mAP50-95 : {metrics.box.map:.4f}")
            print(f"  Precision: {metrics.box.mp:.4f}")
            print(f"  Recall   : {metrics.box.mr:.4f}")

            test_results = {
                "timestamp"  : datetime.datetime.now().isoformat(timespec="seconds"),
                "tag"        : tag,
                "weights"    : str(weights),
                "data_yaml"  : str(data_yaml),
                "split"      : "test",
                "imgsz"      : IMGSZ,
                "conf"       : 0.001,
                "iou"        : IOU,
                "metrics": {
                    "mAP50"    : round(metrics.box.map50, 6),
                    "mAP50_95" : round(metrics.box.map,   6),
                    "precision": round(metrics.box.mp,    6),
                    "recall"   : round(metrics.box.mr,    6),
                },
            }

            test_out_dir.mkdir(parents=True, exist_ok=True)
            json_path = test_out_dir / "test_results.json"
            json_path.write_text(json.dumps(test_results, indent=2))
            print(f"  Results saved → {json_path}")

    # ── export both last and best ─────────────────────────────────────────
    for tag in ("last", "best"):
        weights = Path(results.save_dir) / "weights" / f"{tag}.pt"
        YOLO(str(weights)).export(
            format   = EXPORT_FORMAT,
            imgsz    = IMGSZ,
            opset    = EXPORT_OPSET,
            simplify = EXPORT_SIMPLIFY,
            dynamic  = EXPORT_DYNAMIC,
            half     = EXPORT_HALF,
            int8     = EXPORT_INT8,
        )
        print(f"  Exported {tag}.onnx")

    print("\nDone.")