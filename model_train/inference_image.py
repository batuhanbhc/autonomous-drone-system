from ultralytics import YOLO
import matplotlib.pyplot as plt
import cv2
import sys

MODEL_WEIGHTS = r"C:\Users\batuh\Desktop\runs\person_detector2\weights\last.pt"
CONF = 0.5

def run_and_plot(image_path: str):
    model = YOLO(MODEL_WEIGHTS)
    results = model.predict(image_path, conf=CONF)

    # plot() returns BGR at inference resolution — resize back to original
    orig = cv2.imread(image_path)
    orig_h, orig_w = orig.shape[:2]

    annotated_bgr = results[0].plot(
        line_width=1,
        labels=False,
        conf=True,
    )

    # resize annotation back to original image dimensions
    annotated_bgr = cv2.resize(annotated_bgr, (orig_w, orig_h), interpolation=cv2.INTER_LINEAR)
    annotated_rgb = cv2.cvtColor(annotated_bgr, cv2.COLOR_BGR2RGB)

    # set figure DPI so 1 figure pixel = 1 screen pixel
    dpi = 100
    fig, ax = plt.subplots(figsize=(orig_w / dpi, orig_h / dpi), dpi=dpi)
    ax.imshow(annotated_rgb)
    ax.axis("off")
    fig.tight_layout(pad=0)
    plt.show()

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python infer_plot.py /path/to/image.jpg")
        sys.exit(1)
    run_and_plot(sys.argv[1])