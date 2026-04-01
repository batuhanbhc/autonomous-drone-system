import cv2
import os

# --- Configuration ---
CAMERA_INDEX = 2
WIDTH = 640
HEIGHT = 480
FORMAT = "MJPEG"  # "MJPEG" or "YUY2"
# ---------------------

save_dir = "calibration_images"
os.makedirs(save_dir, exist_ok=True)

cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)

if not cap.isOpened():
    print(f"Could not open camera {CAMERA_INDEX}")
    exit()

# Set pixel format BEFORE resolution
if FORMAT == "MJPEG":
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
elif FORMAT == "YUY2":
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUY2"))
else:
    print(f"Unknown format: {FORMAT}")
    exit()

cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)

# Verify what was actually set
actual_w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
actual_h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
actual_fourcc = int(cap.get(cv2.CAP_PROP_FOURCC))
actual_fmt = "".join([chr((actual_fourcc >> 8 * i) & 0xFF) for i in range(4)])

print(f"Requested: {WIDTH}x{HEIGHT} @ {FORMAT}")
print(f"Actual:    {actual_w}x{actual_h} @ {actual_fmt}")

if actual_w != WIDTH or actual_h != HEIGHT:
    print("WARNING: Camera did not accept requested resolution!")

img_count = 0
print("\nPress 's' to save an image")
print("Press Ctrl+C to exit\n")

try:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        cv2.imshow("Camera", frame)
        key = cv2.waitKey(1) & 0xFF

        if key == ord('s'):
            filename = os.path.join(save_dir, f"calib_{img_count:03d}.png")
            cv2.imwrite(filename, frame)
            print(f"Saved {filename}  ({img_count + 1} total)")
            img_count += 1

except KeyboardInterrupt:
    print(f"\nStopping camera... ({img_count} images saved)")
finally:
    cap.release()
    cv2.destroyAllWindows()