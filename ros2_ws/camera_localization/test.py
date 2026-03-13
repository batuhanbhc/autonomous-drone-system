import cv2

img = cv2.imread("snapshot.jpg")

def click(event, x, y, flags, param):
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"x={x}, y={y}")

cv2.imshow("img", img)
cv2.setMouseCallback("img", click)
cv2.waitKey(0)
cv2.destroyAllWindows()