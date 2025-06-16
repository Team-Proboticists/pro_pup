import cv2
import time

# Open camera using V4L2 backend
cap = cv2.VideoCapture(0, cv2.CAP_V4L2)

# Set resolution (already applied via v4l2-ctl, but safe to re-assert)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

# Optional: Set FOURCC to BGR3 to match current kernel format
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'BGR3'))

# Add short warm-up delay
time.sleep(0.5)

if not cap.isOpened():
    print("❌ Cannot open camera")
    exit()

# Try to read a few frames, in case the first is empty
for i in range(10):
    ret, frame = cap.read()
    if ret:
        print(f"✅ Grabbed frame {i+1}")
        cv2.imshow("IMX219 - BGR3", frame)
        if cv2.waitKey(0) == 27:
            break
    else:
        print(f"❌ Frame {i+1} failed")

cap.release()
cv2.destroyAllWindows()
