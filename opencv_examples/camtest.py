import cv2
import time

cap = cv2.VideoCapture(1)
# cap.set(cv2.CAP_PROP_FRAME_WIDTH, 848)
# cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M', 'J', 'P', 'G'))

fps = 0
while True:
    start_time = time.time()
    
    ret, frame = cap.read()
    cv2.putText(frame, str(fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    cv2.imshow("Original", frame)

    
    if cv2.waitKey(1) & 0xFF == ord("q"):
        break

    fps = round(1.0 / (time.time() - start_time), 2)
    
print("closing program")
cap.release()
cv2.destroyAllWindows()

