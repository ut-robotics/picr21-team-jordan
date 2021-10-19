"""
Multithreading example. May be useful later, but now use main.py instead
"""
import threading
import time
from queue import Queue
import numpy as np

import cv2
from image_calibration import ImageCalibraion


class ImageProducer(ImageCalibraion):
    def __init__(self):
        super().__init__()

    def apply_image_processing(self, frame):
        hsv = cv2.cvtColor(frame, self.color_type)
        hsv_blured = cv2.medianBlur(hsv, 5)
        hl = self.default_values_ball[0]
        sl = self.default_values_ball[1]
        vl = self.default_values_ball[2]
        hh = self.default_values_ball[3]
        sh = self.default_values_ball[4]
        vh = self.default_values_ball[5]
        clos1 = self.default_values_ball[6]
        clos2 = self.default_values_ball[7]
        dil1 = self.default_values_ball[8]
        dil2 = self.default_values_ball[9]
        kernel1 = np.ones((clos1, clos2), np.uint8)
        kernel2 = np.ones((dil1, dil2), np.uint8)
        lowerLimits = np.array([hl, sl, vl])
        upperLimits = np.array([hh, sh, vh])
        mask = cv2.inRange(hsv_blured, lowerLimits, upperLimits)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel1)
        mask = cv2.dilate(mask, kernel2, iterations=1)
        mask = cv2.bitwise_not(mask)

        return mask

    def main(self, out_q):
        while True:
            start_time = time.time()
            _, frame = self.cap.read()
            mask = self.apply_image_processing(frame)

            cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow("Original", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)
            out_q.put(mask)

        self.cap.release()
        cv2.destroyAllWindows()


class StateMachine:
    def __init__(self):
        self.initial_state = "initial"

    def main(self, in_q):
        cv2.namedWindow("Mask")
        while True:
            mask = in_q.get()
            cv2.imshow("Mask", mask)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break


def producer(out_q):
    camera_image = ImageProducer()
    camera_image.main(out_q)


def consumer(in_q):
    state_machine = StateMachine()
    state_machine.main(in_q)


if __name__ == "__main__":
    q = Queue()
    t1 = threading.Thread(target=producer, args=(q,))
    t2 = threading.Thread(target=consumer, args=(q,))
    t1.start()
    t2.start()
