"""
Multithreading example. May be useful later, but now use main.py instead
"""

from image_calibration import CameraImage
import time

import cv2
import threading
from queue import Queue


class ImageProducer(CameraImage):
    def __init__(self):
        super().__init__()
        pass

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


class StateMachine():
    def __init__(self):
        self.initial_state = "initial"

    def main(self, in_q):
        cv2.namedWindow("Mask")
        while True:
            mask = in_q.get()
            cv2.imshow("Mask", mask)
            self.run_current_state(mask)
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
    t1 = threading.Thread(target=producer, args =(q, ))
    t2 = threading.Thread(target=consumer, args =(q, ))
    t1.start()
    t2.start()
