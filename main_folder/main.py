import threading
import time

import cv2
import numpy as np

from image_calibration import ImageCalibraion
from socket_server import SocketServer
from state_machine import StateMachine


class ImageGetter(ImageCalibraion):
    def __init__(self, enable_pyrealsense=False, enable_gui=True): #TODO keep gui separate form other code
        super(ImageGetter, self).__init__(enable_pyrealsense=enable_pyrealsense)
        self.enable_gui = enable_gui
        cv2.namedWindow(self.original_window)
        cv2.namedWindow(self.mask_window)
        
        self.STATE = "initial"
        self.CENTER_OFFSET = 70
        self.CENTER_RANGE = range(1)
        self.X, self.Y = -1, -1
        self.SIZE = -1
        self.MINIMAL_BALL_SIZE_TO_DETECT = 30

    def run_current_state(self, frame, mask):
        if self.STATE == "initial":
            self.X, self.Y, self.SIZE = self.get_ball_coordinates(mask, frame)
            is_ball_in_center = True if self.X in self.CENTER_RANGE else False
            cv2.putText(frame, "Ball in centre: " + str(is_ball_in_center), (5, 65), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2)

            ###___uncomment when movements is ready___###
            # if not is_ball_in_center:
            #     if x > self.center_range[-1]:
            #         robot_movement.move_left()

            #     if x < self.center_range[-1]:
            #         robot_movement.move_right()

            #     if x in self.center_range and size < 300:
            #         robot_movement.move_forward()

            #############################################

    def get_ball_coordinates(self, inspected_frame, target_frame):
        """initial state action"""
        keypoints: list = self.detector.detect(inspected_frame)
        cv2.drawKeypoints(inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # detect all keypoints
        kp_sizes = []
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > self.MINIMAL_BALL_SIZE_TO_DETECT:
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
                    text = str(round(x)) + " : " + str(round(y)) + ":::" + str(round(keypoint.size))
                    # cv2.putText(target_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            return int(round(x)), int(round(y)), int(round(size))
        except ValueError:
            return 0, 0, 0

    def draw_info(self, frame):
        
        cv2.line(frame, (self.CENTER_RANGE[0], 0), (self.CENTER_RANGE[0], self.HEIGHT), (0, 0, 0), 3)
        cv2.line(frame, (self.CENTER_RANGE[-1], 0), (self.CENTER_RANGE[-1], self.HEIGHT), (0, 0, 0), 3)
        cv2.putText(frame, str(self.FPS), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "State: " + self.STATE, (120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        text = str(self.X) + " : " + str(self.Y) + ":::" + str(round(self.SIZE))
        cv2.putText(frame, text, (self.X, self.Y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

    def main(self, socket_data):
        while True:
            if socket_data:
                self.STATE = socket_data.pop(0)

            start_time = time.time()
            if self.enable_pyrealsense:

                color_image, depth_image = self.get_frame_using_pyrealsense() # TODO do something with depth
                mask_image = self.apply_image_processing(color_image)  
            else:
                _, color_image = self.cap.read()
                mask_image = self.apply_image_processing(color_image)

            self.WIDTH = mask_image.shape[1]
            self.HEIGHT = mask_image.shape[0]
            self.CENTER_RANGE = range(int(self.WIDTH / 2) - self.CENTER_OFFSET, int(self.WIDTH / 2) + self.CENTER_OFFSET, 1)
            self.X, self.Y, self.SIZE = self.get_ball_coordinates(mask_image, color_image)

            # self.run_current_state(color_image, mask_image) #TODO make state class in different file.

            self.draw_info(color_image)
            cv2.imshow(self.original_window, color_image)
            cv2.imshow(self.mask_window, mask_image)
            cv2.imshow(self.depth_window, depth_image) if self.enable_pyrealsense else -1
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.FPS = round(1.0 / (time.time() - start_time), 2)

        self.cap.release() if not self.enable_pyrealsense else self.pipeline.stop()
        cv2.destroyAllWindows() if self.enable_gui else -1


def producer(out_q):
    camera_image = SocketServer()
    camera_image.main(out_q)


def consumer(in_q):
    state_machine = ImageGetter(enable_pyrealsense=False, enable_gui=True)
    state_machine.main(in_q)


if __name__ == "__main__":
    q = []    
    t1 = threading.Thread(target=producer, args=(q,))
    t1.daemon = True
    t2 = threading.Thread(target=consumer, args=(q,))
    t1.start()
    t2.start()
