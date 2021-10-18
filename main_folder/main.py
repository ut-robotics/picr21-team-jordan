import threading
import time

import cv2
import numpy as np

from image_calibration import CameraImage


class StateMachine(CameraImage):
    def __init__(self):
        super(StateMachine, self).__init__()
        cv2.namedWindow(self.original_window)
        cv2.namedWindow(self.mask_window)
        self.center_range = range(-1)

        self.state = "initial"

    def run_current_state(self, frame, mask):
        if self.state == "initial":
            x, y, size = self.get_ball_coordinates(mask, frame)
            is_ball_in_center = True if x in self.center_range else False
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
        # for debbuging and finding blobs
        # contours, _ = cv2.findContours(inspected_frame, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        # for cnt in contours:
        #     approx = cv2.approxPolyDP(cnt, 0.001 * cv2.arcLength(cnt, True), True)
        #     cv2.drawContours(target_frame, [approx], 0, (255, 0, 0), 2)

        keypoints: list = self.detector.detect(inspected_frame)
        cv2.drawKeypoints(inspected_frame, keypoints, np.array([]), (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

        # detect all keypoints
        kp_sizes = []
        minimal_size_to_detect = 30
        if len(keypoints) > 0:
            for keypoint in keypoints:
                kp_sizes.append(keypoint.size)
                if keypoint.size > minimal_size_to_detect:
                    text = str(round(keypoint.pt[0])) + " : " + str(round(keypoint.pt[1])) + ":::" + str(round(keypoint.size))
                    x, y = int(keypoint.pt[0]), int(keypoint.pt[1])
                    cv2.putText(target_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

        # detects biggest keypoint, marks with different color
        try:
            biggest_keypoint_index = kp_sizes.index(max(kp_sizes))
            biggest_keypoint = keypoints[biggest_keypoint_index]
            x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
            text = str(round(biggest_keypoint.pt[0])) + " : " + str(round(biggest_keypoint.pt[1])) + ":::" + str(round(biggest_keypoint.size))
            cv2.putText(target_frame, text, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            return x, y, size
        except ValueError:
            return 0, 0, 0
    
    def draw_info(self, frame):
        image_width = frame.shape[1]
        image_heigt = frame.shape[0]
        center = int(image_width / 2)
        center_offset = 70
        self.center_range = range(center - center_offset, center + center_offset, 1)
        cv2.line(frame, (self.center_range[0], 0), (self.center_range[0], image_heigt), (0, 0, 0), 3)
        cv2.line(frame, (self.center_range[-1], 0), (self.center_range[-1], image_heigt), (0, 0, 0), 3)

        cv2.putText(frame, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, "State: " + self.state, (120, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
    
    def main(self, socket_data):
        while True:
            if socket_data:
                self.state = socket_data.pop(0)
        
            start_time = time.time()
            _, frame = self.cap.read()
        
            mask = self.apply_image_processing(frame)
            self.run_current_state(frame, mask)

            self.draw_info(frame)
            cv2.imshow("Original", frame)
            cv2.imshow("Thresh", mask)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

        self.cap.release()
        cv2.destroyAllWindows()


class SocketServer:
    def __init__(self):
        pass

    def main(self, out_q):
        while True:
            data = str(input(":::")) #TODO socket server data
            out_q.append(data)


def producer(out_q):
    camera_image = SocketServer()
    camera_image.main(out_q)


def consumer(in_q):
    state_machine = StateMachine()
    state_machine.main(in_q)


if __name__ == "__main__":
    q = []
    t1 = threading.Thread(target=producer, args=(q,))
    t2 = threading.Thread(target=consumer, args=(q,))
    t1.start()
    t2.start()
    # camera_image = StateMachine()
    # camera_image.main()
