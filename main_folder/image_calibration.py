import time

import cv2

import constants as const
import file_manager
from camera import Camera
from image_processing import ImageProcessing


class ImageCalibration:
    """
    This class calibrates threshold values to properly see all objects. Saves values to /config file
    """

    def __init__(self):
        self.Cam = Camera()
        self.ImageProcess = ImageProcessing()
        self.default_values_ball = file_manager.get_default_values(const.CONFIG_PATH, const.BALL)
        self.default_values_basket_blue = file_manager.get_default_values(const.CONFIG_PATH, const.BASKET_BLUE)
        self.default_values_basket_rose = file_manager.get_default_values(const.CONFIG_PATH, const.BASKET_ROSE)
        self.fps = 0

    def update_value(self, new_value):
        self.trackbar_value = new_value

    def mainloop(self, type):
        if type == const.BALL:
            default_values = self.default_values_ball
        elif type == const.BASKET_BLUE:
            default_values = self.default_values_basket_blue
        elif type == const.BASKET_ROSE:
            default_values = self.default_values_basket_rose

        cv2.namedWindow(const.ORIGINAL_WINDOW)
        cv2.namedWindow(const.MASKED_WINDOW)
        cv2.namedWindow(const.TRACKBAR_WINDOW, cv2.WINDOW_NORMAL)
        cv2.namedWindow(const.DEPTH_WINDOW)

        for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh"]):
            cv2.createTrackbar(value, const.TRACKBAR_WINDOW, default_values[index], 255, self.update_value)
        for index, value in enumerate(["dil1", "dil2", "clos1", "clos2"]):
            cv2.createTrackbar(value, const.TRACKBAR_WINDOW, default_values[index + 6], 50, self.update_value)

        while True:
            start_time = time.time()

            for index, value in enumerate(["hl", "sl", "vl", "hh", "sh", "vh", "dil1", "dil2", "clos1", "clos2"]):
                default_values[index] = cv2.getTrackbarPos(value, const.TRACKBAR_WINDOW)

            color_image = self.Cam.get_rgb_frame()
            color_image = cv2.resize(color_image, (const.WIDTH_RESIZED, const.HEIGHT_RESIZED))
            depth_image = self.Cam.get_depth_frame()
            mask_image = self.ImageProcess.get_masked_image(color_image, type, default_values=default_values)

            x, y, radius, center = self.ImageProcess.get_obj_coords(mask_image)
            if radius > const.MIN_BALL_RADIUS_TO_DETECT:
                cv2.circle(color_image, (int(x), int(y)), int(radius), (0, 255, 255), 5)
                cv2.circle(color_image, center, 5, (0, 0, 255), -1)
                cv2.putText(color_image, str(round(x)) + " : " + str(round(y)) + "\n" + str(round(radius)), (5, 80), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(color_image, str(self.fps), (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.putText(color_image, type, (5, 55), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            cv2.imshow(const.ORIGINAL_WINDOW, color_image)
            cv2.imshow(const.MASKED_WINDOW, mask_image)
            cv2.imshow(const.DEPTH_WINDOW, depth_image)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                file_manager.save_default_values(const.CONFIG_PATH, type, [str(x) for x in default_values])
                break
            self.fps = round(1.0 / (time.time() - start_time), 2)

    def main(self):
        self.mainloop(const.BALL)
        # self.mainloop(const.BASKET_BLUE)
        # self.mainloop(const.BASKET_ROSE)
        self.Cam.stop_camera()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    camera_image = ImageCalibration()
    camera_image.main()
