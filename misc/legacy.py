# TODO delete legacy, if no more blob used
# BLOB_MIN_AREA = 0
# BLOB_MAX_AREA = 999_999
# MIN_DISTANCE_BETWEEN_BLOBS = 40
# MINIMAL_BALL_SIZE_TO_DETECT = 7


# self.blobparams = cv2.SimpleBlobDetector_Params()
# self.blobparams.minDistBetweenBlobs = const.MIN_DISTANCE_BETWEEN_BLOBS
# self.blobparams.filterByArea = True
# self.blobparams.minArea = const.BLOB_MIN_AREA
# self.blobparams.maxArea = const.BLOB_MAX_AREA
# self.blobparams.filterByInertia = False
# self.blobparams.filterByConvexity = False
# self.blobparams.filterByCircularity = False
# self.detector = cv2.SimpleBlobDetector_create(self.blobparams)


# def get_biggest_blob_coords(self, inspected_frame):
#     keypoints: list = self.detector.detect(inspected_frame)
#     # detect all keypoints
#     kp_sizes = []
#     if len(keypoints) > 0:
#         for keypoint in keypoints:
#             kp_sizes.append(keypoint.size)
#             if keypoint.size > const.MINIMAL_BALL_SIZE_TO_DETECT:
#                 x, y = int(keypoint.pt[0]), int(keypoint.pt[1])

#     # detects biggest keypoint
#     try:
#         biggest_keypoint = keypoints[kp_sizes.index(max(kp_sizes))]
#         x, y, size = int(biggest_keypoint.pt[0]), int(biggest_keypoint.pt[1]), biggest_keypoint.size
#         return int(round(x)), int(round(y)), int(round(size))
#     except ValueError:
#         return -1, -1, -1

print("2222")