import math

import cv2
import numpy as np

TARGET_TAG = 2 # suction_cup / winch end effector

CAMERA_FOV = 1.5708 # rad TODO must check
CAMERA_FORWARD = 0.07 # camera is this far in front of robot

DICTIONARY = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
DETECTOR_PARAMS = cv2.aruco.DetectorParameters()


def detect_tags(image):
    # returns a list of (tag id, bearing) for every tag seen
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, DICTIONARY, parameters=DETECTOR_PARAMS)
    if ids is None:
        return []

    width = image.shape[1]
    focal_length_px = width / (2 * math.tan(CAMERA_FOV / 2))
    tags = []
    for i in range(len(ids)):
        centre_x = float(np.mean(corners[i][0][:, 0]))
        bearing = math.atan((width / 2 - centre_x) / focal_length_px)
        tags.append((int(ids[i][0]), bearing))
    return tags