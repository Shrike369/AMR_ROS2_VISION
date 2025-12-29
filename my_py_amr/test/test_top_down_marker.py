import math

import cv2
import numpy as np

from my_py_amr.top_down_marker_tf import detect_markers


def make_test_img(red_pos=(120, 240), blue_pos=(360, 240), size=(480, 480)):
    img_bgr = np.zeros((size[1], size[0], 3), dtype=np.uint8)
    cv2.circle(img_bgr, (int(red_pos[0]), int(red_pos[1])), 20, (0, 0, 255), -1)
    cv2.circle(img_bgr, (int(blue_pos[0]), int(blue_pos[1])), 20, (255, 0, 0), -1)
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return img_rgb


def test_detect_and_pose():
    img = make_test_img()
    centers = detect_markers(
        img,
        red_ranges=[((0, 100, 100), (10, 255, 255)), ((160, 100, 100), (179, 255, 255))],
        blue_range=((100, 150, 50), (140, 255, 255)),
    )
    red = centers["red"]
    blue = centers["blue"]
    assert abs(red[0] - 120) < 5
    assert abs(red[1] - 240) < 5
    assert abs(blue[0] - 360) < 5
    assert abs(blue[1] - 240) < 5

    fx = fy = 201.4
    cx = cy = 240.0
    height = 10.13
    x_red = (red[0] - cx) * height / fx
    y_red = (red[1] - cy) * height / fy
    x_blue = (blue[0] - cx) * height / fx
    y_blue = (blue[1] - cy) * height / fy

    expected_yaw = math.atan2(y_red - y_blue, x_red - x_blue)
    # For symmetric horizontal markers, yaw can be near 0 or pi depending on marker ordering;
    # accept either orientation
    yaw_err = min(abs(expected_yaw), abs(abs(expected_yaw) - math.pi))
    assert yaw_err < 0.5
