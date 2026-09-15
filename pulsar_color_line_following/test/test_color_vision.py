"""Functional checks for the start-area orange stripe."""

import cv2
import numpy as np

from pulsar_color_line_following.color_vision import detect_orange_line


def make_frame(center):
    frame = np.full((720, 1280, 3), 230, np.uint8)
    cv2.rectangle(frame, (center - 65, 240), (center - 23, 719),
                  (160, 90, 30), -1)
    cv2.rectangle(frame, (center - 22, 240), (center + 22, 719),
                  (0, 135, 255), -1)
    cv2.rectangle(frame, (center + 23, 240), (center + 65, 719),
                  (160, 90, 30), -1)
    # A white QR patch interrupts the orange line close to the image bottom.
    cv2.rectangle(frame, (center - 28, 650), (center + 28, 710),
                  (255, 255, 255), -1)
    return frame


def test_orange_position_with_blue_sides_and_qr_patch():
    left = detect_orange_line(make_frame(480))
    middle = detect_orange_line(make_frame(640))
    right = detect_orange_line(make_frame(800))
    assert left.error < -0.20
    assert abs(middle.error) < 0.02
    assert right.error > 0.20
    assert all(d.center is not None for d in (left, middle, right))


def test_blue_without_orange_is_not_a_line():
    frame = np.full((720, 1280, 3), 230, np.uint8)
    cv2.rectangle(frame, (500, 240), (640, 719), (160, 90, 30), -1)
    assert detect_orange_line(frame).center is None


def test_previous_center_blocks_large_jump():
    detection = detect_orange_line(make_frame(900), previous_center_x=300)
    assert detection.center is None
