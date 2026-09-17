"""Orange center stripe detection without a fixed pixel width."""

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class ColorDetection:
    mask: np.ndarray
    roi_top: int
    roi_bottom: int
    contour: object = None
    center: object = None
    error: object = None
    area: float = 0.0


def detect_orange_line(
    frame,
    roi_start_ratio=0.45,
    roi_end_ratio=0.88,
    hsv_lower=(5, 80, 70),
    hsv_upper=(25, 255, 255),
    min_area_ratio=0.0005,
    minimum_height_ratio=0.18,
    previous_center_x=None,
    maximum_center_jump_ratio=0.25,
):
    """Find an orange stripe in the lower image, away from the QR at its foot."""

    if (
        frame is None
        or frame.size == 0
        or frame.ndim != 3
        or frame.shape[2] != 3
    ):
        raise ValueError(
            'frame must be a non-empty BGR image'
        )

    height, width = frame.shape[:2]

    if not 0 <= roi_start_ratio < roi_end_ratio <= 1:
        raise ValueError(
            'ROI ratios must satisfy 0 <= start < end <= 1'
        )

    top = int(height * roi_start_ratio)
    bottom = max(
        top + 1,
        int(height * roi_end_ratio),
    )
    bottom = min(
        bottom,
        height,
    )

    roi = frame[top:bottom]

    hsv = cv2.cvtColor(
        roi,
        cv2.COLOR_BGR2HSV,
    )

    lower = np.asarray(
        hsv_lower,
        dtype=np.uint8,
    )

    upper = np.asarray(
        hsv_upper,
        dtype=np.uint8,
    )

    if (
        lower.shape != (3,)
        or upper.shape != (3,)
        or np.any(lower > upper)
    ):
        raise ValueError(
            'HSV bounds must be three increasing channel values'
        )

    mask = cv2.inRange(
        hsv,
        lower,
        upper,
    )

    kernel = np.ones(
        (3, 3),
        dtype=np.uint8,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_OPEN,
        kernel,
    )

    mask = cv2.morphologyEx(
        mask,
        cv2.MORPH_CLOSE,
        kernel,
    )

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )

    candidates = []

    minimum_area = (
        float(min_area_ratio)
        * width
        * (bottom - top)
    )

    for contour in contours:
        area = cv2.contourArea(
            contour
        )

        if area < minimum_area:
            continue

        _, _, stripe_width, stripe_height = (
            cv2.boundingRect(contour)
        )

        if (
            stripe_height
            < minimum_height_ratio * (bottom - top)
        ):
            continue

        if stripe_width > 0.5 * width:
            continue

        moments = cv2.moments(
            contour
        )

        if moments['m00'] <= 0:
            continue

        x = int(
            moments['m10']
            / moments['m00']
        )

        y = (
            int(
                moments['m01']
                / moments['m00']
            )
            + top
        )

        if previous_center_x is not None:
            distance = abs(
                x - previous_center_x
            )

            if (
                distance
                > maximum_center_jump_ratio * width
            ):
                continue

            reference = previous_center_x

        else:
            reference = width / 2

        score = (
            area
            / max(
                stripe_width * stripe_height,
                1,
            )
            - 0.35
            * abs(x - reference)
            / width
        )

        candidates.append(
            (
                score,
                contour,
                (x, y),
                area,
            )
        )

    if not candidates:
        return ColorDetection(
            mask,
            top,
            bottom,
        )

    _, contour, center, area = max(
        candidates,
        key=lambda item: item[0],
    )

    error = float(
        np.clip(
            (center[0] - width / 2)
            / (width / 2),
            -1,
            1,
        )
    )

    return ColorDetection(
        mask,
        top,
        bottom,
        contour,
        center,
        error,
        area,
    )


def draw_detection(
    frame,
    detection,
):
    """Annotate a copy of the camera frame."""

    image = frame.copy()

    height, width = image.shape[:2]

    cv2.rectangle(
        image,
        (0, detection.roi_top),
        (
            width - 1,
            detection.roi_bottom - 1,
        ),
        (255, 0, 0),
        2,
    )

    cv2.line(
        image,
        (
            width // 2,
            detection.roi_top,
        ),
        (
            width // 2,
            detection.roi_bottom,
        ),
        (0, 255, 255),
        2,
    )

    if detection.center is None:
        cv2.putText(
            image,
            'ORANGE LINE NOT FOUND',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 0, 255),
            2,
        )

    else:
        contour = detection.contour.copy()
        contour[:, 0, 1] += detection.roi_top

        cv2.drawContours(
            image,
            [contour],
            -1,
            (0, 255, 0),
            2,
        )

        cv2.circle(
            image,
            detection.center,
            6,
            (0, 0, 255),
            -1,
        )

        cv2.putText(
            image,
            f'ORANGE error={detection.error:+.3f}',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

    return image
