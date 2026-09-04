"""OpenCV helpers used by the line detector node."""

from dataclasses import dataclass
from typing import Optional, Tuple

import cv2
import numpy as np


@dataclass
class LineDetection:
    """Result of detecting a line in the lower part of an image."""

    mask: np.ndarray
    roi_top: int
    contour: Optional[np.ndarray]
    center: Optional[Tuple[int, int]]
    error: Optional[float]
    area: float
    search_left: int = 0
    search_right: Optional[int] = None
    candidate_count: int = 0
    score: float = 0.0


def detect_line(
    frame: np.ndarray,
    roi_start_ratio: float = 0.55,
    threshold: int = 80,
    invert_threshold: bool = True,
    blur_kernel_size: int = 5,
    morph_kernel_size: int = 5,
    min_contour_area: float = 500.0,
    roi_left_ratio: float = 0.10,
    roi_right_ratio: float = 0.90,
    minimum_line_width_ratio: float = 0.015,
    maximum_line_width_ratio: float = 0.14,
    minimum_line_height_ratio: float = 0.20,
    expected_line_width_ratio: float = 0.06,
    previous_center_x: Optional[int] = None,
    maximum_center_jump_ratio: float = 0.20,
) -> LineDetection:
    """Find the best dark (or light) line candidate in the lower ROI.

    ``error`` is normalized to [-1, 1]. Negative values mean that the
    detected line is left of the image center, positive values mean right.
    """
    if frame is None or frame.size == 0:
        raise ValueError('frame must be a non-empty image')
    if frame.ndim not in (2, 3):
        raise ValueError('frame must be a grayscale or BGR image')

    height, width = frame.shape[:2]
    roi_start_ratio = float(np.clip(roi_start_ratio, 0.0, 0.99))
    roi_top = int(height * roi_start_ratio)
    roi = frame[roi_top:height, :]
    roi_height = height - roi_top

    roi_left_ratio = float(np.clip(roi_left_ratio, 0.0, 0.99))
    roi_right_ratio = float(np.clip(roi_right_ratio, 0.01, 1.0))
    if roi_left_ratio >= roi_right_ratio:
        raise ValueError('roi_left_ratio must be smaller than roi_right_ratio')
    search_left = int(width * roi_left_ratio)
    search_right = int(width * roi_right_ratio)

    minimum_line_width_ratio = float(minimum_line_width_ratio)
    maximum_line_width_ratio = float(maximum_line_width_ratio)
    minimum_line_height_ratio = float(minimum_line_height_ratio)
    expected_line_width_ratio = float(expected_line_width_ratio)
    maximum_center_jump_ratio = float(maximum_center_jump_ratio)
    if not 0.0 <= minimum_line_width_ratio < maximum_line_width_ratio:
        raise ValueError('line width ratios are invalid')
    if expected_line_width_ratio <= 0.0:
        raise ValueError('expected_line_width_ratio must be greater than zero')
    if not 0.0 <= minimum_line_height_ratio <= 1.0:
        raise ValueError('minimum_line_height_ratio must be between 0 and 1')
    if maximum_center_jump_ratio <= 0.0:
        raise ValueError('maximum_center_jump_ratio must be greater than zero')

    if roi.ndim == 3:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    else:
        gray = roi.copy()

    blur_kernel_size = _positive_odd(blur_kernel_size)
    if blur_kernel_size > 1:
        gray = cv2.GaussianBlur(
            gray,
            (blur_kernel_size, blur_kernel_size),
            0,
        )

    threshold_type = (
        cv2.THRESH_BINARY_INV
        if invert_threshold
        else cv2.THRESH_BINARY
    )
    _, mask = cv2.threshold(
        gray,
        int(np.clip(threshold, 0, 255)),
        255,
        threshold_type,
    )

    morph_kernel_size = max(1, int(morph_kernel_size))
    if morph_kernel_size > 1:
        kernel = np.ones(
            (morph_kernel_size, morph_kernel_size),
            dtype=np.uint8,
        )
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Ignore fixed vehicle structure and shadows close to the image edges.
    mask[:, :search_left] = 0
    mask[:, search_right:] = 0

    contours, _ = cv2.findContours(
        mask,
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE,
    )
    candidates = []
    for contour in contours:
        candidate = _evaluate_candidate(
            contour=contour,
            gray=gray,
            threshold=threshold,
            invert_threshold=invert_threshold,
            image_width=width,
            roi_height=roi_height,
            min_contour_area=min_contour_area,
            minimum_line_width_ratio=minimum_line_width_ratio,
            maximum_line_width_ratio=maximum_line_width_ratio,
            minimum_line_height_ratio=minimum_line_height_ratio,
            expected_line_width_ratio=expected_line_width_ratio,
            previous_center_x=previous_center_x,
            maximum_center_jump_ratio=maximum_center_jump_ratio,
        )
        if candidate is not None:
            candidates.append(candidate)

    if not candidates:
        return LineDetection(
            mask=mask,
            roi_top=roi_top,
            contour=None,
            center=None,
            error=None,
            area=0.0,
            search_left=search_left,
            search_right=search_right,
        )

    score, contour, center_x, center_y_roi, area = max(
        candidates,
        key=lambda candidate: candidate[0],
    )
    center_y = center_y_roi + roi_top
    half_width = max(width / 2.0, 1.0)
    error = float(np.clip((center_x - half_width) / half_width, -1.0, 1.0))

    return LineDetection(
        mask=mask,
        roi_top=roi_top,
        contour=contour,
        center=(center_x, center_y),
        error=error,
        area=area,
        search_left=search_left,
        search_right=search_right,
        candidate_count=len(candidates),
        score=score,
    )


def draw_detection(frame: np.ndarray, detection: LineDetection) -> np.ndarray:
    """Return an annotated copy of a line detection frame."""
    debug_frame = frame.copy()
    height, width = debug_frame.shape[:2]
    cv2.line(
        debug_frame,
        (0, detection.roi_top),
        (width - 1, detection.roi_top),
        (255, 0, 0),
        2,
    )
    search_right = (
        width - 1
        if detection.search_right is None
        else detection.search_right
    )
    cv2.line(
        debug_frame,
        (detection.search_left, detection.roi_top),
        (detection.search_left, height - 1),
        (255, 0, 255),
        2,
    )
    cv2.line(
        debug_frame,
        (search_right, detection.roi_top),
        (search_right, height - 1),
        (255, 0, 255),
        2,
    )
    cv2.line(
        debug_frame,
        (width // 2, detection.roi_top),
        (width // 2, height - 1),
        (0, 255, 255),
        2,
    )

    if detection.contour is None or detection.center is None:
        cv2.putText(
            debug_frame,
            'LINE NOT FOUND',
            (20, 40),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 0, 255),
            2,
        )
        return debug_frame

    contour = detection.contour.copy()
    contour[:, 0, 1] += detection.roi_top
    cv2.drawContours(debug_frame, [contour], -1, (0, 255, 0), 3)
    cv2.circle(debug_frame, detection.center, 8, (0, 0, 255), -1)
    cv2.line(
        debug_frame,
        (width // 2, detection.center[1]),
        detection.center,
        (255, 255, 0),
        2,
    )
    cv2.putText(
        debug_frame,
        'LINE '
        f'error={detection.error:+.3f} '
        f'area={detection.area:.0f} '
        f'score={detection.score:.2f} '
        f'candidates={detection.candidate_count}',
        (20, 40),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 0),
        2,
    )
    return debug_frame


def _positive_odd(value: int) -> int:
    value = max(1, int(value))
    return value if value % 2 == 1 else value + 1


def _evaluate_candidate(
    contour,
    gray,
    threshold,
    invert_threshold,
    image_width,
    roi_height,
    min_contour_area,
    minimum_line_width_ratio,
    maximum_line_width_ratio,
    minimum_line_height_ratio,
    expected_line_width_ratio,
    previous_center_x,
    maximum_center_jump_ratio,
):
    """Return a scored line candidate or None when geometry is invalid."""
    area = float(cv2.contourArea(contour))
    if area < float(min_contour_area):
        return None

    _, _, bounding_width, bounding_height = cv2.boundingRect(contour)
    width_ratio = bounding_width / max(float(image_width), 1.0)
    height_ratio = bounding_height / max(float(roi_height), 1.0)
    if not minimum_line_width_ratio <= width_ratio <= maximum_line_width_ratio:
        return None
    if height_ratio < minimum_line_height_ratio:
        return None

    moments = cv2.moments(contour)
    if moments['m00'] == 0.0:
        return None
    center_x = int(moments['m10'] / moments['m00'])
    center_y = int(moments['m01'] / moments['m00'])

    if previous_center_x is None:
        reference_x = image_width / 2.0
        maximum_distance = max(image_width / 2.0, 1.0)
    else:
        reference_x = float(previous_center_x)
        maximum_distance = max(
            image_width * maximum_center_jump_ratio,
            1.0,
        )
        if abs(center_x - reference_x) > maximum_distance:
            return None

    position_score = 1.0 - min(
        abs(center_x - reference_x) / maximum_distance,
        1.0,
    )
    width_score = 1.0 - min(
        abs(width_ratio - expected_line_width_ratio)
        / expected_line_width_ratio,
        1.0,
    )
    height_score = min(height_ratio / 0.70, 1.0)

    contour_mask = np.zeros(gray.shape, dtype=np.uint8)
    cv2.drawContours(contour_mask, [contour], -1, 255, -1)
    mean_intensity = cv2.mean(gray, mask=contour_mask)[0]
    if invert_threshold:
        contrast_score = (
            float(threshold) - mean_intensity
        ) / max(float(threshold), 1.0)
    else:
        contrast_score = (
            mean_intensity - float(threshold)
        ) / max(255.0 - float(threshold), 1.0)
    contrast_score = float(np.clip(contrast_score, 0.0, 1.0))

    score = (
        0.45 * position_score
        + 0.30 * contrast_score
        + 0.15 * width_score
        + 0.10 * height_score
    )
    return score, contour, center_x, center_y, area
