"""
fog_detection.py
------------------
MineGuard - Smart Mine Vehicle Safety & Monitoring System
Fog / Visibility Estimation Module

Purpose
-------
Estimate a rough fog/visibility level from a single BGR camera frame
(e.g. a frame pulled from the ESP32-CAM MJPEG stream).

This is a PROTOTYPE heuristic, not a scientific meteorological
instrument. It combines a few classic, lightweight image-quality
signals that all tend to move together when haze/fog is present:

  - Sharpness (Laplacian variance)
        Fog scatters light and softens edges, so a hazy frame has
        far less high-frequency detail than a clear one.

  - Contrast (grayscale standard deviation)
        Fog compresses the range of light and dark values toward a
        uniform grey, lowering overall contrast.

  - Saturation (mean saturation in HSV)
        Fog scatters white/grey light over the scene, which visibly
        washes out color saturation.

None of these three signals alone is reliable (e.g. a plain grey
wall looks "hazy" by contrast/sharpness alone), but combined with
sensible weights they give a stable, explainable estimate that is
cheap enough to run on every frame in real time.

This module ONLY analyzes a frame that is handed to it. It never
opens a camera, a video file, or a network stream itself.

Usage
-----
    from fog_detection import estimate_fog

    fog_result = estimate_fog(frame)   # frame = OpenCV BGR numpy array
    print(fog_result)
    # {'fog_percentage': 42.3, 'fog_level': 'MODERATE_FOG', 'visibility_score': 57.7}
"""

import cv2
import numpy as np

# --------------------------------------------------------------------
# CONFIGURATION
# --------------------------------------------------------------------

# Reference ("clear day") values used to normalize each raw signal
# into a 0-100 range. These are empirical, tuned for a typical
# webcam / ESP32-CAM feed — not physical constants. Adjust here if
# the prototype's camera/lighting conditions need re-calibration.
REF_MAX_LAPLACIAN_VAR = 1200.0   # sharp, in-focus outdoor scene
REF_MAX_CONTRAST_STD = 60.0      # grayscale std-dev of a clear scene
REF_MAX_SATURATION = 100.0       # mean HSV saturation of a clear scene

# Weights for combining the three sub-scores into one fog score.
# Sharpness is weighted highest because edge/detail loss is the most
# direct and reliable visual symptom of fog.
WEIGHT_SHARPNESS = 0.40
WEIGHT_CONTRAST = 0.35
WEIGHT_SATURATION = 0.25

# Fog level classification bands (on fog_percentage, 0-100).
CLEAR_MAX = 20
LIGHT_FOG_MAX = 40
MODERATE_FOG_MAX = 70
# anything above MODERATE_FOG_MAX -> DENSE_FOG

# Small epsilon to guard against division by zero.
EPSILON = 1e-6


# --------------------------------------------------------------------
# HELPER FUNCTIONS
# --------------------------------------------------------------------

def _is_valid_frame(frame):
    """Return True if 'frame' looks like a usable BGR image array."""
    if frame is None:
        return False
    if not isinstance(frame, np.ndarray):
        return False
    if frame.size == 0:
        return False
    if frame.ndim != 3 or frame.shape[2] != 3:
        return False
    return True


def _clamp(value, low=0.0, high=100.0):
    """Clamp a numeric value into [low, high]."""
    return max(low, min(high, value))


def _calculate_sharpness_score(gray):
    """
    Lower sharpness (Laplacian variance) => more fog.
    Returns a 0-100 'fog contribution' score (100 = very hazy/blurry).
    """
    laplacian_var = cv2.Laplacian(gray, cv2.CV_64F).var()
    clarity_ratio = laplacian_var / (REF_MAX_LAPLACIAN_VAR + EPSILON)
    clarity_ratio = _clamp(clarity_ratio, 0.0, 1.0)
    return 100.0 * (1.0 - clarity_ratio)


def _calculate_contrast_score(gray):
    """
    Lower contrast (grayscale std-dev) => more fog.
    Returns a 0-100 'fog contribution' score.
    """
    contrast_std = float(np.std(gray))
    contrast_ratio = contrast_std / (REF_MAX_CONTRAST_STD + EPSILON)
    contrast_ratio = _clamp(contrast_ratio, 0.0, 1.0)
    return 100.0 * (1.0 - contrast_ratio)


def _calculate_saturation_score(hsv):
    """
    Lower mean saturation => more fog (colors washed out).
    Returns a 0-100 'fog contribution' score.
    """
    mean_saturation = float(np.mean(hsv[:, :, 1]))
    saturation_ratio = mean_saturation / (REF_MAX_SATURATION + EPSILON)
    saturation_ratio = _clamp(saturation_ratio, 0.0, 1.0)
    return 100.0 * (1.0 - saturation_ratio)


def _classify_fog_level(fog_percentage):
    """Map a 0-100 fog_percentage into one of the four fog levels."""
    if fog_percentage <= CLEAR_MAX:
        return "CLEAR"
    if fog_percentage <= LIGHT_FOG_MAX:
        return "LIGHT_FOG"
    if fog_percentage <= MODERATE_FOG_MAX:
        return "MODERATE_FOG"
    return "DENSE_FOG"


def _safe_fallback_result():
    """
    Result returned for invalid/empty frames.

    We deliberately do NOT raise an exception here: this module feeds
    a real-time pipeline (ai_detector.py / integration_controller.py)
    where a single bad/dropped frame should never crash the loop.
    Reporting "CLEAR" with full visibility is the safest default,
    since it does not force an unnecessary speed reduction based on
    a frame we couldn't actually analyze.
    """
    return {
        "fog_percentage": 0.0,
        "fog_level": "CLEAR",
        "visibility_score": 100.0,
    }


# --------------------------------------------------------------------
# MAIN PUBLIC FUNCTION
# --------------------------------------------------------------------

def estimate_fog(frame):
    """
    Estimate fog/visibility conditions from a single BGR frame.

    Args:
        frame (numpy.ndarray): OpenCV BGR image (H x W x 3).

    Returns:
        dict: {
            "fog_percentage": float (0-100, higher = more fog),
            "fog_level": str ("CLEAR" / "LIGHT_FOG" / "MODERATE_FOG" / "DENSE_FOG"),
            "visibility_score": float (0-100, higher = clearer)
        }
    """
    if not _is_valid_frame(frame):
        print("[fog_detection] WARNING: invalid or empty frame received, "
              "returning safe fallback result.")
        return _safe_fallback_result()

    # Convert once, reuse for all sub-signals (keeps this cheap enough
    # for real-time per-frame use).
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    sharpness_score = _calculate_sharpness_score(gray)
    contrast_score = _calculate_contrast_score(gray)
    saturation_score = _calculate_saturation_score(hsv)

    fog_percentage = (
        sharpness_score * WEIGHT_SHARPNESS +
        contrast_score * WEIGHT_CONTRAST +
        saturation_score * WEIGHT_SATURATION
    )
    fog_percentage = round(float(_clamp(fog_percentage)), 2)

    visibility_score = round(float(_clamp(100.0 - fog_percentage)), 2)
    fog_level = _classify_fog_level(fog_percentage)

    return {
        "fog_percentage": fog_percentage,
        "fog_level": fog_level,
        "visibility_score": visibility_score,
    }


# --------------------------------------------------------------------
# MANUAL TEST SECTION
# Importing this module does NOT open a camera. A camera/image is
# only opened when this file is run directly.
# --------------------------------------------------------------------

if __name__ == "__main__":
    print("MineGuard Fog Detection - manual test")
    print("Press 'q' in the video window to quit.\n")

    cap = cv2.VideoCapture(0)  # local webcam for standalone testing

    if not cap.isOpened():
        print("[fog_detection] ERROR: could not open webcam (index 0). "
              "Connect a webcam or adapt this test block to load a "
              "static image instead, e.g.:\n"
              "    frame = cv2.imread('sample.jpg')\n"
              "    print(estimate_fog(frame))")
    else:
        try:
            while True:
                ret, frame = cap.read()
                if not ret:
                    print("[fog_detection] WARNING: failed to read frame, stopping.")
                    break

                result = estimate_fog(frame)

                # Overlay the result on the preview window for quick visual checks.
                overlay_text = (
                    f"Fog: {result['fog_percentage']}%  "
                    f"Level: {result['fog_level']}  "
                    f"Visibility: {result['visibility_score']}"
                )
                cv2.putText(
                    frame, overlay_text, (10, 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2
                )

                print(result)
                cv2.imshow("MineGuard Fog Detection Test", frame)

                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
        finally:
            cap.release()
            cv2.destroyAllWindows()
