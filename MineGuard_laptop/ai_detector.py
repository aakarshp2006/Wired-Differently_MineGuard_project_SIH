"""
ai_detector.py
----------------
MineGuard - Smart Mine Vehicle Safety & Monitoring System
AI Vision Module (YOLO Object Detection + Fog Fusion)

Pipeline this module implements:

    AI Thinker ESP32-CAM (OV2640)
            |  Wi-Fi MJPEG stream
            v
    Python OpenCV frame
            |
            +--> YOLO Object Detection  --\
            |                              +--> Clean structured output
            +--> Fog Detection Module    --/
            v
    (consumed later by Risk Engine + Integration Controller)

This module ONLY produces vision information. It does NOT calculate
risk, does NOT send motor/robot commands, and does NOT run any web
server or dashboard. A downstream module is expected to do:

    vision_result = vision.process_frame(frame)
    risk_result = calculate_risk(
        vision_result["object"],
        ultrasonic_distance,
        vehicle_speed,
    )

------------------------------------------------------------------
SYSTEM HEALTH REPORTING
------------------------------------------------------------------
Every result now reports three independent health flags so the
integration controller can tell the difference between "everything
is fine" and "a sensor/subsystem failed" instead of silently seeing
misleadingly optimistic numbers:

    camera_status : "OK"    -> the incoming frame was valid and usable
                    "ERROR" -> frame was missing/corrupted/empty

    ai_status     : "OK"    -> YOLO ran successfully on a valid frame
                    "ERROR" -> YOLO inference raised an exception,
                               or the frame itself was invalid

    fog_status    : "OK"    -> fog_detection.py produced a real reading
                    "ERROR" -> fog estimation failed or was skipped
                               because the frame was invalid

CAMERA FAILURE IS NEVER REPORTED AS CLEAR WEATHER. If camera_status
is "ERROR", fog_percentage/visibility_score are always None and
fog_level is "UNKNOWN" -- never a fabricated "CLEAR, 100% visibility".

The dictionary returned by process_frame() always has the exact same
set of keys, in every situation (valid frame, invalid frame, YOLO
failure, fog failure). Only the values change. This is deliberate so
the integration controller can index into the result the same way
every time without extra branching.
"""

from typing import Optional, List, Dict, Any

import cv2
import numpy as np
from ultralytics import YOLO

from fog_detection import estimate_fog

# ======================================================================
# CONFIGURATION
# ======================================================================

# URL of the MJPEG stream served by esp32_cam_stream.ino (see that
# module's Serial Monitor output for the exact IP to use here).
ESP32_CAM_URL = "http://192.168.4.2/stream"

# YOLO model file. "yolo11n.pt" (nano) is chosen for real-time speed
# on a laptop CPU; swap for a larger variant if a GPU is available
# and higher accuracy is needed.
YOLO_MODEL = "yolo11n.pt"

# Minimum YOLO confidence (0-1) for a detection to be considered at all.
CONFIDENCE_THRESHOLD = 0.50

# How often (in processed frames) the test loop prints a status line.
PRINT_EVERY_N_FRAMES = 30

# ----------------------------------------------------------------------
# COCO class -> MineGuard safety class mapping
# ----------------------------------------------------------------------
# The Risk Engine only understands "person", "vehicle", "obstacle".
# YOLO/COCO reports dozens of classes, so we translate here.

PERSON_CLASSES = {"person"}

VEHICLE_CLASSES = {"car", "truck", "bus", "motorcycle", "bicycle"}

# Classes that are physically meaningful near a mine vehicle but not
# people or vehicles specifically -> generic "obstacle".
# (Not exhaustive by design: anything YOLO detects that is NOT in
# PERSON_CLASSES, VEHICLE_CLASSES, or IGNORED_CLASSES below falls
# through to "obstacle" automatically -- see _map_class().)
OBSTACLE_CLASSES = {
    "dog", "cat", "cow", "horse", "sheep", "bird",
    "traffic light", "fire hydrant", "stop sign", "parking meter",
    "bench", "backpack", "suitcase", "chair",
}

# Everyday indoor/desk objects that YOLO may pick up but that have no
# bearing on mine vehicle safety -- explicitly ignored so they never
# get promoted to "obstacle" and never distract the Risk Engine.
IGNORED_CLASSES = {
    "keyboard", "laptop", "cell phone", "book", "spoon", "fork", "knife",
    "cup", "bowl", "bottle", "wine glass", "mouse", "remote", "tv",
    "microwave", "oven", "toaster", "sink", "refrigerator", "clock",
    "vase", "scissors", "teddy bear", "hair drier", "toothbrush",
    "handbag", "tie", "umbrella", "frisbee", "skis", "snowboard",
    "sports ball", "kite", "baseball bat", "baseball glove", "skateboard",
    "surfboard", "tennis racket", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake",
    "dining table", "toilet", "potted plant", "couch", "bed",
}

# Relative priority of each MineGuard class when selecting the
# PRIMARY hazard among several detections in the same frame.
# People get top priority (highest safety risk), then vehicles,
# then generic obstacles.
CLASS_PRIORITY = {
    "person": 1.0,
    "vehicle": 0.85,
    "obstacle": 0.6,
}

# Weights for the primary-hazard scoring formula (see
# _calculate_hazard_score() for the full explanation).
WEIGHT_CLASS_PRIORITY = 0.40
WEIGHT_CONFIDENCE = 0.35
WEIGHT_CENTER_PROXIMITY = 0.25

# Colors (BGR) used by annotate_frame().
COLOR_NORMAL_BOX = (0, 200, 0)      # green - ordinary detection
COLOR_PRIMARY_BOX = (0, 0, 255)     # red   - selected primary hazard
COLOR_TEXT_BG = (0, 0, 0)


# ======================================================================
# MineGuardVision
# ======================================================================

class MineGuardVision:
    """
    Wraps a YOLO model + the fog detection heuristic into one clean
    per-frame vision analysis step for MineGuard, with explicit
    camera/AI/fog health reporting on every call.
    """

    def __init__(self, model_path: str = YOLO_MODEL,
                 confidence_threshold: float = CONFIDENCE_THRESHOLD):
        """
        Load the YOLO model once at startup.

        Raises:
            RuntimeError: if the model file cannot be loaded, so the
            failure is caught immediately at setup time rather than
            silently breaking every later process_frame() call.
        """
        self.confidence_threshold = confidence_threshold
        self.model_path = model_path

        try:
            self.model = YOLO(model_path)
            print(f"[ai_detector] YOLO model loaded successfully: {model_path}")
        except Exception as exc:
            # Fail fast and loud: without a model this class is useless.
            raise RuntimeError(
                f"[ai_detector] Failed to load YOLO model '{model_path}': {exc}"
            ) from exc

    # ------------------------------------------------------------------
    # Internal helpers - validation / mapping / scoring
    # ------------------------------------------------------------------

    @staticmethod
    def _is_valid_frame(frame) -> bool:
        """Basic sanity check on an incoming OpenCV frame."""
        return (
            frame is not None
            and isinstance(frame, np.ndarray)
            and frame.size > 0
            and frame.ndim == 3
            and frame.shape[2] == 3
        )

    @staticmethod
    def _map_class(original_class: str) -> Optional[str]:
        """
        Translate a raw YOLO/COCO class name into a MineGuard safety
        class, or return None if the object should be ignored entirely.
        """
        name = original_class.lower().strip()

        if name in IGNORED_CLASSES:
            return None
        if name in PERSON_CLASSES:
            return "person"
        if name in VEHICLE_CLASSES:
            return "vehicle"
        if name in OBSTACLE_CLASSES:
            return "obstacle"

        # Anything else YOLO reports that we haven't explicitly
        # classified: treat conservatively as a generic obstacle
        # rather than silently dropping a potentially real hazard.
        return "obstacle"

    @staticmethod
    def _calculate_center_score(bbox, frame_width: int, frame_height: int) -> float:
        """
        Score (0-1) for how close a detection's bounding-box center is
        to the center of the frame. 1.0 = dead center, 0.0 = at the
        farthest possible edge.

        Rationale: the ultrasonic sensor looks straight ahead, so an
        object dead-center in the camera view is the one most likely
        to correspond to what the ultrasonic sensor is actually
        measuring. Off-center detections are visually present but less
        relevant to the immediate forward collision path.
        """
        x1, y1, x2, y2 = bbox
        box_cx = (x1 + x2) / 2.0
        box_cy = (y1 + y2) / 2.0

        frame_cx = frame_width / 2.0
        frame_cy = frame_height / 2.0

        # Normalize by the distance from center to a corner, so the
        # score is resolution-independent.
        max_dist = ((frame_cx ** 2) + (frame_cy ** 2)) ** 0.5
        max_dist = max(max_dist, 1e-6)  # avoid division by zero

        dist = ((box_cx - frame_cx) ** 2 + (box_cy - frame_cy) ** 2) ** 0.5
        score = 1.0 - (dist / max_dist)
        return max(0.0, min(1.0, score))

    @staticmethod
    def _calculate_hazard_score(mapped_class: str, confidence: float,
                                 center_score: float) -> float:
        """
        Combine safety-class priority, detection confidence, and center
        proximity into a single 0-1 "how much should this detection be
        treated as THE hazard right now" score.

        Weighting rationale:
          - Class priority (40%): a person is always more critical to
            react to than an obstacle of the same confidence/position.
          - Confidence (35%): a detection YOLO is more certain about
            should be trusted more when choosing between candidates.
          - Center proximity (25%): objects directly ahead matter more
            for collision purposes than objects off to the side, but
            this should not override a clearly higher-priority class.
        """
        priority = CLASS_PRIORITY.get(mapped_class, 0.5)
        return (
            priority * WEIGHT_CLASS_PRIORITY
            + confidence * WEIGHT_CONFIDENCE
            + center_score * WEIGHT_CENTER_PROXIMITY
        )

    # ------------------------------------------------------------------
    # Internal helpers - subsystem runners (each isolates its own failures)
    # ------------------------------------------------------------------

    def _run_yolo(self, frame, frame_width: int, frame_height: int):
        """
        Run YOLO on a KNOWN-VALID frame and build the filtered/mapped
        detection list.

        Returns:
            (ai_status, detections)
            ai_status   : "OK" or "ERROR"
            detections  : list of detection dicts (each still carries
                          an internal "_hazard_score" used only for
                          primary-hazard selection). Empty list on
                          failure or when nothing relevant was found.
        """
        try:
            yolo_results = self.model(frame, conf=self.confidence_threshold, verbose=False)
        except Exception as exc:
            print(f"[ai_detector] WARNING: YOLO inference failed: {exc}")
            return "ERROR", []

        detections: List[Dict[str, Any]] = []

        try:
            if len(yolo_results) > 0 and yolo_results[0].boxes is not None:
                boxes = yolo_results[0].boxes
                class_names = yolo_results[0].names  # {class_id: class_name}

                for box in boxes:
                    class_id = int(box.cls[0])
                    original_class = class_names.get(class_id, str(class_id))
                    confidence = float(box.conf[0])  # 0-1

                    if confidence < self.confidence_threshold:
                        continue  # extra safety net beyond the model's own conf filter

                    mapped_class = self._map_class(original_class)
                    if mapped_class is None:
                        continue  # irrelevant object (keyboard, spoon, etc.)

                    x1, y1, x2, y2 = [float(v) for v in box.xyxy[0]]
                    center_score = self._calculate_center_score(
                        (x1, y1, x2, y2), frame_width, frame_height
                    )

                    detections.append({
                        "object": mapped_class,
                        "original_class": original_class,
                        "confidence": round(confidence * 100, 2),
                        "bbox": [round(x1, 1), round(y1, 1), round(x2, 1), round(y2, 1)],
                        "center_score": round(center_score, 2),
                        # kept only for internal primary-hazard selection,
                        # not part of the public schema:
                        "_hazard_score": self._calculate_hazard_score(
                            mapped_class, confidence, center_score
                        ),
                    })
        except Exception as exc:
            # Inference itself succeeded but result parsing broke
            # (e.g. an unexpected Ultralytics API change). Treat this
            # the same as an AI failure rather than crashing.
            print(f"[ai_detector] WARNING: failed to parse YOLO results: {exc}")
            return "ERROR", []

        return "OK", detections

    @staticmethod
    def _run_fog_detection(frame):
        """
        Run fog estimation on a KNOWN-VALID frame, isolating any
        unexpected failure inside fog_detection.py so it can never
        crash the vision pipeline.

        Returns:
            (fog_status, fog_percentage, fog_level, visibility_score)
        """
        try:
            fog_data = estimate_fog(frame)
            return (
                "OK",
                fog_data["fog_percentage"],
                fog_data["fog_level"],
                fog_data["visibility_score"],
            )
        except Exception as exc:
            print(f"[ai_detector] WARNING: fog detection failed: {exc}")
            return "ERROR", None, "UNKNOWN", None

    @staticmethod
    def _build_result(camera_status: str, ai_status: str, fog_status: str,
                       object_: str, confidence: float,
                       fog_percentage, fog_level: str, visibility_score,
                       detections: List[Dict[str, Any]]) -> Dict[str, Any]:
        """
        Single source of truth for the output dictionary shape. EVERY
        return path in process_frame() goes through this function so
        the schema (key set + key order) never varies between the
        success and failure cases.
        """
        return {
            "camera_status": camera_status,
            "ai_status": ai_status,
            "fog_status": fog_status,
            "object": object_,
            "confidence": confidence,
            "fog_percentage": fog_percentage,
            "fog_level": fog_level,
            "visibility_score": visibility_score,
            "detections": detections,
        }

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def process_frame(self, frame) -> Dict[str, Any]:
        """
        Analyze one BGR frame: run YOLO, map + filter detections,
        select the primary hazard, run fog estimation, and return a
        single clean structured result -- with explicit camera/AI/fog
        health flags in every case, valid or not.

        Returns:
            dict: see module docstring for the exact output schema.
        """
        # 1. Validate frame ----------------------------------------------
        # CAMERA FAILURE IS NEVER REPORTED AS "CLEAR, 100% VISIBILITY".
        # An invalid frame means we genuinely know nothing about the
        # scene, so every downstream field reflects that honestly.
        if not self._is_valid_frame(frame):
            print("[ai_detector] WARNING: invalid/empty frame received, skipping.")
            return self._build_result(
                camera_status="ERROR",
                ai_status="ERROR",
                fog_status="ERROR",
                object_="none",
                confidence=0.0,
                fog_percentage=None,
                fog_level="UNKNOWN",
                visibility_score=None,
                detections=[],
            )

        frame_height, frame_width = frame.shape[:2]

        # 2-6. Run YOLO (own failure isolation) ---------------------------
        ai_status, detections = self._run_yolo(frame, frame_width, frame_height)

        # 8. Fog estimation (own failure isolation) ------------------------
        # Always attempted on a valid frame, even if YOLO itself failed --
        # a working fog sensor is still useful information on its own.
        fog_status, fog_percentage, fog_level, visibility_score = self._run_fog_detection(frame)

        # 7. Select primary hazard -----------------------------------------
        if not detections:
            primary_object = "none"
            primary_confidence = 0.0
            public_detections: List[Dict[str, Any]] = []
        else:
            primary = max(detections, key=lambda d: d["_hazard_score"])
            primary_object = primary["object"]
            primary_confidence = primary["confidence"]
            # Strip the internal scoring field before returning to callers.
            public_detections = [
                {k: v for k, v in d.items() if k != "_hazard_score"}
                for d in detections
            ]

        # 9. Return clean, consistently-shaped structured data --------------
        return self._build_result(
            camera_status="OK",
            ai_status=ai_status,
            fog_status=fog_status,
            object_=primary_object,
            confidence=primary_confidence,
            fog_percentage=fog_percentage,
            fog_level=fog_level,
            visibility_score=visibility_score,
            detections=public_detections,
        )

    def annotate_frame(self, frame, result: Dict[str, Any]):
        """
        Draw bounding boxes + labels for every detection, highlight the
        primary hazard, and overlay fog + system health information at
        the top of the frame. Returns the annotated frame (drawn on a
        copy). Safely handles None fog values (camera/AI/fog failures).
        """
        if not self._is_valid_frame(frame):
            return frame

        annotated = frame.copy()
        detections = result.get("detections", [])

        # Identify which detection (if any) is the primary hazard by
        # matching it against the top-level object/confidence the
        # scoring step already selected.
        primary_object = result.get("object")
        primary_confidence = result.get("confidence")

        primary_matched = False
        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox"]]

            is_primary = (
                not primary_matched
                and det["object"] == primary_object
                and det["confidence"] == primary_confidence
            )
            if is_primary:
                primary_matched = True

            color = COLOR_PRIMARY_BOX if is_primary else COLOR_NORMAL_BOX
            thickness = 3 if is_primary else 2

            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, thickness)

            label = f"{det['original_class']}->{det['object']} {det['confidence']}%"
            if is_primary:
                label = "PRIMARY HAZARD: " + label

            label_y = max(y1 - 8, 15)
            cv2.putText(
                annotated, label, (x1, label_y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2
            )

        # ---- Top status banner: system health + fog reading ----------
        camera_status = result.get("camera_status", "ERROR")
        ai_status = result.get("ai_status", "ERROR")
        fog_status = result.get("fog_status", "ERROR")

        fog_percentage = result.get("fog_percentage")
        fog_level = result.get("fog_level", "UNKNOWN")
        visibility_score = result.get("visibility_score")

        # Never try to format None as a number -- show UNKNOWN instead.
        if fog_percentage is None or visibility_score is None:
            fog_text = f"Fog: UNKNOWN  ({fog_level})"
        else:
            fog_text = (
                f"Fog: {fog_percentage}%  ({fog_level})  "
                f"Visibility: {visibility_score}"
            )

        status_text = (
            f"Camera: {camera_status}   AI: {ai_status}   Fog Sensor: {fog_status}"
        )

        banner_height = 50
        cv2.rectangle(annotated, (0, 0), (annotated.shape[1], banner_height), COLOR_TEXT_BG, -1)

        cv2.putText(
            annotated, status_text, (8, 18),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
            (255, 255, 255), 1
        )
        cv2.putText(
            annotated, fog_text, (8, 40),
            cv2.FONT_HERSHEY_SIMPLEX, 0.55,
            (255, 255, 255), 1
        )

        return annotated


# ======================================================================
# CAMERA TEST MODE
# ======================================================================
# Importing this module does NOT open the ESP32-CAM stream or load the
# model on its own -- both only happen when this file is run directly.

if __name__ == "__main__":
    print("MineGuard AI Detector - camera test mode")
    print(f"Connecting to ESP32-CAM stream: {ESP32_CAM_URL}")
    print("Press 'q' in the video window to quit.\n")

    # --- Load the model up front so a bad model path fails immediately ---
    try:
        vision = MineGuardVision(model_path=YOLO_MODEL,
                                  confidence_threshold=CONFIDENCE_THRESHOLD)
    except RuntimeError as exc:
        print(exc)
        raise SystemExit(1)

    # --- Open the ESP32-CAM MJPEG stream ---
    cap = cv2.VideoCapture(ESP32_CAM_URL)

    if not cap.isOpened():
        print(f"[ai_detector] ERROR: could not open stream at {ESP32_CAM_URL}")
        print("Check that the ESP32-CAM is powered on, connected to Wi-Fi, "
              "and that ESP32_CAM_URL matches its printed IP address.")
        raise SystemExit(1)

    frame_count = 0

    try:
        while True:
            ret, frame = cap.read()

            if not ret or frame is None:
                print("[ai_detector] WARNING: failed to read frame from stream "
                      "(camera disconnected or network issue). Retrying...")
                # Still run it through process_frame() so the health
                # flags reflect the failure instead of just skipping
                # the loop iteration silently.
                result = vision.process_frame(frame)
                frame_count += 1
                if frame_count % PRINT_EVERY_N_FRAMES == 0:
                    print(
                        f"[frame {frame_count}] camera={result['camera_status']} "
                        f"ai={result['ai_status']} fog={result['fog_status']}"
                    )
                continue

            result = vision.process_frame(frame)
            annotated = vision.annotate_frame(frame, result)

            cv2.imshow("MineGuard AI Detector", annotated)

            frame_count += 1
            if frame_count % PRINT_EVERY_N_FRAMES == 0:
                print(
                    f"[frame {frame_count}] "
                    f"camera={result['camera_status']} ai={result['ai_status']} "
                    f"fog={result['fog_status']} "
                    f"primary={result['object']} conf={result['confidence']}% "
                    f"fog%={result['fog_percentage']} level={result['fog_level']} "
                    f"visibility={result['visibility_score']} "
                    f"detections={len(result['detections'])}"
                )

            if cv2.waitKey(1) & 0xFF == ord('q'):
                print("[ai_detector] 'q' pressed, shutting down.")
                break

    except KeyboardInterrupt:
        print("\n[ai_detector] Interrupted by user.")

    finally:
        cap.release()
        cv2.destroyAllWindows()
        print("[ai_detector] Camera released and windows closed cleanly.")
