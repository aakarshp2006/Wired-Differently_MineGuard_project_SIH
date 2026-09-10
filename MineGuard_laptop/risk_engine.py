"""
risk_engine.py
----------------
MineGuard - Smart Mine Vehicle Safety System
Risk Engine Module

Live inputs (matches the "Risk engine inputs -> output" concept —
4 of the 6 planned inputs are wired up today; "Relative movement" and
"Road geometry" are future work, see the note at the bottom):
  1. object_type    <- YOLO object detection   ("person" / "vehicle" / "obstacle")
  2. distance        <- HC-SR04 ultrasonic sensor (metres)
  3. vehicle_speed    <- LM393 encoder via ESP32   (metres/second)
  4. fog_level        <- YOLO/camera fog estimation (0-100 %), stands
                          in for "Visibility"

The engine is a HYBRID system:
  - Under normal conditions it produces a smooth, weighted 0-100 risk
    score instead of jumping abruptly between fixed thresholds.
  - Under critical conditions (object inside the danger distance,
    collision imminent, OR visibility too low to trust the camera)
    a hard safety override forces CRITICAL/STOP, regardless of what
    the weighted score would otherwise say.

Output is a 4-tier classification -- SAFE / CAUTION / HIGH RISK /
CRITICAL -- plus a risk score and TTC (Time-To-Collision), matching
the "Risk Engine -> Risk Score + TTC -> SAFE/CAUTION/HIGH RISK/CRITICAL"
flow. TTC itself follows the same conceptual formula everywhere in
this project: TTC = Distance / Closing Speed (closing speed here is
approximated as the vehicle's own speed, since a relative/closing
speed sensor isn't wired up yet -- see the note at the bottom).

Note on fog:
  - Distance and speed come from the ultrasonic sensor / encoder, so
    fog does NOT affect those readings directly.
  - Fog DOES affect how much we can trust the camera's object
    detection. So fog is folded in two ways:
      1. As a weighted risk factor (fog_risk) in the combined score,
         so heavier fog gradually pushes risk up even if the object
         looks far away.
      2. As a hard cutoff: if fog_level > FOG_CRITICAL_PERCENT, the
         camera data is considered unreliable and the engine forces
         CRITICAL / "OPERATION NOT RECOMMENDED", regardless of object
         type, distance or speed.

Pure standard library. No Flask, no GUI, no external dependencies.
Safe to import into any other integration module.
"""

import math

# --------------------------------------------------------------------
# CONFIGURATION
# --------------------------------------------------------------------

# Object-specific safety distances (metres).
# NOTE: scaled DOWN for prototype/desk-scale testing (small robot,
# limited room space) -- these are placeholder values, not real
# mine-safety distances. Before any real deployment, scale these back
# up to proper industrial distances (person: a few metres, vehicle:
# several metres, based on actual stopping distance calculations).
SAFETY_LIMITS = {
    "person":   {"danger": 0.3, "warning": 1.0},
    "vehicle":  {"danger": 0.5, "warning": 1.2},
    "obstacle": {"danger": 0.3, "warning": 0.8},
}

# TTC thresholds (seconds).
TTC_DANGER_SECONDS = 2.0
TTC_WARNING_SECONDS = 5.0

# Fog thresholds (percent, 0-100).
# Below FOG_WARNING_PERCENT      -> fog_risk contribution is 0
# Between WARNING and CRITICAL   -> fog_risk scales 0 -> 100
# At or above FOG_CRITICAL_PERCENT -> hard cutoff, camera untrustworthy
FOG_WARNING_PERCENT = 40.0
FOG_CRITICAL_PERCENT = 90.0

# Weights used in the weighted risk fusion. Must sum to 1.0.
DISTANCE_WEIGHT = 0.5
TTC_WEIGHT = 0.3
FOG_WEIGHT = 0.2

# Classification bands (used only when no critical override applies).
# 4-tier: SAFE -> CAUTION -> HIGH RISK -> CRITICAL
CRITICAL_THRESHOLD = 85
HIGH_RISK_THRESHOLD = 60
CAUTION_THRESHOLD = 30


# --------------------------------------------------------------------
# HELPER FUNCTIONS
# --------------------------------------------------------------------

def validate_inputs(object_type, distance, vehicle_speed, fog_level):
    """
    Validate raw sensor inputs and normalize object_type.
    Raises ValueError on negative distance/speed or an out-of-range
    fog_level. Unknown object labels safely fall back to "obstacle".
    """
    if distance < 0:
        raise ValueError("distance cannot be negative")
    if vehicle_speed < 0:
        raise ValueError("vehicle_speed cannot be negative")
    if not (0.0 <= fog_level <= 100.0):
        raise ValueError("fog_level must be between 0 and 100")

    normalized_type = str(object_type).strip().lower()

    if normalized_type not in SAFETY_LIMITS:
        normalized_type = "obstacle"

    return normalized_type


def calculate_ttc(distance, vehicle_speed):
    """Time-to-collision in seconds. Infinity if speed is 0 or less."""
    if vehicle_speed <= 0:
        return math.inf
    return distance / vehicle_speed


def clamp(value, low=0.0, high=100.0):
    """Clamp a numeric value into [low, high]."""
    return max(low, min(high, value))


def calculate_distance_risk(distance, danger_distance, warning_distance):
    """
    0-100 score based on how close the object is.
    100  -> at or inside danger_distance
    0    -> at or beyond warning_distance
    Linearly interpolated in between.
    """
    if distance <= danger_distance:
        return 100.0
    if distance >= warning_distance:
        return 0.0

    span = warning_distance - danger_distance
    score = 100.0 * (warning_distance - distance) / span
    return clamp(score)


def calculate_ttc_risk(ttc):
    """
    0-100 score based on time-to-collision.
    100  -> ttc <= TTC_DANGER_SECONDS
    0    -> ttc >= TTC_WARNING_SECONDS (or infinite, i.e. stationary vehicle)
    Linearly interpolated in between.
    """
    if ttc == math.inf:
        return 0.0
    if ttc <= TTC_DANGER_SECONDS:
        return 100.0
    if ttc >= TTC_WARNING_SECONDS:
        return 0.0

    span = TTC_WARNING_SECONDS - TTC_DANGER_SECONDS
    score = 100.0 * (TTC_WARNING_SECONDS - ttc) / span
    return clamp(score)


def calculate_fog_risk(fog_level):
    """
    0-100 score based on how much fog is reducing camera reliability.
    0    -> at or below FOG_WARNING_PERCENT (fog not a concern yet)
    100  -> at or above FOG_CRITICAL_PERCENT (camera untrustworthy)
    Linearly interpolated in between.
    """
    if fog_level <= FOG_WARNING_PERCENT:
        return 0.0
    if fog_level >= FOG_CRITICAL_PERCENT:
        return 100.0

    span = FOG_CRITICAL_PERCENT - FOG_WARNING_PERCENT
    score = 100.0 * (fog_level - FOG_WARNING_PERCENT) / span
    return clamp(score)


def classify_risk(risk_score, vehicle_speed, critical_override):
    """
    Turn the final risk score into a 4-tier decision: risk label,
    action, recommended speed and whether to brake.

    If critical_override is True, this ALWAYS returns CRITICAL/STOP,
    regardless of the numeric risk_score.

    Speed-reduction factors (0.7 for CAUTION, 0.4 for HIGH RISK) are
    chosen so a HIGH RISK reading on ~18 km/h comes out close to the
    ~7 km/h "safe speed recommendation" example this design is based
    on -- a smooth taper toward STOP rather than one abrupt cut.
    """
    if critical_override or risk_score >= CRITICAL_THRESHOLD:
        return "CRITICAL", "STOP", 0.0, True

    if risk_score >= HIGH_RISK_THRESHOLD:
        return "HIGH RISK", "SLOW", vehicle_speed * 0.4, False

    if risk_score >= CAUTION_THRESHOLD:
        return "CAUTION", "SLOW", vehicle_speed * 0.7, False

    return "SAFE", "MOVE", vehicle_speed, False


# --------------------------------------------------------------------
# MAIN PUBLIC FUNCTION
# --------------------------------------------------------------------

def calculate_risk(object_type, distance, vehicle_speed, fog_level=0.0):
    """
    Main Risk Engine entry point.

    Args:
        object_type (str): "person", "vehicle", or "obstacle"
                            (case-insensitive, whitespace-tolerant;
                            unknown labels fall back to "obstacle").
        distance (float): distance to the object in metres (HC-SR04).
        vehicle_speed (float): current vehicle speed in m/s.
        fog_level (float): estimated fog density in percent (0-100),
                            from the camera/AI module. Defaults to 0
                            (clear visibility) so existing callers
                            that don't pass fog keep working.

    Returns:
        dict: full risk assessment (see module docstring / README
              for the exact schema).
    """
    normalized_type = validate_inputs(object_type, distance, vehicle_speed, fog_level)
    limits = SAFETY_LIMITS[normalized_type]

    ttc = calculate_ttc(distance, vehicle_speed)

    distance_risk = calculate_distance_risk(distance, limits["danger"], limits["warning"])
    ttc_risk = calculate_ttc_risk(ttc)
    fog_risk = calculate_fog_risk(fog_level)

    combined_risk = (
        (distance_risk * DISTANCE_WEIGHT)
        + (ttc_risk * TTC_WEIGHT)
        + (fog_risk * FOG_WEIGHT)
    )
    combined_risk = clamp(combined_risk)

    # --- CRITICAL SAFETY OVERRIDES ----------------------------------
    # Never trust the weighted average alone to catch a genuinely
    # critical situation (e.g. distance risk 100 + ttc risk 0 would
    # otherwise average out to well below DANGER, which is
    # unacceptable for an object already inside the danger distance).
    distance_critical = distance <= limits["danger"]
    ttc_critical = ttc <= TTC_DANGER_SECONDS
    fog_critical = fog_level > FOG_CRITICAL_PERCENT

    is_critical = distance_critical or ttc_critical or fog_critical

    if is_critical:
        # Keep the displayed score consistent with a CRITICAL classification.
        risk_score = max(combined_risk, CRITICAL_THRESHOLD)
    else:
        risk_score = combined_risk

    risk_score = round(risk_score, 2)

    risk, action, recommended_speed, brake = classify_risk(
        risk_score, vehicle_speed, critical_override=is_critical
    )

    # Fog-specific hard cutoff overrides the action/reason wording,
    # since a fog_critical situation isn't really about the object at
    # all -- it's "we can't trust what the camera is telling us".
    operation_recommended = not fog_critical

    if fog_critical:
        action = "STOP"
        reason = "OPERATION NOT RECOMMENDED - visibility too low (fog > {:.0f}%)".format(
            FOG_CRITICAL_PERCENT
        )
    elif distance_critical:
        reason = "object inside danger distance"
    elif ttc_critical:
        reason = "collision imminent (low time-to-collision)"
    elif risk == "HIGH RISK":
        reason = "high risk, slow down significantly"
    elif risk == "CAUTION":
        reason = "elevated risk, proceed with caution"
    else:
        reason = "no significant risk detected"

    return {
        "object": normalized_type,
        "distance": round(distance, 2),
        "speed": round(vehicle_speed, 2),
        "fog_level": round(fog_level, 2),
        "ttc": None if ttc == math.inf else round(ttc, 2),
        "distance_risk": round(distance_risk, 2),
        "ttc_risk": round(ttc_risk, 2),
        "fog_risk": round(fog_risk, 2),
        "risk_score": risk_score,
        "risk": risk,
        "action": action,
        "recommended_speed": round(recommended_speed, 2),
        "brake": brake,
        "operation_recommended": operation_recommended,
        "reason": reason,
    }


# --------------------------------------------------------------------
# MANUAL TEST SECTION (only runs when this file is executed directly)
# --------------------------------------------------------------------

if __name__ == "__main__":
    test_cases = [
        # 1. Very close person -> CRITICAL (inside danger distance)
        {"object_type": "person", "distance": 0.15, "vehicle_speed": 0.05, "fog_level": 0.0},
        # 2. Person inside the warning band, slow speed -> CAUTION
        {"object_type": "person", "distance": 0.5, "vehicle_speed": 0.05, "fog_level": 0.0},
        # 3. Person closer + closing a bit faster -> HIGH RISK (not yet critical)
        {"object_type": "person", "distance": 0.4, "vehicle_speed": 0.14, "fog_level": 0.0},
        # 4. Person far away -> SAFE
        {"object_type": "person", "distance": 2.0, "vehicle_speed": 0.05, "fog_level": 0.0},
        # 5. Vehicle outside danger distance but closing fast -> TTC override -> CRITICAL
        {"object_type": "vehicle", "distance": 0.6, "vehicle_speed": 0.5, "fog_level": 0.0},
        # 6. Obstacle far away, vehicle stationary -> SAFE
        {"object_type": "obstacle", "distance": 1.5, "vehicle_speed": 0.0, "fog_level": 0.0},
        # 7. Object far + safe, but moderate fog -> fog nudges risk up slightly (still not critical)
        {"object_type": "obstacle", "distance": 1.5, "vehicle_speed": 0.0, "fog_level": 65.0},
        # 8. Object looks totally safe, but fog > 90% -> HARD CUTOFF -> CRITICAL regardless
        {"object_type": "obstacle", "distance": 2.0, "vehicle_speed": 0.0, "fog_level": 95.0},
    ]

    for i, case in enumerate(test_cases, start=1):
        result = calculate_risk(**case)
        print(f"--- Test Case {i}: {case} ---")
        for key, value in result.items():
            print(f"  {key}: {value}")
        print()
