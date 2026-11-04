"""Planar three-joint leg inverse kinematics in millimeters/degrees."""

import math

import config


def solve_leg(radial_mm, lateral_mm, height_mm):
    """Return coxa, femur and tibia model angles for one leg.

    radial_mm is outward from the coxa axis, lateral_mm is sideways from that
    leg's nominal radial plane, and height_mm is down from the coxa-axis plane.
    The coxa link is removed before solving the femur/tibia two-link plane.
    """
    coxa_angle = math.degrees(math.atan2(lateral_mm, radial_mm))
    reach = math.hypot(radial_mm, lateral_mm) - config.COXA_LENGTH_MM
    if reach <= 0:
        raise ValueError("foot target is inside the coxa link")

    distance = math.hypot(reach, height_mm)
    upper = config.FEMUR_LENGTH_MM
    lower = config.TIBIA_LENGTH_MM
    min_reach = abs(upper - lower) + 0.5
    max_reach = upper + lower - 0.5
    if distance < min_reach or distance > max_reach:
        raise ValueError("foot target is outside the leg workspace: %.1f mm" % distance)

    cos_knee = ((reach * reach + height_mm * height_mm - upper * upper - lower * lower)
                / (2 * upper * lower))
    cos_knee = max(-1.0, min(1.0, cos_knee))
    knee_internal = math.acos(cos_knee)
    femur_angle = math.atan2(height_mm, reach) - math.atan2(
        lower * math.sin(knee_internal), upper + lower * math.cos(knee_internal)
    )
    # A straight leg corresponds to 180 degrees in this model's tibia angle.
    tibia_angle = math.pi - knee_internal
    return {
        "coxa": coxa_angle,
        "femur": math.degrees(femur_angle),
        "tibia": math.degrees(tibia_angle),
    }
