"""Hardware and servo configuration for the spider robot."""

ANGLE_MIN = 0
ANGLE_MAX = 180
WEBSOCKET_PORT = 8765

# Initial joint-space gait tuning (degrees). These are starting values and
# must be calibrated for the assembled leg geometry and servo orientation.
GAIT_STEP_DEGREES = 20
GAIT_LIFT_DEGREES = 18
GAIT_CYCLE_SECONDS = 1.2
GAIT_UPDATE_SECONDS = 0.04

# Channel assignment in coxa, femur, tibia order, translated from the legacy
# leg.servos order (tibia, femur, coxa).
SERVO_CHANNELS = {
    "front_left":  (2, 1, 0),
    "front_right": (5, 4, 3),
    "rear_left":   (10, 11, 12),
    "rear_right":  (13, 14, 15),
}

# Legacy invert flags translated to coxa, femur, tibia order. A true value
# reverses the logical servo angle around 90 degrees.
SERVO_DIRECTION = {
    "front_left":  {"coxa": -1, "femur": 1, "tibia": -1},
    "front_right": {"coxa": 1, "femur": -1, "tibia": 1},
    "rear_left":   {"coxa": -1, "femur": -1, "tibia": 1},
    "rear_right":  {"coxa": 1, "femur": 1, "tibia": -1},
}
# The old stand() pose used these positions: front/back coxa 120/60,
# all femurs 60, and all tibias 120. Use it as the initial walking center.
SERVO_CENTER = {
    "front_left":  {"coxa": 120, "femur": 60, "tibia": 120},
    "front_right": {"coxa": 120, "femur": 60, "tibia": 120},
    "rear_left":   {"coxa": 60, "femur": 60, "tibia": 120},
    "rear_right":  {"coxa": 60, "femur": 60, "tibia": 120},
}
