"""Hardware and servo configuration for the spider robot."""

ANGLE_MIN = 0
ANGLE_MAX = 180
WEBSOCKET_PORT = 8765

# Measured joint-axis/link dimensions, in millimeters.
COXA_LENGTH_MM = 33.0
FEMUR_LENGTH_MM = 49.0
TIBIA_LENGTH_MM = 79.0
BODY_FRONT_REAR_COXA_SPACING_MM = 66.0
BODY_LEFT_RIGHT_COXA_SPACING_MM = 69.0
FEMUR_WIDTH_MM = 38.0  # physical width; not used by planar IK

# Neutral foot target, measured from each coxa axis. Initial coxa-axis-to-floor
# height uses the 40 mm measurement provided; verify the exact reference plane.
IK_HOME_RADIAL_MM = 100.0
IK_HOME_HEIGHT_MM = 25.0

# Cartesian gait tuning in millimeters, not joint-angle oscillations.
GAIT_STEP_LENGTH_MM = 45.0
GAIT_LIFT_MM = 25.0
GAIT_CYCLE_SECONDS = 3.5
GAIT_UPDATE_SECONDS = 0.04
GAIT_INPUT_SMOOTH_SECONDS = 0.18

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

# Four-beat crawl: one leg swings at a time to keep three feet planted.
GAIT_LEG_PHASES = {
    "front_left": 0.00,
    "front_right": 0.50,
    "rear_left": 0.75,
    "rear_right": 0.25,
}
GAIT_SWING_FRACTION = 0.22
