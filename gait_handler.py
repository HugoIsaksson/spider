"""Smooth Cartesian walking gait driven by three-joint inverse kinematics."""

import logging
import math
import time

import config
from inverse_kinematics import solve_leg

logger = logging.getLogger(__name__)

LEGS = ("front_left", "front_right", "rear_left", "rear_right")
HALF_BODY_X = config.BODY_FRONT_REAR_COXA_SPACING_MM / 2
HALF_BODY_Y = config.BODY_LEFT_RIGHT_COXA_SPACING_MM / 2
TURN_RADIUS = math.hypot(HALF_BODY_X, HALF_BODY_Y)

# Local outward directions and hip positions use x=forward, y=right.
LEG_GEOMETRY = {
    "front_left":  (HALF_BODY_X, -HALF_BODY_Y, 1, -1),
    "front_right": (HALF_BODY_X, HALF_BODY_Y, 1, 1),
    "rear_left":   (-HALF_BODY_X, -HALF_BODY_Y, -1, -1),
    "rear_right":  (-HALF_BODY_X, HALF_BODY_Y, -1, 1),
}

HOME_ANGLES = solve_leg(config.IK_HOME_RADIAL_MM, 0, config.IK_HOME_HEIGHT_MM)


def _smoothstep(value):
    value = max(0.0, min(1.0, value))
    return value * value * (3.0 - 2.0 * value)


class GaitHandler:
    def __init__(self, servo_handler):
        self.servos = servo_handler
        self.heading = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.target_x = 0.0
        self.target_y = 0.0
        self.current_x = 0.0
        self.current_y = 0.0
        self.current_turn = 0.0
        self.started = time.monotonic()
        self.last_update = self.started

    def move(self, heading, speed, turn=0.0):
        for name, value in (("heading", heading), ("speed", speed), ("turn", turn)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
                raise ValueError(name + " must be a finite number")
        if not 0 <= speed <= 1:
            raise ValueError("speed must be between 0 and 1")
        if not -1 <= turn <= 1:
            raise ValueError("turn must be between -1 and 1")

        was_idle = self.speed == 0 and self.turn == 0
        self.heading = float(heading) % 360
        self.speed = float(speed)
        self.turn = float(turn)
        angle = math.radians(self.heading)
        self.target_x = self.speed * math.cos(angle)
        self.target_y = self.speed * math.sin(angle)
        if was_idle and (self.speed > 0 or self.turn != 0):
            self.started = time.monotonic()
        logger.debug("IK drive target: heading %.1f, speed %.2f, turn %.2f",
                     self.heading, self.speed, self.turn)

    def stop(self):
        self.speed = self.turn = 0.0
        self.target_x = self.target_y = 0.0
        self.current_x = self.current_y = self.current_turn = 0.0
        self.servos.stop()
        logger.info("Gait stopped; servos hold the current pose")

    def update(self):
        now = time.monotonic()
        dt = max(0.0, now - self.last_update)
        self.last_update = now
        smoothing = max(0.01, config.GAIT_INPUT_SMOOTH_SECONDS)
        alpha = min(1.0, dt / smoothing)
        self.current_x += (self.target_x - self.current_x) * alpha
        self.current_y += (self.target_y - self.current_y) * alpha
        self.current_turn += (self.turn - self.current_turn) * alpha

        if (abs(self.current_x) < 0.002 and abs(self.current_y) < 0.002
                and abs(self.current_turn) < 0.002):
            return

        velocities = {}
        max_velocity = 0.0
        for leg in LEGS:
            hip_x, hip_y, _, _ = LEG_GEOMETRY[leg]
            vx = self.current_x - self.current_turn * hip_y / TURN_RADIUS
            vy = self.current_y + self.current_turn * hip_x / TURN_RADIUS
            magnitude = math.hypot(vx, vy)
            velocities[leg] = (vx, vy, magnitude)
            max_velocity = max(max_velocity, magnitude)
        cycle_seconds = max(0.45, config.GAIT_CYCLE_SECONDS / max(0.1, max_velocity))
        elapsed = now - self.started
        pose = {}

        for leg in LEGS:
            _, _, sign_x, sign_y = LEG_GEOMETRY[leg]
            phi = math.atan2(sign_y, sign_x)
            radial_x, radial_y = math.cos(phi), math.sin(phi)
            tangent_x, tangent_y = -radial_y, radial_x
            vx, vy, magnitude = velocities[leg]

            if magnitude > 1e-5:
                direction_x, direction_y = vx / magnitude, vy / magnitude
            else:
                direction_x, direction_y = 0.0, 0.0
            radial_direction = direction_x * radial_x + direction_y * radial_y
            lateral_direction = direction_x * tangent_x + direction_y * tangent_y

            phase = (elapsed / cycle_seconds + config.GAIT_LEG_PHASES[leg]) % 1.0
            swing = phase < config.GAIT_SWING_FRACTION
            if swing:
                progress = _smoothstep(phase / config.GAIT_SWING_FRACTION)
                stride_factor = -1.0 + 2.0 * progress
                lift = config.GAIT_LIFT_MM * math.sin(math.pi * progress)
            else:
                stance_progress = (phase - config.GAIT_SWING_FRACTION) / (1.0 - config.GAIT_SWING_FRACTION)
                progress = _smoothstep(stance_progress)
                stride_factor = 1.0 - 2.0 * progress
                lift = 0.0

            half_step = config.GAIT_STEP_LENGTH_MM / 2.0
            radial = config.IK_HOME_RADIAL_MM + stride_factor * half_step * radial_direction
            lateral = stride_factor * half_step * lateral_direction
            height = config.IK_HOME_HEIGHT_MM - lift
            solved = solve_leg(radial, lateral, height)
            pose[leg] = {
                joint: 90.0 + (solved[joint] - HOME_ANGLES[joint])
                for joint in ("coxa", "femur", "tibia")
            }

        self.servos.set_pose(pose)
