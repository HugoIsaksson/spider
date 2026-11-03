"""A first coordinated walking cycle for four three-joint legs.

This is deliberately a tunable joint-space gait. It is not inverse kinematics;
the amplitudes and leg phase offsets need calibration on the assembled robot.
"""

import math
import logging
import time

import config

logger = logging.getLogger(__name__)

LEGS = ("front_left", "front_right", "rear_left", "rear_right")
# Body-plane position of each leg, used to stagger steps for a requested heading.
LEG_POSITION = {
    "front_left": (1, 1), "front_right": (1, -1),
    "rear_left": (-1, 1), "rear_right": (-1, -1),
}


class GaitHandler:
    def __init__(self, servo_handler):
        self.servos = servo_handler
        self.heading = 0.0
        self.speed = 0.0
        self.turn = 0.0
        self.started = time.monotonic()

    def move(self, heading, speed, turn=0.0):
        if isinstance(heading, bool) or not isinstance(heading, (int, float)):
            raise ValueError("heading must be numeric")
        if isinstance(speed, bool) or not isinstance(speed, (int, float)):
            raise ValueError("speed must be numeric")
        if isinstance(turn, bool) or not isinstance(turn, (int, float)):
            raise ValueError("turn must be numeric")
        if not 0 <= speed <= 1:
            raise ValueError("speed must be between 0 and 1")
        if not -1 <= turn <= 1:
            raise ValueError("turn must be between -1 and 1")
        was_stopped = self.speed == 0 and self.turn == 0
        self.heading = float(heading) % 360
        self.speed = float(speed)
        self.turn = float(turn)
        if was_stopped and (self.speed > 0 or self.turn != 0):
            self.started = time.monotonic()
        logger.debug("Gait input: heading %.1f, speed %.2f, turn %.2f",
                     self.heading, self.speed, self.turn)

    def stop(self):
        self.speed = 0.0
        self.turn = 0.0
        self.servos.stop()
        logger.info("Gait stopped; servo PWM remains active to hold the pose")

    def update(self):
        if self.speed <= 0 and self.turn == 0:
            return
        elapsed = time.monotonic() - self.started
        heading = math.radians(self.heading)
        dx, dy = math.cos(heading), math.sin(heading)
        pose = {}
        for leg in LEGS:
            x, y = LEG_POSITION[leg]
            # Translation velocity plus tangential velocity around the body
            # center. The left stick controls the rotational component.
            vx = self.speed * dx - self.turn * y
            vy = self.speed * dy + self.turn * x
            local_speed = math.hypot(vx, vy)
            if local_speed > 1:
                vx /= local_speed
                vy /= local_speed
                local_speed = 1
            local_heading = math.degrees(math.atan2(vy, vx))
            local_radians = math.radians(local_heading)
            cycle_seconds = max(0.4, config.GAIT_CYCLE_SECONDS / max(0.1, local_speed))
            offset = (x * math.cos(local_radians) + y * math.sin(local_radians)) * 0.125
            phase = ((elapsed / cycle_seconds) + offset) % 1.0
            swing_fraction = 0.28
            if phase < swing_fraction:
                progress = phase / swing_fraction
                fore_aft = 1.0 - 2.0 * progress
                lift = math.sin(math.pi * progress)
            else:
                progress = (phase - swing_fraction) / (1.0 - swing_fraction)
                fore_aft = -1.0 + 2.0 * progress
                lift = 0.0
            # Coxa steers the foot toward the requested travel heading. The
            # remaining joints provide a simple lift and fore/aft leg sweep.
            leg_heading = math.degrees(math.atan2(y, x))
            steering = ((local_heading - leg_heading + 180) % 360) - 180
            steering = max(-35, min(35, steering))
            pose[leg] = {
                "coxa": 90 + steering,
                "femur": 90 + fore_aft * config.GAIT_STEP_DEGREES,
                "tibia": 90 + lift * config.GAIT_LIFT_DEGREES,
            }
        self.servos.set_pose(pose)
