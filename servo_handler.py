"""Spider joint mapping and safe angle-to-PWM conversion."""

import logging
import pi_servo_hat

from config import (ANGLE_MAX, ANGLE_MIN, SERVO_CHANNELS, SERVO_CENTER,
                    SERVO_DIRECTION)

JOINTS = ("coxa", "femur", "tibia")
logger = logging.getLogger(__name__)


class ServoHandler:
    def __init__(self):
        self.driver = pi_servo_hat.PiServoHat()
        if self.driver is None:
            raise RuntimeError("Could not initialize SparkFun Pi Servo HAT")
        connected = None
        if hasattr(self.driver, "is_connected"):
            connected = self.driver.is_connected()
        elif hasattr(self.driver, "isConnected"):
            connected = self.driver.isConnected()
        if connected is False:
            raise RuntimeError("Pi Servo HAT not detected on I2C")
        # SparkFun's examples restart the PCA9685 before the first movement.
        self.driver.restart()
        logger.info("SparkFun Pi Servo HAT initialized")

    def _channel(self, leg, joint):
        if leg not in SERVO_CHANNELS or joint not in JOINTS:
            raise ValueError("Unknown leg or joint")
        return SERVO_CHANNELS[leg][JOINTS.index(joint)]

    def set_joint(self, leg, joint, angle):
        if isinstance(angle, bool) or not isinstance(angle, (int, float)):
            raise ValueError("angle must be numeric")
        if not ANGLE_MIN <= angle <= ANGLE_MAX:
            raise ValueError("angle outside configured safe range")
        # The gait sends offsets around logical 90 degrees. Preserve the old
        # stand pose as the logical center, then apply the legacy inversion.
        logical_angle = SERVO_CENTER[leg][joint] + (angle - 90)
        physical_angle = logical_angle
        if SERVO_DIRECTION[leg][joint] < 0:
            physical_angle = 180 - logical_angle
        # Keep the angle-to-PCA word conversion from the user's existing code.
        value = round(physical_angle * 2.3 + 100)
        logger.debug("Servo %s/%s -> channel %d, angle %.1f, PCA word %d",
                     leg, joint, self._channel(leg, joint), physical_angle, value)
        result = self.driver.PCA9685.set_channel_word(self._channel(leg, joint), 0, value)
        if result is False:
            raise RuntimeError("PCA9685 rejected write on channel %d" % self._channel(leg, joint))

    def set_pose(self, angles):
        # Validate the whole pose before moving any servo.
        updates = []
        for leg, joints in angles.items():
            if not isinstance(joints, dict):
                raise ValueError("each leg pose must be an object")
            for joint, angle in joints.items():
                self._channel(leg, joint)
                if isinstance(angle, bool) or not isinstance(angle, (int, float)):
                    raise ValueError("angle must be numeric")
                if not ANGLE_MIN <= angle <= ANGLE_MAX:
                    raise ValueError("angle outside configured safe range")
                updates.append((leg, joint, angle))
        for leg, joint, angle in updates:
            self.set_joint(leg, joint, angle)

    def stop(self):
        # Keep PWM active so the robot holds its pose after walking stops.
        pass
