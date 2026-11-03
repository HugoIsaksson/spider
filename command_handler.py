"""JSON command parsing and validation, separate from transport and PWM."""

import json
import logging

logger = logging.getLogger(__name__)


class CommandHandler:
    def __init__(self, servo_handler, gait_handler):
        self.servos = servo_handler
        self.gait = gait_handler

    def handle(self, text):
        try:
            message = json.loads(text)
            if not isinstance(message, dict):
                raise ValueError("message must be a JSON object")
            kind = message.get("type")
            if kind == "move":
                self.gait.move(
                    message.get("heading", 0),
                    message.get("speed", 0),
                    message.get("turn", 0),
                )
                logger.debug("Drive: heading=%s speed=%s turn=%s",
                             self.gait.heading, self.gait.speed, self.gait.turn)
            elif kind == "stop":
                self.gait.stop()
                logger.info("Walking stopped")
            else:
                raise ValueError("unknown command type")
            return {"type": "ack", "command": kind}
        except (ValueError, KeyError, TypeError) as exc:
            logger.warning("Rejected command: %s", exc)
            return {"type": "error", "message": str(exc)}
        except Exception as exc:
            logger.exception("Command failed while controlling hardware")
            return {"type": "error", "message": "hardware command failed: %s" % exc}
