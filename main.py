"""Application entry point for a Linux Raspberry Pi."""

import asyncio
import logging
import os

import config
from command_handler import CommandHandler
from connection_handler import ConnectionHandler
from gait_handler import GaitHandler
from servo_handler import ServoHandler

logging.basicConfig(
    level=getattr(logging, os.environ.get("SPIDER_LOG_LEVEL", "INFO").upper(), logging.INFO),
    format="%(asctime)s %(levelname)s %(name)s: %(message)s",
)
logger = logging.getLogger("spider")


async def main():
    logger.info("Initializing servo controller")
    servos = ServoHandler()
    gait = GaitHandler(servos)
    commands = CommandHandler(servos, gait)
    connections = ConnectionHandler(commands)
    async def gait_loop():
        while True:
            try:
                gait.update()
            except Exception:
                logger.exception("Gait update failed; stopping gait")
                gait.stop()
            await asyncio.sleep(config.GAIT_UPDATE_SECONDS)

    asyncio.create_task(gait_loop())
    logger.info("Starting WebSocket server on 0.0.0.0:%d", config.WEBSOCKET_PORT)
    server = await connections.start(config.WEBSOCKET_PORT)
    logger.info("WebSocket listening on port %d", config.WEBSOCKET_PORT)
    await server.wait_closed()


asyncio.run(main())
