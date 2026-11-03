"""WebSocket transport for the Linux Raspberry Pi."""

import json
import logging

from websockets.server import serve

logger = logging.getLogger(__name__)


class ConnectionHandler:
    def __init__(self, command_handler):
        self.commands = command_handler

    # The optional path parameter supports the handler signature used by
    # websockets 10 and 11 on older Raspberry Pi OS Python installations.
    async def _client(self, websocket, path=None):
        remote = getattr(websocket, "remote_address", None)
        logger.info("Browser connected from %s", remote)
        try:
            await websocket.send(json.dumps({"type": "ready"}))
            async for message in websocket:
                logger.info("Received WebSocket message: %s", message)
                result = self.commands.handle(message)
                logger.info("Sending command result: %s", result)
                await websocket.send(json.dumps(result))
        except Exception:
            logger.exception("WebSocket connection failed for %s", remote)
            raise
        finally:
            logger.info("Browser disconnected: %s", remote)

    async def start(self, port):
        server = await serve(self._client, "0.0.0.0", port, max_size=2048)
        logger.info("Listening on 0.0.0.0:%d", port)
        return server
