"""WebSocket server for iPhone → robot arm teleoperation.

Accepts a single client at a time on ``/ws``, parses ``arm_state`` messages,
forwards them to the active :class:`ArmController`, and echoes ``pong``
responses for latency measurement.

Can be run directly (``python server/server.py``) or as a package
(``python -m server``).
"""

from __future__ import annotations

import argparse
import asyncio
import logging
import signal
from contextlib import AsyncExitStack
from pathlib import Path

import websockets
from websockets.asyncio.server import Server, ServerConnection
from websockets.datastructures import Headers
from websockets.http11 import Request, Response

from .arm_controller import ArmController, ConsoleArmController
from .discovery import advertise, _get_lan_ip
from .protocol import make_pong, parse_message

log = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"


class ArmTrackerServer:
    """Single-client WebSocket server for arm tracking with live 3D viewer."""

    def __init__(self, controller: ArmController) -> None:
        self._controller = controller
        self._active_client: ServerConnection | None = None
        self._viewers: set[ServerConnection] = set()

    # --- HTTP / WebSocket routing ---

    async def _process_request(
        self, connection: ServerConnection, request: Request
    ) -> Response | None:
        """Route requests: ``/`` serves the viewer, ``/ws`` and ``/viewer`` upgrade."""
        if request.path == "/":
            return self._serve_viewer()
        if request.path in ("/ws", "/viewer"):
            return None  # proceed with WebSocket handshake
        log.warning("Rejected connection to %s", request.path)
        return connection.respond(404, f"Not Found: {request.path}\n")

    def _serve_viewer(self) -> Response:
        """Return the 3D viewer HTML page as an HTTP response."""
        html_path = _STATIC_DIR / "viewer.html"
        try:
            body = html_path.read_bytes()
        except FileNotFoundError:
            return Response(404, "Not Found", Headers(), b"viewer.html not found\n")
        headers = Headers(
            [
                ("Content-Type", "text/html; charset=utf-8"),
                ("Content-Length", str(len(body))),
                ("Cache-Control", "no-cache"),
            ]
        )
        return Response(200, "OK", headers, body)

    # --- Connection handler ---

    async def handler(self, connection: ServerConnection) -> None:
        """Route WebSocket connections by path."""
        path = connection.request.path
        if path == "/viewer":
            await self._viewer_handler(connection)
        else:
            await self._controller_handler(connection)

    async def _viewer_handler(self, connection: ServerConnection) -> None:
        """Handle a 3D viewer connection (receive-only)."""
        remote = connection.remote_address
        self._viewers.add(connection)
        log.info("Viewer connected: %s (%d viewers)", remote, len(self._viewers))
        try:
            async for _ in connection:
                pass  # viewers don't send meaningful data
        except websockets.ConnectionClosed:
            pass
        finally:
            self._viewers.discard(connection)
            log.info("Viewer disconnected: %s (%d viewers)", remote, len(self._viewers))

    async def _controller_handler(self, connection: ServerConnection) -> None:
        """Handle the iPhone controller connection (single-client, last-writer-wins)."""
        remote = connection.remote_address

        # --- Last-writer-wins: boot the old client if a new one connects ---
        if self._active_client is not None:
            old = self._active_client
            log.info("Replacing old controller with %s", remote)
            self._active_client = None
            try:
                await old.close()
            except Exception:
                pass  # old connection may already be dead

        self._active_client = connection
        log.info("Controller connected: %s", remote)

        try:
            async for raw in connection:
                if not isinstance(raw, str):
                    continue  # ignore binary frames

                msg = parse_message(raw)
                if msg is None:
                    continue

                await self._controller.update(msg.angles, msg.tracking)
                await connection.send(make_pong(msg.timestamp))

                # Broadcast to all connected viewers
                if self._viewers:
                    log.debug("Broadcasting to %d viewers", len(self._viewers))
                    websockets.broadcast(self._viewers, raw)
        except websockets.ConnectionClosed:
            log.info("Controller disconnected: %s", remote)
        finally:
            self._active_client = None
            await self._controller.stop()
            log.info("Controller slot released")


async def main(
    port: int = 8765,
    use_bonjour: bool = True,
) -> None:
    """Start the server and block until interrupted."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    controller = ConsoleArmController()
    tracker = ArmTrackerServer(controller)

    stop_event = asyncio.Event()
    loop = asyncio.get_running_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, stop_event.set)

    async with AsyncExitStack() as stack:
        # Optionally advertise via Bonjour
        if use_bonjour:
            ip = await stack.enter_async_context(advertise(port))
        else:
            ip = _get_lan_ip()

        server: Server = await stack.enter_async_context(
            websockets.serve(
                tracker.handler,
                "0.0.0.0",
                port,
                process_request=tracker._process_request,
            )
        )

        print(f"\nArm Tracker server running on ws://{ip}:{port}/ws")
        print(f"3D Viewer:  http://{ip}:{port}/")
        print("Waiting for iPhone connection…\n")

        await stop_event.wait()
        print("\nShutting down…")


def cli() -> None:
    """Parse CLI args and run the server."""
    parser = argparse.ArgumentParser(
        description="WebSocket server for iPhone arm tracking",
    )
    parser.add_argument(
        "--port",
        type=int,
        default=8765,
        help="WebSocket port (default: 8765)",
    )
    parser.add_argument(
        "--no-bonjour",
        action="store_true",
        help="Disable Bonjour/zeroconf advertisement",
    )
    args = parser.parse_args()
    asyncio.run(main(port=args.port, use_bonjour=not args.no_bonjour))


if __name__ == "__main__":
    cli()
