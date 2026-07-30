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
import json
import logging
import signal
from contextlib import AsyncExitStack
from pathlib import Path

import websockets
from websockets.asyncio.server import Server, ServerConnection
from websockets.datastructures import Headers
from websockets.http11 import Request, Response

from .arm_controller import ArmController, ConsoleArmController
from .discovery import advertise, get_lan_ip
from .feetech_controller import FeetechArmController
from .protocol import make_pong, parse_message

log = logging.getLogger(__name__)

_STATIC_DIR = Path(__file__).parent / "static"
_ZERO_CAL_PATH = Path(__file__).parent / "zero_calibration.json"


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
        """Handle a 3D viewer connection.

        Viewers are normally receive-only (they render the iPhone's stream).
        But when the viewer's "Drive Real Arm" mode is on, it sends the same
        ``arm_state`` messages the iPhone does — so we route those straight
        into the controller, reusing the identical clamp/write pipeline. This
        turns the 3D UI into a manual teleop input.
        """
        remote = connection.remote_address
        self._viewers.add(connection)
        log.info("Viewer connected: %s (%d viewers)", remote, len(self._viewers))
        try:
            async for raw in connection:
                if not isinstance(raw, str):
                    continue
                if await self._handle_control(connection, raw):
                    continue  # was a control message (torque / capture-zero)
                msg = parse_message(raw)
                if msg is None:
                    continue  # not an arm_state — viewers usually send nothing
                await self._controller.update(msg.angles, msg.tracking)
        except websockets.ConnectionClosed:
            pass
        finally:
            self._viewers.discard(connection)
            log.info("Viewer disconnected: %s (%d viewers)", remote, len(self._viewers))

    async def _handle_control(self, connection: ServerConnection, raw: str) -> bool:
        """Handle viewer control messages (torque toggle, capture-zero).

        Returns True if *raw* was a control message (and was handled), so the
        caller skips arm_state parsing. Control verbs only work on a real
        hardware controller; the console controller ignores them gracefully.
        """
        try:
            data = json.loads(raw)
        except (json.JSONDecodeError, TypeError):
            return False
        mtype = data.get("type")

        if mtype == "set_torque":
            on = bool(data.get("on"))
            if hasattr(self._controller, "set_torque"):
                await asyncio.to_thread(self._controller.set_torque, on)
            return True

        if mtype == "capture_zero":
            if hasattr(self._controller, "read_present_ticks"):
                ticks = await asyncio.to_thread(self._controller.read_present_ticks)
                try:
                    _ZERO_CAL_PATH.write_text(json.dumps(ticks, indent=2))
                    log.info("Captured zero calibration: %s -> %s", ticks, _ZERO_CAL_PATH)
                except OSError as exc:
                    log.warning("Failed to write zero calibration: %s", exc)
                # Echo the captured ticks back so the viewer can confirm.
                await connection.send(json.dumps({"type": "zero_captured", "ticks": ticks}))
            return True

        return False

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
            except (websockets.ConnectionClosed, OSError):
                pass  # old connection may already be dead

        self._active_client = connection
        log.info("Controller connected: %s", remote)

        try:
            async for raw in connection:
                # Binary frames (H.264 video) are forwarded verbatim to viewers
                # without parsing — the controller pipeline only cares about
                # text JSON arm_state messages.
                if not isinstance(raw, str):
                    if self._viewers:
                        websockets.broadcast(self._viewers, raw)
                    continue

                msg = parse_message(raw)
                if msg is None:
                    continue

                await self._controller.update(msg.angles, msg.tracking)
                await connection.send(make_pong(msg.timestamp))

                # Broadcast to all connected viewers
                if self._viewers:
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
    feetech_port: str | None = None,
) -> None:
    """Start the server and block until interrupted."""
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
        datefmt="%H:%M:%S",
    )

    controller: ArmController
    if feetech_port:
        feetech = FeetechArmController(feetech_port)
        await asyncio.to_thread(feetech.connect)
        controller = feetech
    else:
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
            ip = get_lan_ip()

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
    parser.add_argument(
        "--feetech-port",
        type=str,
        default=None,
        help="Serial port for Feetech servo bus (e.g. /dev/cu.usbmodem...). "
        "If omitted, the console controller is used.",
    )
    args = parser.parse_args()
    asyncio.run(
        main(
            port=args.port,
            use_bonjour=not args.no_bonjour,
            feetech_port=args.feetech_port,
        )
    )


if __name__ == "__main__":
    cli()
