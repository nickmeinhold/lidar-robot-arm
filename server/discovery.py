"""Bonjour/zeroconf service advertisement.

Advertises ``_armtracker._tcp.local.`` so the iPhone can auto-discover the
server on the local network.
"""

from __future__ import annotations

import logging
import socket
from contextlib import asynccontextmanager
from typing import AsyncIterator

from zeroconf import IPVersion
from zeroconf.asyncio import AsyncServiceInfo, AsyncZeroconf

log = logging.getLogger(__name__)

SERVICE_TYPE = "_armtracker._tcp.local."
SERVICE_NAME = f"ArmTracker Server.{SERVICE_TYPE}"


def _get_lan_ip() -> str:
    """Return this machine's LAN IP address.

    Opens a UDP socket to a public DNS address (no data sent) so the OS
    chooses the correct interface.  Falls back to ``127.0.0.1`` if the
    machine has no network.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
            s.connect(("8.8.8.8", 80))
            return s.getsockname()[0]
    except OSError:
        return "127.0.0.1"


@asynccontextmanager
async def advertise(port: int) -> AsyncIterator[str]:
    """Advertise the server via Bonjour while the context is active.

    Yields the LAN IP address used for advertisement.

    Usage::

        async with advertise(port=8765) as ip:
            print(f"Advertising on {ip}:{port}")
            ...
    """
    ip = _get_lan_ip()
    info = AsyncServiceInfo(
        SERVICE_TYPE,
        SERVICE_NAME,
        addresses=[socket.inet_aton(ip)],
        port=port,
        properties={"version": "1"},
    )

    azc = AsyncZeroconf(ip_version=IPVersion.V4Only)
    try:
        await azc.async_register_service(info)
        log.info("Bonjour: registered %s on %s:%d", SERVICE_NAME, ip, port)
        yield ip
    finally:
        await azc.async_unregister_service(info)
        await azc.async_close()
        log.info("Bonjour: unregistered %s", SERVICE_NAME)
