"""Scan the Feetech servo bus to discover IDs and baud rate.

Pings IDs 1-20 at common SO-100 baud rates, prints what responds.
"""
from __future__ import annotations

import sys
from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/tty.usbmodem5B141154581"
BAUDS = [1_000_000, 500_000, 115_200, 57_600]
IDS = range(1, 21)


def main() -> int:
    port = PortHandler(PORT)
    if not port.openPort():
        print(f"Failed to open {PORT}")
        return 1
    found_any = False
    for proto in (0, 1):
        packet = PacketHandler(proto)
        for baud in BAUDS:
            if not port.setBaudRate(baud):
                continue
            hits = []
            for sid in IDS:
                model, comm, err = packet.ping(port, sid)
                if comm == COMM_SUCCESS:
                    hits.append((sid, model, err))
            if hits:
                found_any = True
                print(f"\nproto={proto} baud={baud}:")
                for sid, model, err in hits:
                    print(f"  ID {sid:>2}  model=0x{model:04X}  err=0x{err:02X}")

    if not found_any:
        print("No servos found on any baud rate.")
    port.closePort()
    return 0 if found_any else 2


if __name__ == "__main__":
    sys.exit(main())
