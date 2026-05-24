"""Deep scan for Feetech servos.

Pings every ID from 1 to 253 at 1 Mbaud, three times each, and reports
which IDs answer at least once. This catches:
  - servos configured to non-default IDs (factory default is 1)
  - servos with intermittent comms (only some pings succeed)
  - the actual model number of each servo

Use this when scan_bus.py finds fewer servos than expected.
"""
from __future__ import annotations

import sys

from scservo_sdk import PortHandler, PacketHandler, COMM_SUCCESS

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5A4B0468521"
BAUD = 1_000_000
PINGS_PER_ID = 3


def main() -> int:
    port = PortHandler(PORT)
    if not port.openPort() or not port.setBaudRate(BAUD):
        print(f"Failed to open {PORT} @ {BAUD}")
        return 1
    packet = PacketHandler(0)

    print(f"Pinging IDs 1-253 at {BAUD} baud, {PINGS_PER_ID}x each...\n")
    print(f"{'ID':>4}  {'hits':>5}  {'model':>8}  {'err':>4}")
    print("-" * 30)

    found = 0
    for sid in range(1, 254):
        hits = 0
        last_model = 0
        last_err = 0
        for _ in range(PINGS_PER_ID):
            model, comm, err = packet.ping(port, sid)
            if comm == COMM_SUCCESS:
                hits += 1
                last_model = model
                last_err = err
        if hits:
            found += 1
            tag = " OK " if hits == PINGS_PER_ID else "flaky"
            print(f"{sid:>4}  {hits}/{PINGS_PER_ID} {tag}  0x{last_model:04X}  0x{last_err:02X}")

    print(f"\nTotal: {found} servo(s) responded.")
    port.closePort()
    return 0 if found else 1


if __name__ == "__main__":
    sys.exit(main())
