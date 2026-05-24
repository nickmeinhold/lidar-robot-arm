"""Low-level Feetech bus probe — bypass the SDK and dump raw bytes.

Sends a hand-built broadcast PING packet at 1 Mbaud and shows every byte
that comes back. Useful when the SDK's ping() returns nothing and we need
to know whether the bus is electrically dead, sending garbage, or just
using a baud rate we haven't tried.
"""
from __future__ import annotations

import sys
import time

import serial

PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/cu.usbmodem5B141154581"
BAUDS = [1_000_000, 2_000_000, 500_000, 250_000, 128_000, 115_200, 76_800, 57_600, 38_400, 19_200, 9600, 4800]


def ping_packet(servo_id: int) -> bytes:
    """Build a Feetech/Dynamixel-style PING: FF FF ID LEN INST CHKSUM."""
    length = 0x02
    inst = 0x01  # PING
    checksum = (~(servo_id + length + inst)) & 0xFF
    return bytes([0xFF, 0xFF, servo_id, length, inst, checksum])


def probe(baud: int) -> bytes:
    s = serial.Serial(PORT, baud, timeout=0.1)
    try:
        # Broadcast ping (ID 0xFE) — every servo on the bus should reply
        for sid in (0xFE, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06):
            s.reset_input_buffer()
            pkt = ping_packet(sid)
            s.write(pkt)
            s.flush()
            time.sleep(0.05)
            data = s.read(64)
            if data:
                return data
        return b""
    finally:
        s.close()


def main() -> int:
    print(f"Probing {PORT}")
    for baud in BAUDS:
        try:
            data = probe(baud)
        except serial.SerialException as e:
            print(f"  baud {baud}: open failed: {e}")
            continue
        if data:
            print(f"  baud {baud}: RX {len(data)} bytes: {data.hex(' ')}")
            return 0
        else:
            print(f"  baud {baud}: silent")
    print("\nNo bytes returned at any baud. Bus is electrically silent.")
    return 1


if __name__ == "__main__":
    sys.exit(main())
