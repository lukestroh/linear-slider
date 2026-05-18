#!/usr/bin/env python3
"""Small UDP smoke-test client for the ClearCore linear slider firmware.

Sends command payloads as "<status>,<pos_steps>" and prints JSON feedback from
the controller. Defaults match the firmware's static ClearCore address.
"""

import argparse
import socket
import time


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="169.254.57.177", help="ClearCore IP")
    parser.add_argument("--port", type=int, default=8888, help="ClearCore UDP port")
    parser.add_argument("--status", type=int, default=0, help="SystemStatus enum value")
    parser.add_argument("--pos", type=int, default=0, help="Target position in steps")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between commands")
    parser.add_argument("--count", type=int, default=20, help="Number of commands to send")
    parser.add_argument("--listen-timeout", type=float, default=0.5, help="Feedback timeout per command")
    args = parser.parse_args()

    command = f"{args.status},{args.pos}".encode("ascii")
    destination = (args.host, args.port)

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("", 0))
        sock.settimeout(args.listen_timeout)
        print(f"local UDP endpoint: {sock.getsockname()[0]}:{sock.getsockname()[1]}")
        print(f"sending {command!r} to {args.host}:{args.port}")

        for _ in range(args.count):
            sock.sendto(command, destination)
            deadline = time.monotonic() + args.listen_timeout
            while time.monotonic() < deadline:
                try:
                    data, sender = sock.recvfrom(1024)
                except socket.timeout:
                    break
                print(f"{sender[0]}:{sender[1]} -> {data.decode('utf-8', errors='replace')}")
            time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
