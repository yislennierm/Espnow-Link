#!/usr/bin/env python3

import argparse
import serial
import serial.tools.list_ports
from modules.crfs import CRSFModule

COMMON_BAUDS = [9600, 19200, 38400, 57600, 115200, 230400, 250000, 400000, 460800, 921600]
MODULES = {
    "crsf": CRSFModule,
}

def pick_port():
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("No serial devices found.")
        return None

    print("Available serial devices:")
    for i, p in enumerate(ports):
        print(f"  [{i}] {p.device}  ({p.description})")

    while True:
        choice = input("Select device number: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 0 <= idx < len(ports):
                return ports[idx].device
        print("Invalid selection, try again.")

def pick_baud():
    print("\nCommon baudrates:")
    for i, b in enumerate(COMMON_BAUDS):
        print(f"  [{i}] {b}")
    choice = input("Select baud (default 400000): ").strip()
    if choice == "":
        return 400000
    if choice.isdigit():
        idx = int(choice)
        if 0 <= idx < len(COMMON_BAUDS):
            return COMMON_BAUDS[idx]
    print("Invalid selection, defaulting to 400000.")
    return 400000

def pick_module():
    print("\nAvailable modules:")
    for i, name in enumerate(MODULES.keys()):
        print(f"  [{i}] {name.upper()}")

    while True:
        choice = input("Select module: ").strip()
        if choice.isdigit():
            idx = int(choice)
            if 0 <= idx < len(MODULES):
                return list(MODULES.values())[idx]
        print("Invalid selection, try again.")

def main():
    parser = argparse.ArgumentParser(description="Protocol Sniffer")
    parser.add_argument("port", nargs="?", help="Serial port (e.g. /dev/cu.usbserial-1410)")
    parser.add_argument("-b", "--baud", type=int, default=None,
                        help="Baudrate (default: 400000)")
    parser.add_argument("-m", "--module", choices=MODULES.keys(), default=None,
                        help="Protocol module to use")
    parser.add_argument("-hex", action="store_true",
                        help="Print raw hex instead of binary text")
    args = parser.parse_args()

    # Interactive selection if missing args
    port = args.port or pick_port()
    baud = args.baud or pick_baud()
    module_cls = MODULES.get(args.module) if args.module else pick_module()

    if not port or not module_cls:
        return

    module = module_cls(hex_output=args.hex)

    ser = serial.Serial(port, baud, timeout=0.1)
    print(f"\n[{module.NAME}] Listening on {port} @ {baud} baud... (Ctrl+C to stop)")

    try:
        while True:
            data = ser.read(64)
            if not data:
                continue
            module.process(data)
    except KeyboardInterrupt:
        print("\nStopped.")
    finally:
        ser.close()

if __name__ == "__main__":
    main()
