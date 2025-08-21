#!/usr/bin/env python3
import argparse, time, sys
import serial
from serial.tools import list_ports

# -------- CRC-8 (0xD5) over [type + payload] for sanity checks (optional) -----
def crsf_crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

def list_serials():
    ports = list(list_ports.comports())
    for i, p in enumerate(ports):
        print(f"[{i}] {p.device} — {p.description or ''}")
    return ports

def autobaud_detect(port: str, candidates, seconds_per=0.8):
    """
    Probe candidate baud rates and score by counting valid CRSF frames
    (0xC8 addr, length [2..250], CRC8-DVB-S2 over [type+payload]).
    Returns (best_baud, hits).
    """
    best_baud, best_hits = None, 0
    for baud in candidates:
        hits = 0
        try:
            with serial.Serial(
                port=port,
                baudrate=baud,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.02,
            ) as ser:
                ser.reset_input_buffer()
                start = time.time()
                buf = bytearray()
                while time.time() - start < seconds_per:
                    buf.extend(ser.read(256))
                    # anchor on 0xC8 frames for a quick validity score
                    while True:
                        i = buf.find(b"\xC8")
                        if i == -1 or len(buf) - i < 5:
                            # keep a little tail
                            if len(buf) > 1024:
                                del buf[:-16]
                            break
                        if i > 0:
                            del buf[:i]
                        if len(buf) < 3:
                            break
                        length = buf[1]
                        if length < 2 or length > 250:
                            del buf[0]
                            continue
                        total = 2 + length
                        if len(buf) < total:
                            break
                        frame = bytes(buf[:total])
                        del buf[:total]
                        if crsf_crc8(frame[2:-1]) == frame[-1]:
                            hits += 1
        except Exception:
            pass
        print(f"[AUTOBAUD] {baud}: {hits} valid frames")
        if hits > best_hits:
            best_hits, best_baud = hits, baud
    return best_baud, best_hits

def main():
    ap = argparse.ArgumentParser(description="Capture serial to .bin and hex-only bytes (no timestamps)")
    ap.add_argument("-p","--port", help="Serial port (e.g. /dev/ttyUSB0 or /dev/cu.usbserial-XXXX)")
    ap.add_argument("-b","--baud", type=int, help="Baudrate (e.g. 400000)")
    ap.add_argument("-a","--autobaud", action="store_true", help="Try common bauds and choose best")
    ap.add_argument("--baud-candidates", default="400000,420000,115200,250000,460800,921600",
                    help="Comma-separated baud list for autobaud")
    ap.add_argument("-d","--duration", type=float, default=20.0, help="Capture seconds (default 20)")
    ap.add_argument("-o","--out-bin", default="capture.bin", help="Output binary file (raw byte stream)")
    ap.add_argument("-x","--out-hex", default=None,
                    help="(Optional) Output hex+timestamp file (legacy format). If omitted, this file is not written.")
    ap.add_argument("-X","--out-hex-bytes", default="capture_bytes.hex",
                    help="Output pure hex bytes (no timestamps), separated by spaces/newlines")
    ap.add_argument("--bytes-per-line", type=int, default=32,
                    help="Bytes per line for --out-hex-bytes (0 = single long line)")
    ap.add_argument("-c","--chunk", type=int, default=512, help="Read chunk size (default 512)")
    args = ap.parse_args()

    port = args.port
    if not port:
        print("No port specified. Available devices:")
        ports = list_serials()
        if not ports:
            print("No serial devices found."); sys.exit(1)
        sel = input("Select device index: ").strip()
        if not sel.isdigit() or not (0 <= int(sel) < len(ports)):
            print("Invalid selection."); sys.exit(1)
        port = ports[int(sel)].device

    baud = args.baud
    if args.autobaud or not baud:
        cands = [int(x.strip()) for x in args.baud_candidates.split(",") if x.strip()]
        print(f"[AUTOBAUD] Trying {cands} …")
        best, hits = autobaud_detect(port, cands)
        if best:
            print(f"[AUTOBAUD] Selected {best} (hits={hits})")
            baud = best
        else:
            print("[AUTOBAUD] No valid frames detected; please set --baud explicitly.")
            sys.exit(2)

    print(f"[CAPTURE] Port={port} Baud={baud} Duration={args.duration}s")
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baud,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.05,
        )
    except Exception as e:
        print(f"Failed to open serial: {e}")
        sys.exit(3)

    t0 = time.time()
    total = 0
    bytes_per_line = max(0, int(args.bytes_per_line))
    hex_bytes_written = 0
    hexline_count = 0

    with open(args.out_bin, "wb") as fbin, \
         (open(args.out_hex, "w") if args.out_hex else open('/dev/null', 'w')) as fhex_ts, \
         open(args.out_hex_bytes, "w") as fhex_bytes:
        if args.out_hex:
            fhex_ts.write("# ts_ms  offset  bytes(hex)\n")
        try:
            bytes_in_line = 0
            while True:
                if time.time() - t0 >= args.duration:
                    break
                data = ser.read(args.chunk)
                if not data:
                    continue
                now_ms = int((time.time() - t0) * 1000)
                # write raw
                fbin.write(data)
                total += len(data)

                # optional legacy ts+hex (unchanged format)
                if args.out_hex:
                    hexline = " ".join(f"{b:02x}" for b in data)
                    fhex_ts.write(f"{now_ms:08d}  {total - len(data):08d}  {hexline}\n")

                # required: pure hex bytes (no timestamps). One space between bytes, wrap at N bytes per line.
                for b in data:
                    if bytes_per_line and bytes_in_line >= bytes_per_line:
                        fhex_bytes.write("\n")
                        hexline_count += 1
                        bytes_in_line = 0
                    if bytes_in_line:
                        fhex_bytes.write(" ")
                    fhex_bytes.write(f"{b:02x}")
                    bytes_in_line += 1
                    hex_bytes_written += 1
            # final newline for hex-bytes file if we wrote anything and didn't end on a newline
            if hex_bytes_written and (not bytes_per_line or bytes_in_line):
                fhex_bytes.write("\n")
                hexline_count += 1
        except KeyboardInterrupt:
            pass
    ser.close()
    print(f"[CAPTURE] Done. Wrote {total} bytes → {args.out_bin}")
    if args.out_hex:
        print(f"[CAPTURE] Timestamped hex written → {args.out_hex}")
    print(f"[CAPTURE] Pure hex bytes written → {args.out_hex_bytes} "
          f"({hex_bytes_written} bytes, {hexline_count} lines, wrap={bytes_per_line or 'no wrap'})")

if __name__ == "__main__":
    main()
