class CRSFModule:
    NAME = "CRSF"

    FRAME_TYPES = {
        0x02: "GPS",
        0x07: "VARIO_SENSOR",
        0x08: "BATTERY_SENSOR",
        0x09: "BARO_ALTITUDE",
        0x0B: "HEARTBEAT",
        0x14: "LINK_STATISTICS",
        0x16: "RC_CHANNELS_PACKED",
        0x17: "SUBSET_RC_CHANNELS",
        0x1C: "LINK_STATISTICS_RX",
        0x1D: "LINK_STATISTICS_TX",
        0x1E: "ATTITUDE",
        0x21: "FLIGHT_MODE",
        0x28: "DEVICE_PING",
        0x29: "DEVICE_INFO",
        0x2B: "PARAM_SETTINGS_ENTRY",
        0x2C: "PARAM_READ",
        0x2D: "PARAM_WRITE",
        0x32: "COMMAND",
        0x34: "DISPLAYPORT_CMD",
    }

    def __init__(self, hex_output=False):
        self.hex_output = hex_output
        self.buffer = bytearray()

    # CRC-8-D5 over [type + payload]
    def crc8(self, data: bytes) -> int:
        crc = 0
        for b in data:
            crc ^= b
            for _ in range(8):
                if crc & 0x80:
                    crc = ((crc << 1) ^ 0xD5) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
        return crc

    def process(self, data: bytes):
        self.buffer.extend(data)

        while True:
            # find next outer CRSF frame anchored on 0xC8 (TX16S -> module line)
            i = self.buffer.find(b"\xC8")
            if i == -1:
                # keep buffer small if it grows
                if len(self.buffer) > 1024:
                    self.buffer.clear()
                return
            if i > 0:
                del self.buffer[:i]  # drop junk before addr

            if len(self.buffer) < 3:
                return

            length = self.buffer[1]
            if length < 2 or length > 250:
                # not a sane CRSF length, skip this 0xC8 and rescan
                del self.buffer[0]
                continue

            total = 2 + length  # addr + len + (type+payload+crc)
            if len(self.buffer) < total:
                return  # wait for more bytes

            frame = bytes(self.buffer[:total])
            del self.buffer[:total]

            ftype = frame[2]
            payload = frame[3:-1]
            crc = frame[-1]

            if self.crc8(frame[2:-1]) != crc:
                # bad CRC: skip this 0xC8 and try to resync to the next one
                if self.buffer:
                    del self.buffer[0]
                continue

            name = self.FRAME_TYPES.get(ftype, f"UNKNOWN_0x{ftype:02X}")
            preview = " ".join(f"{b:02X}" for b in payload[:8])
            print(f"[CRSF-OUT] type=0x{ftype:02X} ({name}) len={len(payload)} data={preview}...")

            # classic RC channels (rare on this line, but support it)
            if ftype == 0x16 and len(payload) == 22:
                self.decode_channels(payload)
                continue

            # ELRS wrapper: unpack embedded subframes carried inside
            if ftype == 0xA9:
                self.unpack_subframes(payload)

    def unpack_subframes(self, blob: bytes):
        """Parse sequential embedded CRSF frames inside an ELRS wrapper (e.g., type 0xA9)."""
        j = 0
        n = len(blob)
        while j + 2 <= n:
            addr = blob[j]
            if j + 1 >= n:
                break
            length = blob[j + 1]
            if length < 2 or length > 250:
                j += 1
                continue
            end = j + 2 + length
            if end > n:
                # incomplete, stop until more outer bytes arrive
                break

            inner = blob[j:end]
            itype = inner[2]
            ipayload = inner[3:-1]
            icrc = inner[-1]

            if self.crc8(inner[2:-1]) == icrc:
                iname = self.FRAME_TYPES.get(itype, f"UNKNOWN_0x{itype:02X}")
                iprev = " ".join(f"{b:02X}" for b in ipayload[:8])
                print(f"  [SUB] addr=0x{addr:02X} type=0x{itype:02X} ({iname}) len={len(ipayload)} data={iprev}...")

                if itype == 0x16 and len(ipayload) == 22:
                    self.decode_channels(ipayload, indent="    ")

                j = end  # consume this inner frame completely
            else:
                j += 1  # slide forward to find the next valid inner frame

    def decode_channels(self, payload: bytes, indent: str = ""):
        # 16 channels, 11-bit packed, little-endian (22 bytes total)
        bits = int.from_bytes(payload, "little")
        raw = [((bits >> (11 * i)) & 0x7FF) for i in range(16)]
        # Map to ~988–2012µs
        us = [988 + ((v - 172) * (2012 - 988)) // (1811 - 172) for v in raw]
        print(indent + " ".join(f"CH{i+1}:{v}" for i, v in enumerate(us)))
