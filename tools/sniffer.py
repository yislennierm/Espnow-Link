import serial

def crc8(data):
    crc = 0
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x80:
                crc = ((crc << 1) ^ 0xD5) & 0xFF
            else:
                crc = (crc << 1) & 0xFF
    return crc

ser = serial.Serial("/dev/tty.usbserial-1430", 400000, timeout=0.1)

while True:
    if ser.read(1) == b'\xC8':  # header
        length = ser.read(1)
        if not length:
            continue
        length = length[0]
        frame = ser.read(length)
        if len(frame) != length:
            continue
        payload = frame[:-1]
        crc = frame[-1]
        if crc8(b'\x16' + payload[:-1]) == crc and frame[0] == 0x16:
            # decode channel data
            raw = int.from_bytes(payload[1:-1], 'little')
            channels = [(raw >> (11*i)) & 0x7FF for i in range(16)]
            print(channels)
