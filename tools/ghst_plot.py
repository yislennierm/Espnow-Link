

# --- Settings ---
##SERIAL_PORT = "/dev/cu.wchusbserial57290135411"   # change to your ESP32 port
##BAUD_RATE = 420000
##CHANNELS = 8  # decode up to 8 channels
import serial
import re
import pyqtgraph as pg
from pyqtgraph.Qt import QtCore, QtWidgets
import sys

# --- CONFIG ---
SERIAL_PORT = "/dev/tty.wchusbserial1430"  # Change to your ESP32 port
#SERIAL_PORT = "/dev/cu.usbmodem14301"  # Change to your ESP32 port
BAUDRATE = 400000
MAX_POINTS = 50  # How many points to show in the plot window

# --- Serial Setup ---
ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=1)

# --- PyQtGraph Setup ---
app = QtWidgets.QApplication(sys.argv)
win = pg.GraphicsLayoutWidget(show=True, title="GHST Channel Monitor")
win.resize(1000, 600)
plots = []
curves = []
data_buffers = []

NUM_CHANNELS = 8  # Adjust based on how many you expect

for ch in range(NUM_CHANNELS):
    p = win.addPlot(row=ch // 4, col=ch % 4, title=f"Channel {ch+1}")
    curve = p.plot(pen='y')
    p.setYRange(0, 4095)
    plots.append(p)
    curves.append(curve)
    data_buffers.append([])

# Regex to catch lines like: [GHST RAW] len=14: 81 0C 31 40 A3 ...
frame_re = re.compile(r"\[GHST RAW\].*?: (.*)")

def decode_frame(bytes_list):
    """Decode GHST 12-bit channel frame into channel values."""
    # Skip sync/len/type if present
    if len(bytes_list) < 4:
        return []

    # Usually format is: [SYNC][LEN][TYPE][PAYLOAD...][CRC]
    # We only care about PAYLOAD part for channel values
    payload = bytes_list[3:-1]  # drop header + CRC

    chans = []
    bitbuf = 0
    bits = 0
    for b in payload:
        bitbuf |= b << bits
        bits += 8
        while bits >= 12 and len(chans) < NUM_CHANNELS:
            chans.append(bitbuf & 0xFFF)  # 12-bit mask
            bitbuf >>= 12
            bits -= 12
    return chans


def update():
    global data_buffers
    while ser.in_waiting:
        line = ser.readline().decode(errors='ignore').strip()
        m = frame_re.search(line)
        if not m:
            continue
        try:
            bytes_list = [int(x, 16) for x in m.group(1).split()]
        except ValueError:
            continue

        channels = decode_frame(bytes_list)
        for i, val in enumerate(channels):
            data_buffers[i].append(val)
            if len(data_buffers[i]) > MAX_POINTS:
                data_buffers[i] = data_buffers[i][-MAX_POINTS:]
            curves[i].setData(data_buffers[i])

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(50)  # update every 50ms

if __name__ == "__main__":
    QtWidgets.QApplication.instance().exec_()
