import time
import logging
from scripts.wt901c import WT901C_RS232


logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200
N_ITER = 5

wt901c = WT901C_RS232(PORT, BAUDRATE)
wt901c.open()

wt901c.activate_custom_comfiguration()
wt901c.set_frame_rate_5()
for _ in range(N_ITER):
    start = time.time()
    while not wt901c.update():
        pass
    end = time.time()
    print("FPS: ", 1.0 / float(end - start))

wt901c.set_frame_rate_100()
for _ in range(N_ITER):
    start = time.time()
    while not wt901c.update():
        pass
    end = time.time()
    print("FPS: ", 1.0 / float(end - start))

while not wt901c.update():
    pass
wt901c.close()
