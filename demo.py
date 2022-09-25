import time
import logging
from scripts.wt901c import WT901C_RS232


logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200
N_ITER = 1000

wt901c = WT901C_RS232(PORT, BAUDRATE)
wt901c.open()

# Initialize angle information (assume sensor is under stationary state)
print("Start to initialize angle params")
wt901c.set_attachment_direction_vertical()
wt901c.set_frame_rate_100()
wt901c.run_sensor_calibration()
wt901c.set_frame_rate_50()
wt901c.initialize_angle()
for _ in range(N_ITER):
    start = time.time()
    while not wt901c.update():
        pass
    end = time.time()
    # @NOTE: Due to setting
    print(wt901c)
    print("FPS: ", 1.0 / float(end - start))

wt901c.close()
