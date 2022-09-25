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

# Initialize angle information (assume sensor is under stationary state)
print("Start to initialize angle params")
wt901c.set_frame_rate_100()
while not wt901c.update():
    pass
print(wt901c)

wt901c.run_sensor_calibration()
while not wt901c.update():
    pass
print(wt901c)
wt901c.close()
