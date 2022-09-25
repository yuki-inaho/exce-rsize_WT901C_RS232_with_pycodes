import time
import logging
import numpy as np
from scripts.wt901c import WT901C_RS232
from ahrs.filters import Tilt
from scripts.rotation import quat2euler

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
wt901c.set_frame_rate_5()
wt901c.initialize_angle()
tilt_estimator = Tilt(as_angles=True)
tilt_initial = tilt_estimator.estimate(acc=wt901c.acceralation)
for _ in range(N_ITER):
    while not wt901c.update():
        pass
    print(tilt_estimator.estimate(acc=wt901c.acceralation) - tilt_initial)


wt901c.close()
