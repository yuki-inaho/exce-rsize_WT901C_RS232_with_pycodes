import time
import logging
import numpy as np
from scripts.wt901c import WT901C_RS232
from ahrs.filters import Madgwick
from scripts.rotation import quat2euler, mat2euler

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200
N_ITER = 10000
FREQUENCY = 100

wt901c = WT901C_RS232(PORT, BAUDRATE)
wt901c.open()

# Initialize angle information (assume sensor is under stationary state)
wt901c.set_attachment_direction_horisontal()
wt901c.set_frame_rate_100()
wt901c.run_sensor_calibration()
wt901c.initialize_angle()

madgwick = Madgwick(frequency=FREQUENCY)
Q = np.array([1.0, 0.0, 0.0, 0.0])
for _ in range(N_ITER):
    start = time.time()
    while not wt901c.update():
        pass
    end = time.time()
    # @NOTE: Due to setting
    print("FPS: ", 1.0 / float(end - start))
    #print(wt901c)
    #Q = madgwick.updateIMU(Q, gyr=wt901c.angular_velocity, acc=wt901c.acceralation)
    Q = madgwick.updateMARG(Q, gyr=wt901c.angular_velocity, acc=wt901c.acceralation, mag=wt901c.magnetic)
    print(np.rad2deg(quat2euler(Q)))

wt901c.close()
