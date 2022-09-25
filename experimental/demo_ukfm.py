import time
import logging
import ukfm
import matplotlib
import numpy as np
from scipy.linalg import block_diag
from ahrs.filters import Madgwick, Complementary
from ahrs.filters.aqua import adaptive_gain
from scripts.wt901c import WT901C_RS232
from scripts.rotation import quat2euler, mat2euler
from ukfm import ATTITUDE as MODEL

ukfm.utils.set_matplotlib_config()

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200
N_ITER = 1000

wt901c = WT901C_RS232(PORT, BAUDRATE)
wt901c.open()

# Initialize angle information (assume sensor is under stationary state)
wt901c.set_attachment_direction_horisontal()
wt901c.set_frame_rate_100()
wt901c.run_sensor_calibration()
wt901c.initialize_angle()

T = 100
imu_freq = 100

""" ukfm
"""
imu_std = np.array([5 / 180 * np.pi, 0.4, 0.4])  # gyro (rad/s)  # accelerometer (m/s^2)  # magnetometer
model = MODEL(T, imu_freq)
################################################################################
# We initialize the filter with the true state.

state0 = model.STATE(Rot=np.eye(3, dtype=float))
Q = imu_std[0] ** 2 * np.eye(3)
R = block_diag(imu_std[1] ** 2 * np.eye(3), imu_std[2] ** 2 * np.eye(3))
P0 = np.zeros((3, 3))  # The state is perfectly initialized
alpha = np.array([1e-3, 1e-3, 1e-3])
ukf = ukfm.UKF(state0=state0, P0=P0, f=model.f, h=model.h, Q=Q, R=R, phi=model.phi, phi_inv=model.phi_inv, alpha=alpha)

for _ in range(N_ITER):
    while not wt901c.update():
        pass
    ukf.propagation(model.INPUT(wt901c.angular_velocity), model.dt)
    ys = np.append(wt901c.acceralation, wt901c.magnetic)
    ukf.update(ys)
    print(np.rad2deg(mat2euler(ukf.state.Rot)))

wt901c.close()
