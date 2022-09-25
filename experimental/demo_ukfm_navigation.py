import time
import logging
import ukfm
import matplotlib
import numpy as np
from scipy.linalg import block_diag
from scripts.wt901c import WT901C_RS232
from scripts.rotation import quat2euler, mat2euler
import ukfm

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
imu_std = np.array([0.1, 0.1])  # gyro (rad/s)  # accelerometer (m/s^2)
obs_freq = 1
obs_std = 1

model = ukfm.INERTIAL_NAVIGATION(T, imu_freq)


################################################################################
# We initialize the filter with the true state.

state0 = model.STATE(Rot=np.eye(3, dtype=float), v=np.zeros(3, dtype=float), p=np.zeros(3, dtype=float))
Q = block_diag(imu_std[0] ** 2 * np.eye(3), imu_std[1] ** 2 * np.eye(3))
# measurement noise covariance matrix
R = obs_std**2 * np.eye(3 * model.N_ldk)
# initial uncertainty matrix such that the state is not perfectly initialized
P0 = block_diag((10 * np.pi / 180) ** 2 * np.eye(3), np.zeros((3, 3)), np.eye(3))
# sigma point parameters
alpha = np.array([1e-3, 1e-3, 1e-3])
# start by initializing the filter with an unaccurate state
state0 = model.STATE(Rot=ukfm.SO3.exp(10 * np.pi / 180 * np.ones(3) / 3).dot(state0.Rot), v=state0.v, p=state0.p + np.array([1, 0.5, 0.7]))
# create the UKF
ukf = ukfm.UKF(state0=state0, P0=P0, f=model.f, h=model.h, Q=Q, R=R, phi=model.phi, phi_inv=model.phi_inv, alpha=alpha)
# set variables for recording estimates along the full trajectory
ukf_states = [state0]
ukf_Ps = np.zeros((model.N, 9, 9))
ukf_Ps[0] = P0

for _ in range(N_ITER):
    while not wt901c.update():
        pass
    ukf.propagation(model.INPUT(wt901c.angular_velocity, wt901c.acceralation), model.dt)
    print(np.rad2deg(mat2euler(ukf.state.Rot)), ukf.state.p)

wt901c.close()
