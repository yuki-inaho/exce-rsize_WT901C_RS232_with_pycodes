import time
import logging
import numpy as np
import pytransform3d.visualizer as pv
from scripts.wt901c import WT901C_RS232
from scripts.rotation import euler2mat

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)

PORT = "/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0"
BAUDRATE = 115200
N_ITER = 1000

wt901c = WT901C_RS232(PORT, BAUDRATE)
wt901c.open()

# Initialize angle information (assume sensor is under nearly static state)
print("Start to initialize angle params")
wt901c.set_attachment_direction_vertical()
wt901c.set_frame_rate_100()
wt901c.run_sensor_calibration()
wt901c.set_frame_rate_50()
wt901c.initialize_angle()


def animation_callback(step, n_frames, frame):
    """ @TODO: avoid to use a global variable?
    """

    global wt901c
    while not wt901c.update():
        pass
    A2B = np.eye(4)
    A2B[:3, :3] = euler2mat(np.deg2rad(wt901c.angle_rpy))
    frame.set_data(A2B)
    return frame


fig = pv.figure(width=500, height=500)
frame = fig.plot_basis(R=np.eye(3), s=0.5)
fig.view_init()
if "__file__" in globals():
    fig.animate(animation_callback, N_ITER, fargs=(N_ITER, frame), loop=True)
    fig.show()

wt901c.close()
