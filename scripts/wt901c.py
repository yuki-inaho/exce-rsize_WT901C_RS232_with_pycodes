import time
import serial
import logging
import numpy as np


logger = logging.getLogger("WT901C_RS232")
SUPPORTED_BAUDRATE_LIST = [9600, 115200]

""" @TODO: Update error message
    @TODO: Add mutex lines
	@TODO: asynchronous state update
"""


class WT901C_RS232:
    def __init__(self, port: str, baudrate: int):
        self._ser = serial.Serial()
        self._initialize_serial_communication(port, baudrate)

        """ Initialize AHRS Information
        """
        self._acceralation_x = 0
        self._acceralation_y = 0
        self._acceralation_z = 0

        self._angular_velocity_x = 0
        self._angular_velocity_y = 0
        self._angular_velocity_z = 0

        self._angle_roll = 0
        self._angle_pitch = 0
        self._angle_yaw = 0

        self._magnetic_x = 0
        self._magnetic_y = 0
        self._magnetic_z = 0

        """ Parameter biases
        """
        self._bias_angle_roll = 0
        self._bias_angle_pitch = 0
        self._bias_angle_yaw = 0

    def __str__(self):
        acc_str = f"Accelaration: {self.angular_velocity} [m * s^(-2)]\n"
        ang_str = f"Angular Velocity: {self.acceralation} [rad * s^(-1)]\n"
        magnetic_str = f"Magnetic: {self.magnetic} [T?]\n"
        angle_str = f"Angle: {self.angle_rpy} [deg]\n"
        return acc_str + ang_str + magnetic_str + angle_str

    def _validate_baudrate(self, baud: int):
        if baud not in SUPPORTED_BAUDRATE_LIST:
            raise ValueError("The given baudrate is not supported")

    def _initialize_serial_communication(self, port: str, baudrate: int):
        self._validate_baudrate(baudrate)
        """
            @TODO: impl parity mode
        """
        self._ser.port = port
        self._ser.baudrate = baudrate  # 115200 for JY61 ,9600 for others
        self._ser.bytesize = serial.EIGHTBITS
        self._ser.parity = serial.PARITY_NONE  # set parity check: no parity
        self._ser.stopbits = serial.STOPBITS_ONE  # number of stop bits
        self._ser.timeout = 1  # non-block read
        self._ser.xonxoff = False  # disable software flow control
        self._ser.rtscts = False  # disable hardware (RTS/CTS) flow control
        self._ser.dsrdtr = False  # disable hardware (DSR/DTR) flow control
        self._ser.writeTimeout = 2  # timeout for write

    def open(self):
        if not self._ser.isOpen():
            logging.warn("Sensor is already opened (try to restart?)")
        try:
            self._ser.open()
            self._ser.reset_input_buffer()
            self._ser.reset_output_buffer()
        except Exception as e:
            logging.error(f"Error to open serial port: {str(e)}")

    def close(self):
        """@TODO: confirm the closing status?"""
        try:
            if self._ser.is_open:
                self._ser.close()
        except Exception as e:
            logging.error(f"Error to close serial port: {str(e)}")

    def reset(self):
        """@TODO: add comment"""
        if self._ser.is_open:
            self.close()
            self.open()

    def __del__(self):
        self.close()

    def capture(self):
        return self._ser.read(size=2).hex()

    """ @TODO: unify uppercase and lowercase
    """

    def _parse_acceralation_output(self, read_data):
        header_A_L = int(read_data[0:2], 16)
        header_A_H = int(read_data[2:4], 16)
        AxL = int(read_data[4:6], 16)
        AxH = int(read_data[6:8], 16)
        AyL = int(read_data[8:10], 16)
        AyH = int(read_data[10:12], 16)
        AzL = int(read_data[12:14], 16)
        AzH = int(read_data[14:16], 16)
        TL_A = int(read_data[16:18], 16)
        TH_A = int(read_data[18:20], 16)
        SUM_A = int(read_data[20:22], 16)

        self._acceralation_x = float(np.short((AxH << 8) | AxL) / 32768.0 * 16.0)
        self._acceralation_y = float(np.short((AyH << 8) | AyL) / 32768.0 * 16.0)
        self._acceralation_z = float(np.short((AzH << 8) | AzL) / 32768.0 * 16.0)

    def _parse_angular_velocity_output(self, read_data):
        start_address_2 = int(read_data[22:24], 16)
        start_address_w = int(read_data[24:26], 16)
        wxL = int(read_data[26:28], 16)
        wxH = int(read_data[28:30], 16)
        wyL = int(read_data[30:32], 16)
        wyH = int(read_data[32:34], 16)
        wzL = int(read_data[34:36], 16)
        wzH = int(read_data[36:38], 16)
        TL_w = int(read_data[38:40], 16)
        TH_w = int(read_data[40:42], 16)
        SUM_w = int(read_data[42:44], 16)
        self._angular_velocity_x = float(np.short((wxH << 8) | wxL) / 32768.0 * 2000.0)
        self._angular_velocity_y = float(np.short((wyH << 8) | wyL) / 32768.0 * 2000.0)
        self._angular_velocity_z = float(np.short((wzH << 8) | wzL) / 32768.0 * 2000.0)

    def _parse_anglular_output(self, read_data):
        start_address_3 = int(read_data[44:46], 16)
        start_address_ypr = int(read_data[46:48], 16)
        RollL = int(read_data[48:50], 16)
        RollH = int(read_data[50:52], 16)
        PitchL = int(read_data[52:54], 16)
        PitchH = int(read_data[54:56], 16)
        YawL = int(read_data[56:58], 16)
        YawH = int(read_data[58:60], 16)
        VL = int(read_data[60:62], 16)
        VH = int(read_data[62:64], 16)
        self._angle_roll = float(np.short((RollH << 8) | RollL) / 32768.0 * 180.0)
        self._angle_pitch = float(np.short((PitchH << 8) | PitchL) / 32768.0 * 180.0)
        self._angle_yaw = float(np.short((YawH << 8) | YawL) / 32768.0 * 180.0)

    def _parse_magnetic_output(self, read_data):
        start_address_4 = int(read_data[66:68], 16)
        start_address_mag = int(read_data[68:70], 16)
        HxL = int(read_data[70:72], 16)
        HxH = int(read_data[72:74], 16)
        HyL = int(read_data[74:76], 16)
        HyH = int(read_data[76:78], 16)
        HzL = int(read_data[78:80], 16)
        HzH = int(read_data[80:82], 16)
        TL_mag = int(read_data[82:84], 16)
        TH_mag = int(read_data[84:86], 16)
        SUM_mag = int(read_data[86:88], 16)

        # Magnetic output
        self._magnetic_x = float(np.short(HxH << 8) | HxL)
        self._magnetic_y = float(np.short(HyH << 8) | HyL)
        self._magnetic_z = float(np.short(HzH << 8) | HzL)

    def _parse_data(self, read_data):
        assert len(read_data) == 88
        self._parse_acceralation_output(read_data)
        self._parse_angular_velocity_output(read_data)
        self._parse_anglular_output(read_data)
        self._parse_magnetic_output(read_data)

    def update(self):
        """
        @TODO: Implement Timeout or deadlock behavior handling
        @TODO: Check is_recoding is necessary?
        """
        is_recording = False
        read_data = ""
        if self._ser.is_open:
            while True:
                read_data = self.capture()
                if read_data == "5551":
                    is_recording = True
                    break
                else:
                    self.reset()
                    time.sleep(100 * 10e-6)
            while is_recording:
                captured_data = self.capture()
                if len(captured_data) < 4:
                    return False
                read_data = read_data + captured_data
                if len(read_data) == 88:
                    is_recording = False
            is_recording = False
            self._parse_data(read_data)
            return True
        else:
            return False

    def set_angle_bias(self, bias_angle_roll: float, bias_angle_pitch: float, bias_angle_yaw: float):
        self._bias_angle_roll = bias_angle_roll
        self._bias_angle_pitch = bias_angle_pitch
        self._bias_angle_yaw = bias_angle_yaw

    def initialize_angle(self):
        self.set_angle_bias(*self.angle_rpy)

    @property
    def acceralation(self):
        return np.array([self._acceralation_x, self._acceralation_y, self._acceralation_z]).copy()

    @property
    def angular_velocity(self):
        return np.array([np.deg2rad(angvec) for angvec in [self._angular_velocity_x, self._angular_velocity_y, self._angular_velocity_z]]).copy()

    @property
    def angle_rpy(self):
        return np.array(
            [self._angle_roll - self._bias_angle_roll, self._angle_pitch - self._bias_angle_pitch, self._angle_yaw - self._bias_angle_yaw]
        ).copy()

    @property
    def magnetic(self):
        return np.array([self._magnetic_x, self._magnetic_y, self._magnetic_z]).copy()
