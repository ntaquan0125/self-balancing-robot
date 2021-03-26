import struct
from enum import Enum
from GUI import MainWindow
from PyQt5 import QtCore, QtSerialPort

START_FLAG = 0x01
END_FLAG = 0x0A

IMU_GYRO = 0
IMU_GYRO_BIAS = 1
IMU_ACCEL = 2
IMU_ACCEL_BIAS = 3
IMU_RPY = 4
MOTOR_POS = 5
MOTOR_VEL = 6
PID_TILT = 7
PID_VEL = 8
SAVE_LOAD_PARAMS = 9


class Serial(MainWindow):
    BAUDRATES = (
        QtSerialPort.QSerialPort.Baud1200,
        QtSerialPort.QSerialPort.Baud2400,
        QtSerialPort.QSerialPort.Baud4800,
        QtSerialPort.QSerialPort.Baud9600,
        QtSerialPort.QSerialPort.Baud19200,
        QtSerialPort.QSerialPort.Baud38400,
        QtSerialPort.QSerialPort.Baud57600,
        QtSerialPort.QSerialPort.Baud115200,
    )

    def __init__(self):
        super(Serial, self).__init__()

    def message_check(self, msg):
        return msg[0] == START_FLAG

    def message_decode(self, msg):
        if not self.message_check(msg):
            return False

        msg_id = msg[2]
        if msg_id == IMU_GYRO:
            self.gyro_X = struct.unpack('<f', msg[3: 7])[0]
            self.gyro_Y = struct.unpack('<f', msg[7: 11])[0]
            self.gyro_Z = struct.unpack('<f', msg[11: 15])[0]
            return True

        elif msg_id == IMU_GYRO_BIAS: 
            self.gyro_bias_X = struct.unpack('<f', msg[3: 7])[0]
            self.gyro_bias_Y = struct.unpack('<f', msg[7: 11])[0]
            self.gyro_bias_Z = struct.unpack('<f', msg[11: 15])[0]
            return True

        elif msg_id == IMU_ACCEL:
            self.accel_X = struct.unpack('<f', msg[3: 7])[0]
            self.accel_Y = struct.unpack('<f', msg[7: 11])[0]
            self.accel_Z = struct.unpack('<f', msg[11: 15])[0]
            return True

        elif msg_id == IMU_ACCEL_BIAS: 
            self.accel_bias_X = struct.unpack('<f', msg[3: 7])[0]
            self.accel_bias_Y = struct.unpack('<f', msg[7: 11])[0]
            self.accel_bias_Z = struct.unpack('<f', msg[11: 15])[0]
            return True

        elif msg_id == IMU_RPY: 
            self.r = struct.unpack('<f', msg[3: 7])[0]
            self.p = struct.unpack('<f', msg[7: 11])[0]
            self.y = struct.unpack('<f', msg[11: 15])[0]
            self.write_form(self.imu_form, 0, self.r)
            self.write_form(self.imu_form, 1, self.p)
            self.write_form(self.imu_form, 2, self.y)
            return True

        elif msg_id == MOTOR_POS:
            self.position[0] = struct.unpack('<i', msg[3: 7])[0]
            self.position[1] = struct.unpack('<i', msg[7: 11])[0]
            self.write_form(self.encoder_form, 0, self.position[0])
            self.write_form(self.encoder_form, 1, self.position[1])
            return True

        elif msg_id == MOTOR_VEL:
            self.velocity[0] = struct.unpack('<f', msg[3: 7])[0]
            self.velocity[1] = struct.unpack('<f', msg[7: 11])[0]
            return True

        elif msg_id == PID_TILT:
            self.pid_tilt_Kp = struct.unpack('<f', msg[3: 7])[0]
            self.pid_tilt_Ki = struct.unpack('<f', msg[7: 11])[0]
            self.pid_tilt_Kd = struct.unpack('<f', msg[11: 15])[0]
            self.write_form(self.pid1_form, 0, self.pid_tilt_Kp)
            self.write_form(self.pid1_form, 1, self.pid_tilt_Ki)
            self.write_form(self.pid1_form, 2, self.pid_tilt_Kd)
            return True

        elif msg_id == PID_VEL:
            self.pid_vel_Kp = struct.unpack('<f', msg[3: 7])[0]
            self.pid_vel_Ki = struct.unpack('<f', msg[7: 11])[0]
            self.pid_vel_Kd = struct.unpack('<f', msg[11: 15])[0]
            self.write_form(self.pid2_form, 0, self.pid_vel_Kp)
            self.write_form(self.pid2_form, 1, self.pid_vel_Ki)
            self.write_form(self.pid2_form, 2, self.pid_vel_Kd)
            return True

        return False

    def message_pack(self, msg_id):
        msg = QtCore.QByteArray()

        if msg_id == IMU_GYRO:
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_X)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_Y)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_Z)))

        elif msg_id == IMU_GYRO_BIAS:
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_bias_X)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_bias_Y)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.gyro_bias_Z)))

        elif msg_id == IMU_ACCEL:
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_X)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_Y)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_Z)))

        elif msg_id == IMU_ACCEL_BIAS: 
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_bias_X)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_bias_Y)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.accel_bias_Z)))

        elif msg_id == IMU_RPY: 
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.r)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.p)))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.y)))

        elif msg_id == MOTOR_POS:
            msg_len = 8 + 4
            msg.append(QtCore.QByteArray(struct.pack("<i", self.position[0])))
            msg.append(QtCore.QByteArray(struct.pack("<i", self.position[1])))

        elif msg_id == MOTOR_VEL:
            msg_len = 8 + 4
            msg.append(QtCore.QByteArray(struct.pack("<i", self.velocity[0])))
            msg.append(QtCore.QByteArray(struct.pack("<i", self.velocity[1])))

        elif msg_id == PID_TILT:
            msg_len = 12 + 4
            msg.resize(0)
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid1_form, 0))))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid1_form, 1))))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid1_form, 2))))

        elif msg_id == PID_VEL:
            msg_len = 12 + 4
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid2_form, 0))))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid2_form, 1))))
            msg.append(QtCore.QByteArray(struct.pack("<f", self.read_form(self.pid2_form, 2))))

        elif msg_id == SAVE_LOAD_PARAMS:
            msg_len = 1 + 4
            msg.append(bytes([0]))

        else:
            msg_len = 0

        msg.prepend(bytes([msg_id]))
        msg.prepend(bytes([msg_len]))
        msg.prepend(bytes([START_FLAG]))
        msg.append(bytes([END_FLAG]))

        return msg, msg_len
