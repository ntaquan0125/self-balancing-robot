import sys
import threading
import time
import numpy as np
from PyQt5 import QtBluetooth, QtCore, QtSerialPort, QtWidgets

from GUI import MainWindow
from serial import Serial, PID_TILT, PID_VEL, SAVE_LOAD_PARAMS

__platform__ = sys.platform

WINDOW_WIDTH = 100

class App(Serial):
    def __init__(self):
        super(App, self).__init__()
        self.GUI_init()
        self.function_init()
        self.timer_init(self.update_plot, 100)
        self.show()

    def timer_init(self, func, timeout):
        self.timer = QtCore.QTimer()
        self.timer.timeout.connect(func)
        self.timer.setInterval(timeout)

    def function_init(self):
        # Config port's information
        self.port_list.insertItems(0, [self.get_available_port() + str(i + 1) for i in range(100)])
        self.baudrate_list.insertItems(0, [str(s) for s in self.BAUDRATES])
        self.serial = QtSerialPort.QSerialPort()
        
        # Connect buttons
        self.connect_btn.toggled.connect(self.on_connect)
        self.send_btn.clicked.connect(self.send)
        self.save_btn.clicked.connect(self.save_params)
        self.load_btn.clicked.connect(self.load_params)

        self.rx_data = list()

        # Prepare buffers
        self.t = WINDOW_WIDTH
        self.buffer0 = np.zeros((WINDOW_WIDTH))
        self.buffer1 = np.zeros((WINDOW_WIDTH))
        self.buffer2 = np.zeros((WINDOW_WIDTH))
        self.buffer3 = np.zeros((WINDOW_WIDTH))
        self.buffer4 = np.zeros((WINDOW_WIDTH))
        self.buffer5 = np.zeros((WINDOW_WIDTH))
        self.buffer6 = np.zeros((WINDOW_WIDTH))
        self.buffer7 = np.zeros((WINDOW_WIDTH))
        self.buffer8 = np.zeros((WINDOW_WIDTH))
        self.gyro_X = self.gyro_Y = self.gyro_Z = 0
        self.accel_X = self.accel_Y = self.accel_Z = 0
        self.r = self.p = self.y = 0

    def bluetooth_init(self):
        self.sock = QtBluetooth.QBluetoothSocket(QtBluetooth.QBluetoothServiceInfo.RfcommProtocol)

    def get_available_port(self):
        return {"linux": '/dev/ttyS', "win32": 'COM'}[__platform__]

    @QtCore.pyqtSlot(bool)
    def on_connect(self, checked):
        self.connect_btn.setText('Disconnect' if checked else 'Connect')
        current_port = self.port_list.currentText()
        self.serial = QtSerialPort.QSerialPort(
            current_port,
            baudRate=self.BAUDRATES[self.baudrate_list.currentIndex()],
            readyRead=self.receive
        )
        if (not self.serial.isOpen()) and checked:
            if not self.serial.open(QtCore.QIODevice.ReadWrite):
                self.output_text.append(current_port + ' error')
                self.connect_btn.setChecked(False)
            else:
                self.output_text.append(current_port + ' opened')
                self.timer.start()
        else:
            self.serial.close()
            self.output_text.append(current_port + ' closed')
            self.timer.stop()

    @QtCore.pyqtSlot()
    def receive(self):
        while self.serial.canReadLine():
            text = self.serial.readLine().data()
            # text = text.strip('*\x00').rstrip('\r\n')
            msg = bytearray(text)
            if self.tabs.currentIndex() == 3:
                self.output_text.append(text)
            try:
                # self.rx_data = [float(i) for i in text.split(',')]
                self.message_decode(msg)
            except ValueError as e:
                print(e)

    @QtCore.pyqtSlot()
    def send(self):
        if self.serial.isOpen():
            if self.message_line.text():
                self.tx_data = bytearray((self.message_line.text() + '\r\n').encode())
                self.serial.write(self.tx_data)
                self.output_text.append(self.message_line.text())

    @QtCore.pyqtSlot()
    def save_params(self):
        if self.serial.isOpen():
            self.tx_data, length = self.message_pack(PID_TILT)
            self.serial.write(self.tx_data)
            self.tx_data, length = self.message_pack(PID_VEL)
            self.serial.write(self.tx_data)
            print('Saved!')

    @QtCore.pyqtSlot()
    def load_params(self):
        if self.serial.isOpen():
            self.tx_data, length = self.message_pack(SAVE_LOAD_PARAMS)
            self.serial.write(self.tx_data)
            print('Loaded!')

    @QtCore.pyqtSlot()
    def update_plot(self):
        try:
            self.buffer0[:-1] = self.buffer0[1:]
            self.buffer1[:-1] = self.buffer1[1:]
            self.buffer2[:-1] = self.buffer2[1:]
            self.buffer3[:-1] = self.buffer3[1:]
            self.buffer4[:-1] = self.buffer4[1:]
            self.buffer5[:-1] = self.buffer5[1:]
            self.buffer6[:-1] = self.buffer6[1:]
            self.buffer7[:-1] = self.buffer7[1:]
            self.buffer8[:-1] = self.buffer8[1:]
            self.t += 1
            self.buffer0[-1] = self.gyro_X
            self.buffer1[-1] = self.gyro_Y
            self.buffer2[-1] = self.gyro_Z
            self.buffer3[-1] = self.accel_X
            self.buffer4[-1] = self.accel_Y
            self.buffer5[-1] = self.accel_Z
            self.buffer6[-1] = self.r
            self.buffer7[-1] = self.p
            self.buffer8[-1] = self.y
        except IndexError:
            pass
        finally:
            self.curve0.setData(self.buffer0)
            self.curve1.setData(self.buffer1)
            self.curve2.setData(self.buffer2)
            self.curve3.setData(self.buffer3)
            self.curve4.setData(self.buffer4)
            self.curve5.setData(self.buffer5)
            self.curve6.setData(self.buffer6)
            self.curve7.setData(self.buffer7)
            self.curve8.setData(self.buffer8)
            QtWidgets.QApplication.processEvents()

if __name__ == '__main__':
    app = QtWidgets.QApplication(sys.argv)
    window = App()
    sys.exit(app.exec_())
