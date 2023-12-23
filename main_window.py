from ui import Ui_MainWindow
import json
import serial
import serial.tools.list_ports

import pyqtgraph as pg
import re

from PyQt6 import *

from PyQt6.QtCore import *
from PyQt6.QtGui import *
from PyQt6.QtWidgets import *

from matplotlib.backends.backend_qtagg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

CONFIG_FILE = "config.json"

with open(CONFIG_FILE, "r") as f:
    config_dict: dict = json.load(f)

t = []
y = []

baudrate = 9600
timeout = 1

PORT_CH340 = 'COM12'
PORT_CUSTOM_IN = 'COM8'
PORT_CUSTOM_OUT = 'COM9'

ser: serial.Serial = None
tx_cnt = 0
rx_cnt = 0
ptr = 0

channel_list = [[] for i in range(7)]
channel_cnt = 0

gyro_roll = 0.0
gyro_pitch = 0.0
inner_pitch_lasterror = 0.0
inner_roll_lasterror = 0.0
inner_pitch_integrator = 0.0
inner_roll_integrator = 0.0
inner_pitch_output = 0.0
inner_roll_output = 0.0


read_pitch = 0.0
read_roll = 0.0
read_yaw = 0.0
rx_yaw = 0.0
rx_pitch = 0.0
rx_roll = 0.0
rx_updown = 0.0
outer_pitch_lasterror = 0.0
outer_roll_lasterror = 0.0
outer_pitch_integrator = 0.0
outer_roll_integrator = 0.0
outer_pitch_output = 0.0
outer_roll_output = 0.0

motor1 = 0.0
motor2 = 0.0
motor3 = 0.0
motor4 = 0.0

inner_kp = 0.0
inner_ki = 0.0
inner_kd = 0.0

outer_kp = 0.0
outer_ki = 0.0
outer_kd = 0.0

kalman_time = 0
send_time = 0
receive_time = 0
inner_loop_time = 0
motor_time = 0
outer_loop_time = 0

##################################################
# Task
##################################################
task_list = []

count63 = 0
count62 = 0
count61 = 0
count3 = 0
count4 = 0

t2 = []
y2 = []
ptr2 = 0

switch_cnt = 0

task_dict = {
    63: "idle",
    62: "stat",
    61: "unknown",
    3: "led_on",
    4: "led_off",
}

convert_dict = {
    63: 0,
    62: 1,
    61: 2,
    3: 3,
    4: 4,
}

show_list = [
    (0, 'idle'),
    (1, 'stat'),
    (2, 'unknown'),
    (3, 'led_on'),
    (4, 'led_off')]

##################################################
#
#
#                 main window
#
#
##################################################


class MainWindow(QMainWindow):
    ui: Ui_MainWindow
    threads: list[QThread]

    def __init__(self):
        super().__init__()
        self.threads = []
        self.bytes = []

        # Set UI
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        # Stacked Widget
        self.ui.pushButton_usart.clicked.connect(
            lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.pushButton_chan.clicked.connect(
            lambda: self.ui.stackedWidget.setCurrentIndex(1))
        self.ui.pushButton_pid.clicked.connect(
            lambda: self.ui.stackedWidget.setCurrentIndex(2))
        self.ui.pushButton_task.clicked.connect(
            lambda: self.ui.stackedWidget.setCurrentIndex(3))
        self.ui.pushButton_3d.clicked.connect(
            lambda: self.ui.stackedWidget.setCurrentIndex(4))
        self.ui.pushButton_3d.clicked.connect(
            lambda: self.ui.openGLWidget.update())

        # Set Current Index
        self.ui.stackedWidget.setCurrentIndex(
            int(config_dict['stackedWidget_index']))

        # timer to count tx rx
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.tx_rx_cnt)
        self.timer.start(100)

        # start && stop
        self.ui.pushButton_start.clicked.connect(self.usart_start_pressed)
        self.ui.pushButton_clear.clicked.connect(
            lambda: self.ui.textBrowser_usart.clear())

        self.ui.textBrowser_usart.document().setMaximumBlockCount(1000)

        # usart combox
        self.ui.comboBox_port.addItems(
            [ser.description for ser in serial.tools.list_ports.comports()])
        for i in range(self.ui.comboBox_port.count()):
            if "CH340" in self.ui.comboBox_port.itemText(i):
                self.ui.comboBox_port.setCurrentIndex(i)
                break

        self.ui.pushButton_open.clicked.connect(self.open_close)
        self.ui.pushButton_open2.clicked.connect(self.open_close)
        self.ui.pushButton_open3.clicked.connect(self.open_close)
        self.ui.pushButton_open4.clicked.connect(self.open_close)
        self.ui.pushButton_open5.clicked.connect(self.open_close)

        self.ui.pushButton_pid_send.clicked.connect(self.pid_send)
        
        self.ui.textBrowser_pid.setText(config_dict['pid'])
        
        self.ui.textBrowser_pid.document().setMaximumBlockCount(20)
        self.ui.textBrowser_pid.moveCursor(QTextCursor.MoveOperation.End)
        
        self.ui.pushButton_dot1.clicked.connect(self.setDot)
        self.ui.pushButton_dot2.clicked.connect(self.setDot)
        self.ui.pushButton_dot3.clicked.connect(self.setDot)
        self.ui.pushButton_dot4.clicked.connect(self.setDot)
        
        self.ui.pushButton_dot1.setChecked(True)

        ##############################################################
        # Channel View
        ##############################################################
        pg.setConfigOptions(antialias=True)  # 抗锯齿

        self.plot = pg.PlotWidget(enableAutoRange=False)  # 启用自动缩放

        self.plot.addLegend(size=(60, 20), offset=(0, -200))
        self.plot.showGrid(x=True, y=True, alpha=0.5)  # 显示网格

        self.ui.verticalLayout_chan.addWidget(self.plot)

        global t, y
        self.curve1 = pg.PlotDataItem(t, y, name='chan1', pen='g')
        self.curve2 = pg.PlotDataItem(t, y, name='chan2', pen='r')
        self.curve3 = pg.PlotDataItem(t, y, name='chan3', pen='b')
        self.curve4 = pg.PlotDataItem(t, y, name='chan4', pen='y')
        self.plot.addItem(self.curve1)
        self.plot.addItem(self.curve2)
        self.plot.addItem(self.curve3)
        self.plot.addItem(self.curve4)

        ##############################################################
        # Task View
        ##############################################################
        straxis = pg.AxisItem(orientation='left')
        straxis.setTicks([show_list])
        self.plot2 = pg.PlotWidget(
            enableAutoRange=True, axisItems={'left': straxis})
        self.plot2.setYRange(0, 5, padding=0)
        # self.plot.setXRange(0, 5, padding=0)
        self.ui.verticalLayout_taskView.addWidget(self.plot2)

        global t2, y2
        self.curve22 = pg.PlotDataItem(t2, y2, stepMode=True)
        self.plot2.addItem(self.curve22)

        ##############################################################
        # Timer
        ##############################################################

        self.timer2 = QTimer(self)
        self.timer2.timeout.connect(self.update1)
        self.timer2.start(50)

        self.timer_pid = QTimer(self)
        self.timer_pid.timeout.connect(self.update_pid)
        self.timer_pid.start(50)
        
        ##############################################################
        # 3D
        ##############################################################
        self.figure_pitch = Figure()
        self.canvas_pitch = FigureCanvas(self.figure_pitch)
        self.ui.verticalLayout_graph_pitch.addWidget(self.canvas_pitch)

        self.axes_pitch = self.figure_pitch.add_subplot(111)
        self.axes_pitch.set_title('PID Control')
        self.axes_pitch.set_xlabel('Time')
        self.axes_pitch.set_ylabel('Pitch')
        
        # self.figure_roll = Figure()
        # self.canvas_roll = FigureCanvas(self.figure_roll)
        # self.ui.verticalLayout_graph_roll.addWidget(self.canvas_roll)
        
        # self.axes_roll = self.figure_roll.add_subplot(111)
        # self.axes_roll.set_title('PID Control')
        # self.axes_roll.set_xlabel('Time')
        # self.axes_roll.set_ylabel('Roll')
        
        self.figure_gyrox = Figure()
        self.canvas_gyrox = FigureCanvas(self.figure_gyrox)
        self.ui.verticalLayout_graph_gyrox.addWidget(self.canvas_gyrox)
        
        self.axes_gyrox = self.figure_gyrox.add_subplot(111)
        self.axes_gyrox.set_title('PID Control')
        self.axes_gyrox.set_xlabel('Time')
        self.axes_gyrox.set_ylabel('GyroX')
        
        # self.figure_gyroy = Figure()
        # self.canvas_gyroy = FigureCanvas(self.figure_gyroy)
        # self.ui.verticalLayout_graph_gyroy.addWidget(self.canvas_gyroy)
        
        # self.axes_gyroy = self.figure_gyroy.add_subplot(111)
        # self.axes_gyroy.set_title('PID Control')
        # self.axes_gyroy.set_xlabel('Time')
        # self.axes_gyroy.set_ylabel('GyroY')
        
        self.pitch_list = []
        self.readpitch_list = []
        self.roll_list = []
        self.x_list = []
        self.gyrox_list = []
        self.gyroy_list = []
        self.pitchoutput_list = []
        self.rolloutput_list = []
        
        self.run_flag = True
       
    def setDot(self):
        sender = self.sender()
        if sender == self.ui.pushButton_dot1:
            step = 0.1
            self.ui.pushButton_dot1.setChecked(True)
            self.ui.pushButton_dot2.setChecked(False)
            self.ui.pushButton_dot3.setChecked(False)
            self.ui.pushButton_dot4.setChecked(False)
        elif sender == self.ui.pushButton_dot2:
            step = 0.01
            self.ui.pushButton_dot1.setChecked(False)
            self.ui.pushButton_dot2.setChecked(True)
            self.ui.pushButton_dot3.setChecked(False)
            self.ui.pushButton_dot4.setChecked(False)
        elif sender == self.ui.pushButton_dot3:
            step = 0.001
            self.ui.pushButton_dot1.setChecked(False)
            self.ui.pushButton_dot2.setChecked(False)
            self.ui.pushButton_dot3.setChecked(True)
            self.ui.pushButton_dot4.setChecked(False)
        elif sender == self.ui.pushButton_dot4:
            step = 0.0001
            self.ui.pushButton_dot1.setChecked(False)
            self.ui.pushButton_dot2.setChecked(False)
            self.ui.pushButton_dot3.setChecked(False)
            self.ui.pushButton_dot4.setChecked(True)
        self.ui.doubleSpinBox_outer_p.setSingleStep(step)
        self.ui.doubleSpinBox_outer_i.setSingleStep(step)
        self.ui.doubleSpinBox_outer_d.setSingleStep(step)
        self.ui.doubleSpinBox_inner_p.setSingleStep(step)
        self.ui.doubleSpinBox_inner_i.setSingleStep(step)
        self.ui.doubleSpinBox_inner_d.setSingleStep(step)

    def open_close(self):
        global ser
        current = self.ui.comboBox_port.currentText()
        if self.ui.pushButton_open.text() == "打开串口":

            # pattern_port = r'(.*)\((.*)\)'
            # match = re.match(pattern_port, current)
            # if match:
                # current = match.group(2)
            try:
                ser = serial.Serial("/dev/tty.wchusbserial140", baudrate, timeout=timeout)
                ser.baudrate = 115200
                ser.bytesize = serial.EIGHTBITS
                ser.stopbits = serial.STOPBITS_ONE
                ser.parity = serial.PARITY_NONE
                if ser.isOpen():
                    print("[open success]")
                # init usart thread
                    self.usart_thread = UsartThread()
                    self.usart_thread.msg.connect(self.usart_msg)
                    self.usart_thread.start()
                    self.threads.append(self.usart_thread)
                    self.usart_start_pressed()
                    self.ui.pushButton_open.setText("关闭串口")
                    self.ui.pushButton_open2.setText("关闭串口")
                    self.ui.pushButton_open3.setText("关闭串口")
                    self.ui.pushButton_open4.setText("关闭串口")
                    self.ui.pushButton_open5.setText("关闭串口")
                    self.run_flag = True
                else:
                    print("[open false]")
            except:
                print("[open false]")
        else:
            self.ui.pushButton_start.setText("Start")
            self.usart_thread.stop()
            ser.close()
            self.ui.pushButton_open.setText("打开串口")
            self.ui.pushButton_open2.setText("打开串口")
            self.ui.pushButton_open3.setText("打开串口")
            self.ui.pushButton_open4.setText("打开串口")
            self.ui.pushButton_open5.setText("打开串口")
            self.curve1.clear()
            global t, y, ptr
            t = []
            y = []
            ptr = 0
            global channel_list
            channel_list = [[] for i in range(7)]
            self.run_flag = False
            self.axes_pitch.clear()
            # self.axes_roll.clear()
            self.pitch_list = []
            self.readpitch_list = []
            self.roll_list = []
            self.x_list = []
            self.gyrox_list = []
            self.gyroy_list = []
            self.pitchoutput_list = []
            self.rolloutput_list = []
            
            

    def pid_send(self):
        inner_p = self.ui.doubleSpinBox_inner_p.value()
        inner_i = self.ui.doubleSpinBox_inner_i.value()
        inner_d = self.ui.doubleSpinBox_inner_d.value()
        outer_p = self.ui.doubleSpinBox_outer_p.value()
        outer_i = self.ui.doubleSpinBox_outer_i.value()
        outer_d = self.ui.doubleSpinBox_outer_d.value()
        data = "[pid:%.4f;%.4f;%.4f;%.4f;%.4f;%.4f]\n" % (
            inner_p, inner_i, inner_d, outer_p, outer_i, outer_d)
        thread = SendThread(data)
        thread.start()
        self.threads.append(thread)
        data_reverse = "[pid:%.4f;%.4f;%.4f;%.4f;%.4f;%.4f]\n" % (
            outer_p, outer_i, outer_d, inner_p, inner_i, inner_d)
        self.ui.textBrowser_pid.insertPlainText(data_reverse)
        self.ui.textBrowser_pid.moveCursor(QTextCursor.MoveOperation.End)
        config_dict['pid'] = self.ui.textBrowser_pid.toPlainText()
        with open(CONFIG_FILE, "w") as f:
            json.dump(config_dict, f, indent=4)

    def tx_rx_cnt(self):
        global tx_cnt
        global rx_cnt

        self.ui.statusbar.showMessage(
            "Tx: " + str(tx_cnt) + " Rx: " + str(rx_cnt))

    def format_float(self, f):
        if f >= 0:
            return "+" + format(f, '10.2f')
        else:
            return "-" + format(-f, '10.2f')
        
    def format_float2(self, f):
        if f >= 0:
            return " " + format(f, '6.2f')
        else:
            return "-" + format(-f, '6.2f')
        
    def format_float3(self, f):
        if f >= 0:
            return " " + format(f, '6.4f')
        else:
            return "-" + format(-f, '6.4f')

    def update_pid(self):
        if self.run_flag is False:
            return
            
        global gyro_roll, gyro_pitch, inner_pitch_lasterror, inner_roll_lasterror, inner_pitch_integrator, inner_roll_integrator, inner_pitch_output, inner_roll_output, motor1, motor2, motor3, motor4, read_pitch, read_roll, read_yaw, rx_yaw, rx_pitch, rx_roll, rx_updown, outer_pitch_lasterror, outer_roll_lasterror, outer_pitch_integrator, outer_roll_integrator, outer_pitch_output, outer_roll_output

        self.ui.label_gyro_roll.setText(self.format_float(gyro_roll))
        self.ui.label_gyro_pitch.setText(self.format_float(gyro_pitch))
        self.ui.label_inner_pitch_lasterror.setText(
            self.format_float(inner_pitch_lasterror))
        self.ui.label_inner_roll_lasterror.setText(
            self.format_float(inner_roll_lasterror))
        self.ui.label_inner_pitch_integrator.setText(
            self.format_float(inner_pitch_integrator))
        self.ui.label_inner_roll_integrator.setText(
            self.format_float(inner_roll_integrator))
        self.ui.label_inner_pitch_output.setText(
            self.format_float(inner_pitch_output))
        self.ui.label_inner_roll_output.setText(
            self.format_float(inner_roll_output))

        self.ui.label_pitch.setText(self.format_float(read_pitch))
        self.ui.label_roll.setText(self.format_float(read_roll))
        self.ui.label_yaw.setText(self.format_float(read_yaw))
        self.ui.label_rx_yaw.setText(self.format_float(rx_yaw))
        self.ui.label_rx_pitch.setText(self.format_float(rx_pitch))
        self.ui.label_rx_roll.setText(self.format_float(rx_roll))
        self.ui.label_rx_updown.setText(self.format_float(rx_updown))
        self.ui.label_outer_pitch_lasterror.setText(
            self.format_float(outer_pitch_lasterror))
        self.ui.label_outer_roll_lasterror.setText(
            self.format_float(outer_roll_lasterror))
        self.ui.label_outer_pitch_integrator.setText(
            self.format_float(outer_pitch_integrator))
        self.ui.label_outer_roll_integrator.setText(
            self.format_float(outer_roll_integrator))
        self.ui.label_outer_pitch_output.setText(
            self.format_float(outer_pitch_output))
        self.ui.label_outer_roll_output.setText(
            self.format_float(outer_roll_output))

        self.ui.lcdNumber_1.display(format(motor1, '.2f'))
        self.ui.lcdNumber_2.display(format(motor2, '.2f'))
        self.ui.lcdNumber_3.display(format(motor3, '.2f'))
        self.ui.lcdNumber_4.display(format(motor4, '.2f'))
        
        self.ui.label_motor1.setText("%s - %s - %s = %s" % (
            self.format_float(rx_updown), self.format_float(inner_pitch_output), self.format_float(inner_roll_output), self.format_float(motor1)))
        self.ui.label_motor2.setText("%s - %s + %s = %s" % (
            self.format_float(rx_updown), self.format_float(inner_pitch_output), self.format_float(inner_roll_output), self.format_float(motor2)))
        self.ui.label_motor3.setText("%s + %s + %s = %s" % (
            self.format_float(rx_updown), self.format_float(inner_pitch_output), self.format_float(inner_roll_output), self.format_float(motor3)))
        self.ui.label_motor4.setText("%s + %s - %s = %s" % (
            self.format_float(rx_updown), self.format_float(inner_pitch_output), self.format_float(inner_roll_output), self.format_float(motor4)))

        self.ui.label_inner_p.setText(self.format_float3(inner_kp))
        self.ui.label_inner_i.setText(self.format_float3(inner_ki))
        self.ui.label_inner_d.setText(self.format_float3(inner_kd))

        self.ui.label_outer_p.setText(self.format_float3(outer_kp))
        self.ui.label_outer_i.setText(self.format_float3(outer_ki))
        self.ui.label_outer_d.setText(self.format_float3(outer_kd))

        self.ui.label_outer_pitch_output_cal.setText(
            "%s*%s(%s) + %s*%s(%s) + %s*%s(%s) = %s" % (
                self.format_float2(outer_kp),
                self.format_float2((outer_pitch_output - outer_ki * outer_pitch_integrator - outer_kd * outer_pitch_lasterror) / outer_kp if outer_kp != 0 else 0),
                self.format_float2((outer_pitch_output - outer_ki * outer_pitch_integrator - outer_kd * outer_pitch_lasterror)),
                self.format_float2(outer_ki), self.format_float2(outer_pitch_integrator), self.format_float2(outer_ki * outer_pitch_integrator),
                self.format_float2(outer_kd), self.format_float2(outer_pitch_lasterror), self.format_float2(outer_kd * outer_pitch_lasterror),
                self.format_float2(outer_pitch_output)))
        self.ui.label_outer_roll_output_cal.setText(
            "%s*%s(%s) + %s*%s(%s) + %s*%s(%s) = %s" % (
                self.format_float2(outer_kp),
                self.format_float2((outer_roll_output - outer_ki * outer_roll_integrator - outer_kd * outer_roll_lasterror) / outer_kp if outer_kp != 0 else 0),
                self.format_float2((outer_roll_output - outer_ki * outer_roll_integrator - outer_kd * outer_roll_lasterror)),
                self.format_float2(outer_ki), self.format_float2(outer_roll_integrator), self.format_float2(outer_ki * outer_roll_integrator),
                self.format_float2(outer_kd), self.format_float2(outer_roll_lasterror), self.format_float2(outer_kd * outer_roll_lasterror),
                self.format_float2(outer_roll_output)))
        self.ui.label_inner_pitch_output_cal.setText(
            "%s*%s(%s) + %s*%s(%s) + %s*%s(%s) = %s" % (
                self.format_float2(inner_kp),
                self.format_float2((inner_pitch_output - inner_ki * inner_pitch_integrator - inner_kd * inner_pitch_lasterror) / inner_kp if inner_kp != 0 else 0),
                self.format_float2((inner_pitch_output - inner_ki * inner_pitch_integrator - inner_kd * inner_pitch_lasterror)),
                self.format_float2(inner_ki), self.format_float2(inner_pitch_integrator), self.format_float2(inner_ki * inner_pitch_integrator),
                self.format_float2(inner_kd), self.format_float2(inner_pitch_lasterror), self.format_float2(inner_kd * inner_pitch_lasterror),
                self.format_float2(inner_pitch_output)))
        self.ui.label_inner_roll_output_cal.setText(
            "%s*%s(%s) + %s*%s(%s) + %s*%s(%s) = %s" % (
                self.format_float2(inner_kp),
                self.format_float2((inner_roll_output - inner_ki * inner_roll_integrator - inner_kd * inner_roll_lasterror) / inner_kp if inner_kp != 0 else 0),
                self.format_float2((inner_roll_output - inner_ki * inner_roll_integrator - inner_kd * inner_roll_lasterror)),
                self.format_float2(inner_ki), self.format_float2(inner_roll_integrator), self.format_float2(inner_ki * inner_roll_integrator),
                self.format_float2(inner_kd), self.format_float2(inner_roll_lasterror), self.format_float2(inner_kd * inner_roll_lasterror),
                self.format_float2(inner_roll_output)))
        # if self.ui.doubleSpinBox_outer_p.value() == 0:
        #     self.ui.doubleSpinBox_outer_p.setValue(outer_kp)
        #     self.ui.doubleSpinBox_outer_i.setValue(outer_ki)
        #     self.ui.doubleSpinBox_outer_d.setValue(outer_kd)
        #     self.ui.doubleSpinBox_inner_p.setValue(inner_kp)
        #     self.ui.doubleSpinBox_inner_i.setValue(inner_ki)
        #     self.ui.doubleSpinBox_inner_d.setValue(inner_kd)
        self.ui.label_kalman_time.setText("%d" % (kalman_time))
        self.ui.label_send_time.setText("%d" % (send_time))
        self.ui.label_receive_time.setText("%d" % (receive_time))
        self.ui.label_inner_loop_time.setText("%d" % (inner_loop_time))
        self.ui.label_motor_time.setText("%d" % (motor_time))
        self.ui.label_outer_loop_time.setText("%d" % (outer_loop_time))
        
        length = 200
        
        if len(self.pitch_list) > length:
            self.pitch_list.pop(0)
        self.pitch_list.append(read_pitch)
        if len(self.readpitch_list) > length:
            self.readpitch_list.pop(0)
        self.readpitch_list.append(rx_pitch)
        self.x_list = [i for i in range(len(self.pitch_list))]
        self.axes_pitch.clear()
        self.axes_pitch.plot(self.x_list, self.pitch_list, 'b', label='Pitch')
        
        # self.axes_pitch.axhline(y=0, color='r', linestyle='--')
        self.axes_pitch.plot(self.x_list, self.readpitch_list, 'r', label='ReadPitch')
        self.axes_pitch.legend()
        self.axes_pitch.grid()
        self.axes_pitch.set_ylim([-20, 20])
        self.canvas_pitch.draw()
        
        # if len(self.roll_list) > 50:
        #     self.roll_list.pop(0)
        # self.roll_list.append(read_roll)
        # self.x_list = [i for i in range(len(self.roll_list))]
        # self.axes_roll.clear()
        # self.axes_roll.plot(self.x_list, self.roll_list, 'b', label='Roll')
        # # self.axes_roll.legend()
        # self.axes_roll.axhline(y=0, color='r', linestyle='--')
        # self.axes_roll.grid()
        # self.axes_roll.set_ylim([-40, 40])
        # self.canvas_roll.draw()
        
        if len(self.gyrox_list) > length:
            self.gyrox_list.pop(0)
        self.gyrox_list.append(gyro_pitch)
        if len(self.pitchoutput_list) > length:
            self.pitchoutput_list.pop(0)
        self.pitchoutput_list.append(outer_pitch_output)
        self.x_list = [i for i in range(len(self.gyrox_list))]
        self.axes_gyrox.clear()
        self.axes_gyrox.plot(self.x_list, self.gyrox_list, 'b', label='GyroP')
        
        self.axes_gyrox.plot(self.x_list, self.pitchoutput_list, 'r', label='OuterPOut')
        self.axes_gyrox.legend()
        self.axes_gyrox.grid()
        self.axes_gyrox.set_ylim([-20, 20])
        self.canvas_gyrox.draw()
        
        # if len(self.gyroy_list) > 50:
        #     self.gyroy_list.pop(0)
        # self.gyroy_list.append(gyro_roll)
        # if len(self.rolloutput_list) > 50:
        #     self.rolloutput_list.pop(0)
        # self.rolloutput_list.append(inner_roll_output)
        # self.x_list = [i for i in range(len(self.gyroy_list))]
        # self.axes_gyroy.clear()
        # self.axes_gyroy.plot(self.x_list, self.gyroy_list, 'b', label='GyroY')
        # # self.axes_gyroy.legend()
        # self.axes_gyroy.plot(self.x_list, self.rolloutput_list, 'r', label='RollOutput')
        # self.axes_gyroy.grid()
        # self.axes_gyroy.set_ylim([-40, 40])
        # self.canvas_gyroy.draw()
       

    def usart_start_pressed(self):
        if self.ui.pushButton_start.text() == "Start":
            self.usart_thread.continue_()
            self.ui.pushButton_start.setText("Stop")
        else:
            self.usart_thread.pause()
            self.ui.pushButton_start.setText("Start")

    def usart_msg(self, msg):
        self.ui.textBrowser_usart.insertPlainText(msg)
        self.ui.textBrowser_usart.moveCursor(QTextCursor.MoveOperation.End)
        self.ui.textBrowser.insertPlainText(msg)
        self.ui.textBrowser.moveCursor(QTextCursor.MoveOperation.End)

    def update1(self):
        if len(channel_list[1]) > 0:
            # if len(channel_list[1]) > 50:
            #     channel_list[1] = channel_list[1][1:]
            #     channel_list[2] = channel_list[2][1:]
            #     channel_list[3] = channel_list[3][1:]
            #     channel_list[4] = channel_list[4][1:]
            #     channel_list[5] = channel_list[5][1:]
            #     channel_list[6] = channel_list[6][1:]
            #     global ptr
            #     ptr -= 1
            global ptr
            ptr += 1
            y = channel_list[1]
            self.curve1.setData(y)
            self.curve1.setPos(ptr, 0)

            y = channel_list[2]
            self.curve2.setData(y)
            self.curve2.setPos(ptr, 0)

            y = channel_list[3]
            self.curve3.setData(y)
            self.curve3.setPos(ptr, 0)

            y = channel_list[4]
            self.curve4.setData(y)
            self.curve4.setPos(ptr, 0)

            self.ui.lineEdit_chan1.setText("chan1: %d (max:%d→, min:%d←) Right ←→" % (
                channel_list[1][-1], max(channel_list[1]), min(channel_list[1])))
            self.ui.lineEdit_chan2.setText("chan2: %d (max:%d↑, min:%d↓) Right ↑↓" % (
                channel_list[2][-1], max(channel_list[2]), min(channel_list[2])))
            self.ui.lineEdit_chan3.setText("chan3: %d (max:%d↑, min:%d↓) Left  ↑↓" % (
                channel_list[3][-1], max(channel_list[3]), min(channel_list[3])))
            self.ui.lineEdit_chan4.setText("chan4: %d (max:%d→, min:%d←) Left  ←→" % (
                channel_list[4][-1], max(channel_list[4]), min(channel_list[4])))

    def update2(self):
        # self.data[:-1] = self.data[1:]
        # self.data[-1] = np.random.normal()
        # self.curve.setData(self.data)
        global t2, y2, ptr2
        if len(t2) == 1:
            ptr2 = self.curve2.pos().x()
            print(ptr2)
        if len(t2) > 0:
            tmp_t = t2.copy()
            tmp_t.append(t2[-1]+1)
            self.curve2.setData(tmp_t, y2)
            ptr2 += 1
            self.curve2.setPos(ptr2, 0)

    def keyPressEvent(self, a0: QKeyEvent):
        if a0.key() == Qt.Key.Key_Q:
            self.close()

    def closeEvent(self, event):
        config_dict['stackedWidget_index'] = self.ui.stackedWidget.currentIndex()
        

        with open(CONFIG_FILE, "w") as f:
            json.dump(config_dict, f, indent=4)

        for thread in self.threads:
            thread.stop()

        global ser
        if ser:
            ser.close()


pattern_chan = r'\[chan:(\d{4});(\d{4});(\d{4});(\d{4});(\d{4});(\d{4})\]'
pattern_in = r'\[in:([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+)\]'
pattern_out = r'\[out:([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+)\]'
pattern_pid = r'\[pid:([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+)\]'
pattern_time = r'\[time:(\d+);(\d+);(\d+);(\d+);(\d+);(\d+)\]'
pattern_p = r'\[f:([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+);([-\d.]+)\]'


def pattern_chan_action(match):
    global channel_list
    channel_list[1].append(int(match.group(1)))
    channel_list[2].append(int(match.group(2)))
    channel_list[3].append(int(match.group(3)))
    channel_list[4].append(int(match.group(4)))
    channel_list[5].append(int(match.group(5)))
    channel_list[6].append(int(match.group(6)))


def pattern_in_action(match):
    global gyro_roll, gyro_pitch, inner_pitch_lasterror, inner_roll_lasterror, inner_pitch_integrator, inner_roll_integrator, inner_pitch_output, inner_roll_output, motor1, motor2, motor3, motor4
    gyro_roll = float(match.group(1))
    gyro_pitch = float(match.group(2))
    inner_pitch_lasterror = float(match.group(3))
    inner_roll_lasterror = float(match.group(4))
    inner_pitch_integrator = float(match.group(5))
    inner_roll_integrator = float(match.group(6))
    inner_pitch_output = float(match.group(7))
    inner_roll_output = float(match.group(8))
    motor1 = float(match.group(9))
    motor2 = float(match.group(10))
    motor3 = float(match.group(11))
    motor4 = float(match.group(12))


def pattern_out_action(match):
    global rx_yaw, rx_pitch, rx_roll, outer_pitch_lasterror, outer_roll_lasterror, outer_pitch_integrator, outer_roll_integrator, outer_pitch_output, outer_roll_output, rx_updown, read_pitch, read_roll, read_yaw
    read_pitch = float(match.group(1))
    read_roll = float(match.group(2))
    read_yaw = float(match.group(3))
    rx_yaw = float(match.group(4))
    rx_pitch = float(match.group(5))
    rx_roll = float(match.group(6))
    rx_updown = float(match.group(7))
    outer_pitch_lasterror = float(match.group(8))
    outer_roll_lasterror = float(match.group(9))
    outer_pitch_integrator = float(match.group(10))
    outer_roll_integrator = float(match.group(11))
    outer_pitch_output = float(match.group(12))
    outer_roll_output = float(match.group(13))


def pattern_pid_action(match):
    global inner_kp, inner_ki, inner_kd, outer_kp, outer_ki, outer_kd
    inner_kp = float(match.group(1))
    inner_ki = float(match.group(2))
    inner_kd = float(match.group(3))
    outer_kp = float(match.group(4))
    outer_ki = float(match.group(5))
    outer_kd = float(match.group(6))
    
def pattern_time_action(match):
    global kalman_time, send_time, receive_time, inner_loop_time, motor_time, outer_loop_time
    kalman_time = int(match.group(1))
    send_time = int(match.group(2))
    receive_time = int(match.group(3))
    inner_loop_time = int(match.group(4))
    motor_time = int(match.group(5))
    outer_loop_time = int(match.group(6))
    
def pattern_p_action(match):
    global read_pitch, gyro_pitch, outer_pitch_output, motor1, motor2, motor3, motor4
    read_pitch = float(match.group(1))
    gyro_pitch = float(match.group(2))
    outer_pitch_output = float(match.group(3))
    motor1 = float(match.group(4))
    motor2 = float(match.group(5))
    motor3 = float(match.group(6))
    motor4 = float(match.group(7))


class UsartThread(QThread):
    msg = pyqtSignal(str)

    def __init__(self, parent=None):
        super().__init__(parent=parent)
        self.flag = False
        self.mode = "line"

    def run(self):
        global rx_cnt, tx_cnt
        while True:
            if self.flag and ser.isOpen():
                # try:
                if self.mode == "byte":
                    data = ser.read(1)
                elif self.mode == "line":
                    data = ser.readline()
                try:
                    data = data.decode("utf-8")
                    # try:
                    match_dict = {
                        pattern_chan: pattern_chan_action,
                        pattern_in: pattern_in_action,
                        pattern_out: pattern_out_action,
                        pattern_pid: pattern_pid_action,
                        pattern_time: pattern_time_action,
                        pattern_p: pattern_p_action,
                    }
                    for pattern, action in match_dict.items():
                        match = re.match(pattern, data)
                        if match:
                            action(match)
                            break
                    # except:
                    #     pass
                except:
                    data = ""#"?" * len(data)
                rx_cnt += len(data)
                self.msg.emit(data)
                # except KeyboardInterrupt:
                #     break

    def pause(self):
        self.flag = False

    def continue_(self):
        self.flag = True

    def stop(self):
        self.terminate()


class SendThread(QThread):
    def __init__(self, str, parent=None):
        super().__init__(parent=parent)
        self.str = str

    def run(self):
        ser.write(self.str.encode("utf-8") + b"\n")
        
    def stop(self):
        self.terminate()
