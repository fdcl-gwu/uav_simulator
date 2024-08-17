from rover import rover
from plot_utils import plot_data

import gi
import numpy as np
import os
import sys

import rclpy

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, \
    QHBoxLayout, QPushButton, QRadioButton, QMainWindow

from rclpy.node import Node
from rclpy.node import Node
from sensor_msgs.msg import Imu

class GuiNode(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'gui')

        QMainWindow.__init__(self)

        self.t0 = self.get_clock().now()

        self.R_imu = np.zeros([3, 3])
        self.a_imu = np.zeros([3, 1])
        self.w_imu = np.zeros([3, 1])

        self.init_gui()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)

        # self.window.show()


    def init_gui(self):
        # app = QApplication([])
        window = QWidget()
        window.setWindowTitle("FDCL UAV Simulator")
        # window.setGeometry(100, 100, 280, 80)

        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)
        # main_layout.setMargin(10)


        # Status box
        self.label_time = QLabel("0.0")
        self.label_time.setMinimumWidth(80)
        self.label_freq_imu = QLabel("0.0")
        self.label_freq_gps = QLabel("0.0")
        self.label_freq_control = QLabel("0.0")
        self.label_freq_estimator = QLabel("0.0")

        thread_status_box = QHBoxLayout()
        thread_status_box.setAlignment(Qt.AlignLeft)
        thread_status_box.addWidget(self.label_time)
        thread_status_box.addWidget(self.label_freq_imu)
        thread_status_box.addWidget(self.label_freq_gps)
        thread_status_box.addWidget(self.label_freq_control)
        thread_status_box.addWidget(self.label_freq_estimator)


        # Control box
        self.button_on = QPushButton("Shutdown")
        self.button_on.clicked.connect(self.on_btn_close_clicked)

        self.button_motor_on = QPushButton("Turn on motors")
        self.button_motor_on.clicked.connect(self.on_btn_motor_on_clicked)

        controls_box = QHBoxLayout()
        controls_box.setAlignment(Qt.AlignLeft)
        controls_box.addWidget(self.button_on)
        controls_box.addWidget(self.button_motor_on)


        # Flight mode box
        label_flight_mode = QLabel("Mode: ")
        self.radio_button_modes = \
            [ \
                QRadioButton("Idle"), \
                QRadioButton("Warmup"), \
                QRadioButton("Take-off"), \
                QRadioButton("Stay"), \
                QRadioButton("Manual"), \
                QRadioButton("Circle") \
            ]
        num_flight_modes = len(self.radio_button_modes)
        
        flight_mode_box = QHBoxLayout()
        flight_mode_box.setAlignment(Qt.AlignLeft)
        flight_mode_box.addWidget(label_flight_mode)
        for i in range(num_flight_modes):
            flight_mode_box.addWidget(self.radio_button_modes[i])
            self.radio_button_modes[i].toggled.connect(self.on_flight_mode_changed)


        # State box
        state_box = QHBoxLayout()
        state_box.setAlignment(Qt.AlignLeft)
        state_box.addSpacing(5)
        [state_box, self.label_x] = self.init_vbox(state_box, "x", 4)
        [state_box, self.label_v] = self.init_vbox(state_box, "v", 4)
        [state_box, self.label_a] = self.init_vbox(state_box, "a", 4)
        [state_box, self.label_R] = self.init_attitude_vbox(state_box, "R")
        [state_box, self.label_W] = self.init_vbox(state_box, "W", 4)


        # Desired box
        desired_box = QHBoxLayout()
        desired_box.setAlignment(Qt.AlignLeft)
        desired_box.addSpacing(5)
        [desired_box, self.label_xd] = self.init_vbox(desired_box, "xd", 4)
        [desired_box, self.label_vd] = self.init_vbox(desired_box, "vd", 4)
        [desired_box, self.label_b1d] = self.init_vbox(desired_box, "b1d", 4)
        [desired_box, self.label_Rd] = self.init_attitude_vbox(desired_box, "Rd")
        [desired_box, self.label_Wd] = self.init_vbox(desired_box, "Wd", 4)


        # Sensor box
        sensor_box = QHBoxLayout()
        sensor_box.setAlignment(Qt.AlignLeft)
        sensor_box.addSpacing(5)
        [sensor_box, self.label_imu_ypr] = self.init_vbox(sensor_box, "IMU YPR", 4)
        [sensor_box, self.label_imu_a] = self.init_vbox(sensor_box, "IMU a", 4)
        [sensor_box, self.label_imu_W] = self.init_vbox(sensor_box, "IMU W", 4)
        [sensor_box, self.label_gps_x] = self.init_vbox(sensor_box, "GPS x", 4)
        [sensor_box, self.label_gps_v] = self.init_vbox(sensor_box, "GPS v", 4)


        # Error box
        error_box = QHBoxLayout()
        error_box.setAlignment(Qt.AlignLeft)
        error_box.addSpacing(5)
        [error_box, self.label_ex] = self.init_vbox(error_box, "ex", 4)
        [error_box, self.label_ev] = self.init_vbox(error_box, "ev", 4)
        [error_box, self.label_fM] = self.init_vbox(error_box, "f-M", 4)
        [error_box, self.label_f] = self.init_vbox(error_box, "f", 4)
        [error_box, self.label_thr] = self.init_vbox(error_box, "Throttle", 4)


        # Left box
        left_box = QVBoxLayout()
        left_box.setAlignment(Qt.AlignTop)
        left_box.addSpacing(5)
        left_box.addLayout(state_box)
        main_layout.addSpacing(10)
        left_box.addLayout(desired_box)


        # Right box
        right_box = QVBoxLayout()
        right_box.setAlignment(Qt.AlignTop)
        right_box.addSpacing(5)
        right_box.addLayout(sensor_box)
        main_layout.addSpacing(10)
        right_box.addLayout(error_box)

        # Data box
        data_box = QHBoxLayout()
        data_box.setAlignment(Qt.AlignLeft)
        data_box.addLayout(left_box)
        data_box.addSpacing(5)
        data_box.addLayout(right_box)

        main_layout.setAlignment(Qt.AlignTop)
        main_layout.addLayout(thread_status_box)
        main_layout.addSpacing(5)
        main_layout.addLayout(controls_box)
        main_layout.addSpacing(5)
        main_layout.addLayout(flight_mode_box)
        main_layout.addSpacing(5)
        main_layout.addLayout(data_box)

        window.setLayout(main_layout)
        self.setCentralWidget(window)

        # return window


    
    
    def update(self):
        t_now = self.get_clock().now()
        t_sec = float((t_now - self.t0).to_msg().sec)

        self.label_time.setText(f't: {t_sec:5.1f} s')
        self.label_freq_imu.setText(f'IMU: {t_sec:5.1f} Hz')
        self.label_freq_gps.setText(f'GPS: {t_sec:5.1f} Hz')
        self.label_freq_control.setText(f'CTR: {t_sec:5.1f} Hz')
        self.label_freq_estimator.setText(f'EST: {t_sec:5.1f} Hz')

        # self.update_vbox(self.label_x, rover.x)
        # self.update_vbox(self.label_v, rover.v)
        # self.update_vbox(self.label_a, self.a_imu)
        # self.update_vbox(self.label_R, rover.R)
        # self.update_vbox(self.label_W, rover.W)

        # self.update_vbox(self.label_xd, rover.xd)
        # self.update_vbox(self.label_vd, rover.vd)
        # self.update_vbox(self.label_b1d, rover.b1d)
        # self.update_vbox(self.label_Rd, rover.Rd)
        # self.update_vbox(self.label_Wd, rover.Wd)

        # self.update_vbox(self.label_imu_ypr, rover.imu_ypr)

        self.update_vbox(self.label_imu_a, rover.a_imu)
        self.update_vbox(self.label_imu_W, rover.W_imu)
        self.update_vbox(self.label_gps_x, rover.x_gps)
        self.update_vbox(self.label_gps_v, rover.v_gps)

        # self.update_vbox(self.label_ex, rover.ex)
        # self.update_vbox(self.label_ev, rover.ev)
        # self.update_vbox(self.label_fM, rover.fM)
        # self.update_vbox(self.label_f, rover.f)
        # # self.update_vbox(self.label_thr, rover.thr)


    def init_vbox(self, box, name, num):
        vbox = QVBoxLayout()
        vbox.setAlignment(Qt.AlignTop)

        labels = []
        for _ in range(num):
            label_i = QLabel("{:5.2}".format(0.0))

            labels.append(label_i)
            vbox.addWidget(label_i)

        labels[0].setText(name)
        box.addLayout(vbox)

        return [box, labels]

    
    def init_attitude_vbox(self, box, name):
        vbox_array = [QVBoxLayout(), QVBoxLayout(), QVBoxLayout()]
        labels = []

        for vbox_index in range(len(vbox_array)):
            vbox = vbox_array[vbox_index]
            vbox.setAlignment(Qt.AlignTop)

            for _ in range(4):
                label_i = QLabel("{:6.4}".format(0.0))
            
                labels.append(label_i)
                vbox.addWidget(label_i)

            box.addLayout(vbox)

        labels[0].setText(name)
        labels[4].setText("")
        labels[8].setText("")

        return [box, labels]
    

    def update_vbox(self, labels, data):
        breakpoint
        for i in range(3):
            labels[i + 1].setText("{:>5.2f}".format(data[i]))


    def on_btn_close_clicked(self):
        self.get_logger().info("Shutdown button clicked")
        # self.destroy_node()
        self.destroy_node()
        self.close()
        rclpy.shutdown()
        # rclpy.shutdown()
        # self.get_logger().info("Program closed")
        # raise Exception("Program closed")

    
    def on_btn_motor_on_clicked(self):
        if rover.motor_on:
            rover.motor_on = False
            self.button_motor_on.setText("Turn on motors")
        else:
            rover.motor_on = True
            self.button_motor_on.setText("Turn off motors")


    def on_flight_mode_changed(self):
        for i in range(len(self.radio_button_modes)):
            if self.radio_button_modes[i].isChecked():
                self.get_logger().info('Mode switched to {}'.format(i))
                rover.mode = i
                rover.x_offset = np.zeros(3)
                rover.yaw_offset = 0.0
                break


    def on_key_press(self, event):
        if event.key() == Qt.Key_Q:
            self.on_btn_close_clicked()
        elif event.key() == Qt.Key_M:
            self.on_btn_motor_on_clicked()


        


class Gui():
    def __init__(self):

        self.builder = Gtk.Builder()
        self.builder.add_from_file("gui.glade")

        self.window = self.builder.get_object('window_main')
        self.window.set_title('UAV Simulator')
        self.window.connect('destroy', Gtk.main_quit)
        self.window.connect('key-press-event', self.on_key_press)

        self.btn_close = self.builder.get_object('btn_close')
        self.btn_close.connect('clicked', self.on_btn_close_clicked)

        self.tgl_motor_on = self.builder.get_object('tgl_motor_on')
        self.tgl_motor_on.connect('toggled', self.on_tgl_motor_on_toggled)

        self.tgl_save = self.builder.get_object('tgl_save')
        self.tgl_save.connect('toggled', self.on_tgl_save_toggled)

        self.lbl_save = self.builder.get_object('lbl_save')
        self.lbl_save.set_text(rover.t0.strftime('log_%Y%m%d_%H%M%S.txt'))

        self.btn_plot = self.builder.get_object('btn_plot')
        self.btn_plot.connect('clicked', self.on_btn_plot_clicked)

        self.lbl_t = self.builder.get_object('lbl_t')
        self.lbl_freq_imu = self.builder.get_object('lbl_freq_imu')
        self.lbl_freq_gps = self.builder.get_object('lbl_freq_gps')
        self.lbl_freq_control = self.builder.get_object('lbl_freq_control')
        self.lbl_freq_log = self.builder.get_object('lbl_freq_log')

        self.modes = ['Idle', 'Warm-up', 'Take-off', 'Land', 'Stay', 'Circle']
        self.rdo_mode = []
        for i in range(len(self.modes)):
            self.rdo_mode.append(
                self.builder.get_object('rdo_mode_{}'.format(i)))
            self.rdo_mode[i].set_label(self.modes[i])
            self.rdo_mode[i].connect('toggled', self.on_rdo_buton_clicked, i)

        self.x = self.get_vbox('lbl_x')
        self.v = self.get_vbox('lbl_v')
        self.a = self.get_vbox('lbl_a')
        self.W = self.get_vbox('lbl_W')
        self.R = self.get_grid('lbl_R')

        self.xd = self.get_vbox('lbl_xd')
        self.vd = self.get_vbox('lbl_vd')
        self.Wd = self.get_vbox('lbl_Wd')
        self.b1d = self.get_vbox('lbl_b1d')
        self.Rd = self.get_grid('lbl_Rd')

        self.window.connect('destroy', Gtk.main_quit)
        self.window.show_all()


    def on_btn_close_clicked(self, *args):
        print('GUI: close button clicked')
        rover.on = False
        Gtk.main_quit()


    def on_btn_plot_clicked(self, widget):
        print('GUI: generating plots ..')
        # plot_data()
        os.system('python3 plot_utils.py')


    def on_tgl_save_toggled(self, widget):
        if self.tgl_save.get_active():
            print('GUI: saving data started ..')
            rover.save_on = True
        else:
            print('GUI: saving data stopped')
            rover.save_on = False


    def on_tgl_motor_on_toggled(self, widget):
        if self.tgl_motor_on.get_active():
            print('GUI: motor on ..')
            rover.motor_on = True
        else:
            print('GUI: motor off')
            rover.motor_on = False


    def on_rdo_buton_clicked(self, widget, num):
        if widget.get_active():
            print('GUI: mode switched to {}'.format(self.modes[num]))
            rover.mode = num
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0


    def on_key_press(self, widget, event):
        key = event.keyval
        
        x_step = 0.1
        yaw_step = 0.02
        
        if key == Gdk.KEY_M or  key == Gdk.KEY_m:
            print('GUI: turning motors off')
            rover.motor_on = False
            self.tgl_motor_on.set_active(False)
        elif key == Gdk.KEY_0:
            self.rdo_mode[0].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_1:
            self.rdo_mode[1].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_2:
            self.rdo_mode[2].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_3:
            self.rdo_mode[3].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_4:
            self.rdo_mode[4].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_5:
            self.rdo_mode[5].set_active(True)
            rover.x_offset = np.zeros(3)
            rover.yaw_offset = 0.0
        elif key == Gdk.KEY_W or key == Gdk.KEY_w:
            rover.x_offset[0] += x_step
        elif key == Gdk.KEY_S or key == Gdk.KEY_s:
            rover.x_offset[0] -= x_step
        elif key == Gdk.KEY_D or key == Gdk.KEY_d:
            rover.x_offset[1] += x_step
        elif key == Gdk.KEY_A or key == Gdk.KEY_a:
            rover.x_offset[1] -= x_step
        elif key == Gdk.KEY_L or key == Gdk.KEY_l:
            rover.x_offset[2] += x_step
        elif key == Gdk.KEY_P or key == Gdk.KEY_p:
            rover.x_offset[2] -= x_step
        elif key == Gdk.KEY_E or key == Gdk.KEY_e:
            rover.yaw_offset += yaw_step
        elif key == Gdk.KEY_Q or key == Gdk.KEY_q:
            rover.yaw_offset -= yaw_step


    def update_gui(self):
        self.lbl_t.set_text('t = {:0.1f} s'.format(rover.t))
        self.lbl_freq_imu.set_text( \
            'IMU = {:3.0f} Hz'.format(rover.freq_imu))
        self.lbl_freq_gps.set_text(
            'GPS = {:3.0f} Hz'.format(rover.freq_gps))
        self.lbl_freq_control.set_text(
            'CTRL = {:3.0f} Hz'.format(rover.freq_control))
        self.lbl_freq_log.set_text(
            'LOG = {:3.0f} Hz'.format(rover.freq_log))

        self.update_vbox(self.x, rover.x)
        self.update_vbox(self.v, rover.v)
        self.update_vbox(self.a, rover.a)
        self.update_vbox(self.W, rover.W)
        self.update_grid(self.R, rover.R)

        self.update_vbox(self.xd, rover.control.xd)
        self.update_vbox(self.vd, rover.control.xd_dot)
        self.update_vbox(self.b1d, rover.control.b1d)
        self.update_vbox(self.Wd, rover.control.Wd)
        self.update_grid(self.Rd, rover.control.Rd)

        return True


    def get_vbox(self, name):
        vbox = []
        for i in range(3):
            vbox.append(self.builder.get_object('{}_{}'.format(name, i + 1)))
        return vbox


    def update_vbox(self, vbox, data):
        for i in range(3):
            vbox[i].set_text('{:8.2f}'.format(data[i]))


    def get_grid(self, name):
        vbox = []
        for i in range(1, 4):
            for j in range(1, 4):
                vbox.append(self.builder.get_object('{}_{}{}'.format(name, \
                    i, j)))
        return vbox


    def update_grid(self, grid, data):
        for i in range(3):
            for j in range(3):
                k = 3 * i + j
                grid[k].set_text('{:8.2f}'.format(data[i, j]))


def thread_gui():
    print('GUI: starting thread ..')

    gui = Gui()
    GLib.idle_add(gui.update_gui)
    Gtk.main()

    rover.on = False
    print('GUI: thread closed!')


if __name__=='__main__':
    thread_gui()