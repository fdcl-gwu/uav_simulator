from .matrix_utils import q_to_R

import numpy as np
import os

from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import QApplication, QLabel, QWidget, QVBoxLayout, \
    QHBoxLayout, QPushButton, QRadioButton, QMainWindow

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import WrenchStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Int32

from uav_gazebo.msg import StateData, DesiredData, ErrorData


class GuiNode(Node, QMainWindow):
    def __init__(self):
        Node.__init__(self, 'gui')
        QMainWindow.__init__(self)

        self.t0 = self.get_clock().now()

        self.motor_on = False

        # States
        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

         # Desired states
        self.xd = np.zeros(3)
        self.vd = np.zeros(3)
        self.b1d = np.zeros(3)
        self.b1d[0] = 1.0
        self.Rd = np.identity(3)
        self.Wd = np.zeros(3)

        # Errors
        self.ex = np.zeros(3)
        self.ev = np.zeros(3)
        self.eR = np.identity(3)
        self.eW = np.zeros(3)
        self.eIX = np.zeros(3)
        self.eIR = np.zeros(3)

        # Sensor measurements
        self.R_imu = np.zeros([3, 3])
        self.a_imu = np.zeros([3, 1])
        self.W_imu = np.zeros([3, 1])

        self.x_gps = np.zeros([3, 1])
        self.v_gps = np.zeros([3, 1])

        # Motor forces
        self.fM = np.zeros(4)

        # Manual mode
        self.x_offset = np.zeros(3)
        self.x_offset_delta = 0.1
        self.yaw_offset = 0.0
        self.yaw_offset_delta = 0.02

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        self.init_gui()
        self.init_subscribers()
        self.init_publishers()

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)
        self.timer.start(1000)


    def init_gui(self):
        window = QWidget()
        window.setWindowTitle("FDCL UAV Simulator")

        main_layout = QVBoxLayout()
        main_layout.setSpacing(10)

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
                QRadioButton("Circle"), \
                QRadioButton("Land") \
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
        [error_box, self.label_fM] = self.init_vbox(error_box, "f-M", 5)
        # [error_box, self.label_f] = self.init_vbox(error_box, "f", 5)
        # [error_box, self.label_thr] = self.init_vbox(error_box, "Throttle", 5)

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
        window.keyPressEvent = self.on_key_press

    def init_subscribers(self):

        self.sub_states = self.create_subscription( \
            StateData,
            '/uav/states',
            self.states_callback,
            1)
        
        self.sub_desired = self.create_subscription( \
            DesiredData,
            '/uav/desired',
            self.desired_callback,
            1)
        
        self.sub_errors = self.create_subscription( \
            ErrorData,
            '/uav/errors',
            self.errors_callback,
            1)

        self.sub_fM = self.create_subscription( \
            WrenchStamped,
            '/uav/fm',
            self.fM_callback,
            1)
        
        self.sub_imu = self.create_subscription( \
            Imu,
            '/uav/imu',
            self.imu_callback,
            1)
        
        self.sub_gps = self.create_subscription( \
            Odometry,
            '/uav/gps',
            self.gps_callback,
            1)


    def init_publishers(self):
        self.pub_mode = self.create_publisher(Int32, '/uav/mode', 1) 
    
    
    def update(self):
        t_now = self.get_clock().now()
        t_sec = float((t_now - self.t0).to_msg().sec)

        self.label_time.setText(f't: {t_sec:5.1f} s')
        self.label_freq_imu.setText(f'IMU: {t_sec:5.1f} Hz')
        self.label_freq_gps.setText(f'GPS: {t_sec:5.1f} Hz')
        self.label_freq_control.setText(f'CTR: {t_sec:5.1f} Hz')
        self.label_freq_estimator.setText(f'EST: {t_sec:5.1f} Hz')

        self.update_vbox(self.label_x, self.x)
        self.update_vbox(self.label_v, self.v)
        self.update_vbox(self.label_a, self.a)
        self.update_attitude_vbox(self.label_R, self.R)
        self.update_vbox(self.label_W, self.W)

        self.update_vbox(self.label_xd, self.xd)
        self.update_vbox(self.label_vd, self.vd)
        self.update_vbox(self.label_b1d, self.b1d)
        self.update_attitude_vbox(self.label_Rd, self.Rd)
        self.update_vbox(self.label_Wd, self.Wd)

        # TODO
        # self.update_vbox(self.label_imu_ypr, self.imu_ypr)

        self.update_vbox(self.label_imu_a, self.a_imu)
        self.update_vbox(self.label_imu_W, self.W_imu)
        self.update_vbox(self.label_gps_x, self.x_gps)
        self.update_vbox(self.label_gps_v, self.v_gps)

        self.update_vbox(self.label_ex, self.ex)
        self.update_vbox(self.label_ev, self.ev)

        self.update_vbox(self.label_fM, self.fM, data_size=4)
        # self.update_vbox(self.label_f, self.f)
        # self.update_vbox(self.label_thr, self.thr)


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
    

    def update_vbox(self, labels, data, data_size=3):
        for i in range(data_size):
            labels[i + 1].setText("{:>5.2f}".format(data[i]))

    
    def update_attitude_vbox(self, labels, data):
        for i in range(3):
            for j in range(1, 4):
                labels[i * 4 + j].setText("{:>5.2f}".format(data[i, j - 1]))


    def on_btn_close_clicked(self):
        self.get_logger().info("Shutdown button clicked")
        self.destroy_node()
        self.close()
        rclpy.shutdown()

    
    def on_btn_motor_on_clicked(self):
        if self.motor_on:
            self.get_logger().info('Turning motors on')
            self.motor_on = False
            self.button_motor_on.setText("Turn on motors")
        else:
            self.get_logger().info('Turning motors off')
            self.motor_on = True
            self.button_motor_on.setText("Turn off motors")


    def on_flight_mode_changed(self):
        pass
        for i in range(len(self.radio_button_modes)):
            if self.radio_button_modes[i].isChecked():
                self.get_logger().info('Mode switched to {}'.format(i))
                
                msg = Int32()
                msg.data = i
                self.pub_mode.publish(msg)

                self.x_offset = np.zeros(3)
                self.yaw_offset = 0.0
                break


    def on_key_press(self, event):
        self.get_logger().info('Key presed {}'.format(event.key()))
        if event.key() == Qt.Key_M:
            self.on_btn_motor_on_clicked()
        elif event.key() == Qt.Key_QuoteLeft:
            self.radio_button_modes[0].setChecked(True)
        elif event.key() == Qt.Key_1:
            self.radio_button_modes[1].setChecked(True)
        elif event.key() == Qt.Key_2:
            self.radio_button_modes[2].setChecked(True)
        elif event.key() == Qt.Key_3:
            self.radio_button_modes[3].setChecked(True)
        elif event.key() == Qt.Key_4:
            self.radio_button_modes[4].setChecked(True)
        elif event.key() == Qt.Key_5:
            self.radio_button_modes[5].setChecked(True)
        elif event.key() == Qt.Key_6:
            self.radio_button_modes[6].setChecked(True)
        elif event.key() == Qt.Key_W:
            self.x_offset[0] += self.x_offset_delta
        elif event.key() == Qt.Key_S:
            self.x_offset[0] -= self.x_offset_delta
        elif event.key() == Qt.Key_D:
            self.x_offset[1] += self.x_offset_delta
        elif event.key() == Qt.Key_A:
            self.x_offset[1] -= self.x_offset_delta
        elif event.key() == Qt.Key_L:
            self.x_offset[2] += self.x_offset_delta
        elif event.key() == Qt.Key_P:
            self.x_offset[2] -= self.x_offset_delta
        elif event.key() == Qt.Key_E:
            self.yaw_offset += self.yaw_offset_delta
        elif event.key() == Qt.Key_Q:
            self.yaw_offset -= self.yaw_offset_delta


    def states_callback(self, msg):
        """Callback function for the states subscriber.

        Args:
            msg: (StateData) State data message
        """
       
        for i in range(3):
            self.x[i] = msg.position[i]
            self.v[i] = msg.velocity[i]
            self.a[i] = msg.acceleration[i]
            self.W[i] = msg.angular_velocity[i]

            for j in range(3):
                self.R[i, j] = msg.attitude[3*i + j]


    def desired_callback(self, msg):
        """Callback function for the desired subscriber.

        Args:
            msg: (DesiredData) Desired data message
        """

        for i in range(3):
            self.xd[i] = msg.position[i]
            self.vd[i] = msg.velocity[i]
            self.b1d[i] = msg.b1[i]
            self.Wd[i] = msg.angular_velocity[i]

            for j in range(3):
                self.Rd[i, j] = msg.attitude[3*i + j]


    def errors_callback(self, msg):
        """Callback function for the error subscriber.

        Args:
            msg: (ErrorData) Error data message
        """

        for i in range(3):
            self.ex[i] = msg.position[i]
            self.ev[i] = msg.velocity[i]
            self.eR[i] = msg.attitude[i]
            self.eW[i] = msg.angular_velocity[i]
            self.eIX[i] = msg.position_integral[i]
            self.eIR[i] = msg.attitude_integral[i]

    
    def imu_callback(self, msg):
        """Callback function for the IMU subscriber.

        Args:
            msg: (Imu) IMU message
        """

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        # Note that ENU and the NED here refer to their direction order.
        # ENU: E - axis 1, N - axis 2, U - axis 3
        # NED: N - axis 1, E - axis 2, D - axis 3
        self.R_fg = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        q_gazebo = msg.orientation
        a_gazebo = msg.linear_acceleration
        W_gazebo = msg.angular_velocity

        q = np.array([q_gazebo.x, q_gazebo.y, q_gazebo.z, q_gazebo.w])

        R_gi = q_to_R(q) # IMU to Gazebo frame
        R_fi = self.R_fg.dot(R_gi)  # IMU to FDCL frame (NED freme)

        # FDCL-UAV expects IMU accelerations without gravity.
        a_i = np.array([a_gazebo.x, a_gazebo.y, a_gazebo.z])
        self.a_imu = R_gi.T.dot(R_gi.dot(a_i) - self.ge3)

        self.W_imu = np.array([W_gazebo.x, W_gazebo.y, W_gazebo.z])


    def gps_callback(self, msg):
        """Callback function for the GPS subscriber.

        Args:
            msg: (GPS) GPS message
        """

        x_gazebo = msg.pose.pose.position
        v_gazebo = msg.twist.twist.linear

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        self.x_gps = np.array([x_gazebo.x, -x_gazebo.y, -x_gazebo.z])
        self.v_gps = np.array([v_gazebo.x, -v_gazebo.y, -v_gazebo.z])


    def fM_callback(self, msg):
        self.fM[0] = msg.wrench.force.z
        self.fM[1] = msg.wrench.torque.x
        self.fM[2] = msg.wrench.torque.y
        self.fM[3] = msg.wrench.torque.z


def main(args=None):
    print("Starting gui node")

    rclpy.init(args=args)

    app = QApplication([])
    app.processEvents()

    gui = GuiNode()
    gui.show()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(gui)

    while rclpy.ok():
        executor.spin_once()
        app.processEvents()
    
    app.quit()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gui.destroy_node()
    
    # rclpy.shutdown()
    
    print("Terminating gui node")