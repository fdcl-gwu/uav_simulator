from matrix_utils import hat, vee, q_to_R
from control import Control

import datetime
import numpy as np
import pdb

from sensor_msgs.msg import Imu
from std_msgs.msg import String


class Rover:
    def __init__(self):

        self.t0 = datetime.datetime.now()
        self.t = 0.0

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        self.R_fg = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        # Transformation from IMU frame to the body frame.
        self.R_bi = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])
        self.R_bi_T = self.R_bi.T

        self.control = Control()
        self.control.use_integral = True  # Enable integral control

    
    def update_current_time(self):
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()


    def get_current_time(self):
        t_now = datetime.datetime.now()
        return (t_now - self.t0).total_seconds()
   
    
    def run_controller(self):

        fM = self.control.run(self.x, self.v, self.a, self.R, self.W)
        return fM


    def imu_correction(self, a_i, W_i, R_fi):
        self.a = self.R_bi.dot(a_i)
        self.W = self.R_bi.dot(W_i)
        self.R = R_fi.dot(self.R_bi_T)

    
    def gps_correction(self, x_g, v_g):
        self.x = x_g
        self.v = v_g


    def ros_imu_callback(self, message):
        q_gazebo = message.orientation
        a_gazebo = message.linear_acceleration
        W_gazebo = message.angular_velocity

        q = np.array([q_gazebo.x, q_gazebo.y, q_gazebo.z, q_gazebo.w])

        R_gi = q_to_R(q) # IMU to Gazebo frame
        R_fi = self.R_fg.dot(R_gi)

        # FDCL-UAV expects IMU accelerations without gravity.
        a_i = np.array([a_gazebo.x, a_gazebo.y, a_gazebo.z])
        a_f = R_fi.T.dot(R_gi.dot(a_i) - self.ge3)

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        a_i = np.array([a_f[0], -a_f[1], -a_f[2]])

        W_i = np.array([W_gazebo.x, W_gazebo.y, W_gazebo.z])

        self.imu_correction(a_i, W_i, R_fi)


    def ros_gps_callback(self, message):
        x_gazebo = message.pose.pose.position
        v_gazebo = message.twist.twist.linear

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        x_g = np.array([x_gazebo.x, -x_gazebo.y, -x_gazebo.z])
        v_g = np.array([v_gazebo.x, -v_gazebo.y, -v_gazebo.z])

        self.gps_correction(x_g, v_g)

    