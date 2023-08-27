from matrix_utils import hat, vee, q_to_R
from control import Control
from estimator import Estimator
from trajectory import Trajectory

import datetime
import numpy as np
import pdb
import rospy
import threading

from geometry_msgs.msg import Pose, Twist, Wrench
from geometry_msgs.msg import Vector3, Point, Quaternion
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Imu
from std_msgs.msg import String


class Rover:
    def __init__(self):

        self.on = True
        self.motor_on = False
        self.save_on = False
        self.mode = 0

        self.t0 = datetime.datetime.now()
        self.t = 0.0
        self.t_pre = 0.0
        self.freq_imu = 0.0
        self.freq_gps = 0.0
        self.freq_control = 0.0
        self.freq_log = 0.0

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        self.g = 9.81
        self.ge3 = np.array([0.0, 0.0, self.g])

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        # Note that ENU and the NED here refer to their direction order.
        # ENU: E - axis 1, N - axis 2, U - axis 3
        # NED: N - axis 1, E - axis 2, D - axis 3
        self.R_fg = np.array([
            [1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0],
            [0.0, 0.0, -1.0]
        ])

        self.V_R_imu = np.diag([0.01, 0.01, 0.01])
        self.V_x_gps = np.diag([0.01, 0.01, 0.01])
        self.V_v_gps = np.diag([0.01, 0.01, 0.01])

        self.control = Control()
        self.control.use_integral = True  # Enable integral control

        self.estimator = Estimator()
        self.trajectory = Trajectory()

        self.lock = threading.Lock()

    
    def update_current_time(self):
        t_now = datetime.datetime.now()
        self.t = (t_now - self.t0).total_seconds()


    def get_current_time(self):
        t_now = datetime.datetime.now()
        return (t_now - self.t0).total_seconds()
   
    
    def run_controller(self):
        self.update_current_time()

        with self.lock:
            states = self.estimator.get_states()
            desired = self.trajectory.get_desired(rover.mode, states, \
                self.x_offset, self.yaw_offset)
            fM = self.control.run(states, desired)

            self.x, self.v, self.a, self.R, self.W = states

        return fM


    def ros_imu_callback(self, message):
        q_gazebo = message.orientation
        a_gazebo = message.linear_acceleration
        W_gazebo = message.angular_velocity

        q = np.array([q_gazebo.x, q_gazebo.y, q_gazebo.z, q_gazebo.w])

        R_gi = q_to_R(q) # IMU to Gazebo frame
        R_fi = self.R_fg.dot(R_gi)  # IMU to FDCL frame (NED freme)

        # FDCL-UAV expects IMU accelerations without gravity.
        a_i = np.array([a_gazebo.x, a_gazebo.y, a_gazebo.z])
        a_i = R_gi.T.dot(R_gi.dot(a_i) - self.ge3)

        W_i = np.array([W_gazebo.x, W_gazebo.y, W_gazebo.z])

        with self.lock:
            self.estimator.prediction(a_i, W_i)
            self.estimator.imu_correction(R_fi, self.V_R_imu)


    def ros_gps_callback(self, message):
        x_gazebo = message.pose.pose.position
        v_gazebo = message.twist.twist.linear

        # Gazebo uses ENU frame, but NED frame is used in FDCL.
        x_g = np.array([x_gazebo.x, -x_gazebo.y, -x_gazebo.z])
        v_g = np.array([v_gazebo.x, -v_gazebo.y, -v_gazebo.z])

        with self.lock:
            self.estimator.gps_correction(x_g, v_g, self.V_x_gps, self.V_v_gps)



def reset_uav():
    rospy.wait_for_service('/gazebo/set_model_state')
    
    init_position = Point(x=0.0, y=0.0, z=0.2)
    init_attitude = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
    init_pose = Pose(position=init_position, orientation=init_attitude)

    zero_motion = Vector3(x=0.0, y=0.0, z=0.0)
    init_velocity = Twist(linear=zero_motion, angular=zero_motion)

    model_state = ModelState(model_name='uav', reference_frame='world', \
        pose=init_pose, twist=init_velocity)
    
    reset_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
    reset_state(model_state)

    print('Resetting UAV successful ..')


rover = Rover()
