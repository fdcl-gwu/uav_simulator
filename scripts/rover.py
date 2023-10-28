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

        self.on = True                      # Turn on/off the rover
        self.motor_on = False               # Turn on/off the motor
        self.save_on = False                # Turn on/off the log
        self.mode = 0                       # Mode of the rover

        self.t0 = datetime.datetime.now()   # Start time
        self.t = 0.0                        # Current time
        self.t_pre = 0.0                    # Previous time
        self.freq_imu = 0.0                 # IMU frequency
        self.freq_gps = 0.0                 # GPS frequency
        self.freq_control = 0.0             # Control frequency
        self.freq_log = 0.0                 # Log frequency

        self.x = np.zeros(3)                # (3x3 numpy array) Position
        self.v = np.zeros(3)                # (3x3 numpy array) Velocity
        self.a = np.zeros(3)                # (3x3 numpy array) Acceleration
        self.R = np.identity(3)             # (3x3 numpy array) current attitude of the UAV in SO(3)
        self.W = np.zeros(3)                # (3x1 numpy array) current angular velocity of the UAV [rad/s]

        self.x_offset = np.zeros(3)         # (3x1 numpy array) Position offset
        self.yaw_offset = 0.0               # (float) Yaw offset

        self.g = 9.81                           # (float) Gravitational acceleration 
        self.ge3 = np.array([0.0, 0.0, self.g]) # (3x1 numpy array) Gravitational acceleration in the ENU frame

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
        """
        The function is used as a callback function for the IMU sensor data.
        
        The function first extracts the orientation quaternion, linear acceleration, 
            and angular velocity from the message. It converts the orientation quaternion to a 
            rotation matrix and transforms it to the FDCL frame. It also removes gravity from 
            the linear acceleration and transforms it to the FDCL frame. Finally, it transforms 
            the angular velocity to the FDCL frame.

        The function then acquires a lock to ensure that the estimator is not modified by another thread. 
            It predicts the states using the IMU measurements and corrects the states using 
            the orientation measurement.
        """
        
        # Extract orientation, linear acceleration, and angular velocity from the message
        q_gazebo = message.orientation
        a_gazebo = message.linear_acceleration
        W_gazebo = message.angular_velocity

        # Convert the orientation quaternion to a rotation matrix
        q = np.array([q_gazebo.x, q_gazebo.y, q_gazebo.z, q_gazebo.w])
        R_gi = q_to_R(q) # IMU to Gazebo frame
        R_fi = self.R_fg.dot(R_gi)  # IMU to FDCL frame (NED freme)

        # Remove gravity from the linear acceleration and transform to the FDCL frame
        a_i = np.array([a_gazebo.x, a_gazebo.y, a_gazebo.z])
        a_i = R_gi.T.dot(R_gi.dot(a_i) - self.ge3)

        # Transform the angular velocity to the FDCL frame
        W_i = np.array([W_gazebo.x, W_gazebo.y, W_gazebo.z])

        # Acquire a lock to ensure that the estimator is not modified by another thread
        with self.lock:
            # Predict the states using the IMU measurements
            self.estimator.prediction(a_i, W_i)

            # Correct the states using the orientation measurement
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
        """
        It is used to reset the UAV to its initial state.

        The function first waits for the set_model_state service to become available. 
            It then sets the initial position and attitude of the UAV to (0, 0, 0.2) and 
            (0, 0, 0, 1) respectively. It also sets the initial velocity of the UAV to zero.

        The function then creates a ModelState object with the initial pose and velocity of the UAV. 
            It calls the set_model_state service to reset the UAV to its initial state 
            using the ModelState object.

        Finally, the function prints a message indicating that the reset was successful.
        """
        
        # Wait for the set_model_state service to become available
        rospy.wait_for_service('/gazebo/set_model_state')
        
        # Set the initial position and attitude of the UAV
        init_position = Point(x=0.0, y=0.0, z=0.2)
        init_attitude = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        init_pose = Pose(position=init_position, orientation=init_attitude)

        # Set the initial velocity of the UAV to zero
        zero_motion = Vector3(x=0.0, y=0.0, z=0.0)
        init_velocity = Twist(linear=zero_motion, angular=zero_motion)

        # Create a ModelState object with the initial pose and velocity
        model_state = ModelState(model_name='uav', reference_frame='world', \
            pose=init_pose, twist=init_velocity)
        
        # Call the set_model_state service to reset the UAV to its initial state
        reset_state = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        reset_state(model_state)

        # Print a message indicating that the reset was successful
        print('Resetting UAV successful ..')


rover = Rover()
