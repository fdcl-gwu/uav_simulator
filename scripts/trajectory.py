import datetime
import numpy as np
from numpy import ndarray
import pdb


class Trajectory:
    def __init__(self):
        self.mode = 0                       # 0: idle, 1: warm-up, 2: take-off, 3: land, 4: stay, 5: circle
        self.is_mode_changed = False        # True if mode is changed
        self.is_landed = False              # True if the UAV is landed

        self.t0 = datetime.datetime.now()   # initial time
        self.t = 0.0                        # current time
        self.t_traj = 0.0                   # time elapsed since the start of the trajectory

        self.xd = np.zeros(3)               # desired position
        """desired position"""
        
        self.xd_dot = np.zeros(3)           # desired velocity
        self.xd_2dot = np.zeros(3)          # desired acceleration
        self.xd_3dot = np.zeros(3)          # desired jerk
        self.xd_4dot = np.zeros(3)          # desired snap

        self.b1d = np.zeros(3)              # (3x1 numpy array) desired direction of the first body axis
        self.b1d[0] = 1.0                
        self.b1d_dot = np.zeros(3)          # (3x1 numpy array) desired angular velocity of b1d
        self.b1d_2dot = np.zeros(3)         # (3x1 numpy array) desired angular acceleration of b1d

        self.x = np.zeros(3)                # current position
        self.v = np.zeros(3)                # current velocity
        self.a = np.zeros(3)                # current acceleration
        self.R = np.identity(3)             # (3x3 numpy array) current attitude of the UAV in SO(3)
        self.W = np.zeros(3)                # (3x1 numpy array) current angular velocity of the UAV [rad/s]

        self.x_init = np.zeros(3)           # initial position
        self.v_init = np.zeros(3)           # initial velocity
        self.a_init = np.zeros(3)           # initial acceleration
        self.R_init = np.identity(3)        # (3x3 numpy array) initial attitude of the UAV in SO(3)
        self.W_init = np.zeros(3)           # (3x1 numpy array) initial angular velocity of the UAV [rad/s]
        self.b1_init = np.zeros(3)          # (3x1 numpy array) initial direction of the first body axis
        self.theta_init = 0.0               # initial yaw angle

        self.trajectory_started = False     # True if a trajectory is started
        self.trajectory_complete = False    # True if a trajectory is complete
        
        # Manual mode
        self.manual_mode = False            # True if the UAV is in manual mode
        self.manual_mode_init = False       # True if the UAV is in manual mode for the first time
        self.x_offset = np.zeros(3)         # (3x1 numpy array) offset in the x-direction
        self.yaw_offset = 0.0               # offset in the yaw angle

        # Take-off
        self.takeoff_end_height = -1.0      # (m)
        self.takeoff_velocity = -1.0        # (m/s)

        # Landing
        self.landing_velocity = 1.0                 # (m/s)
        self.landing_motor_cutoff_height = -0.25    # (m)

        # Circle
        self.circle_center = np.zeros(3)    # (3x1 numpy array) center of the circle
        self.circle_linear_v = 1.0          # (m/s)
        self.circle_W = 1.2                 # (rad/s)
        self.circle_radius = 1.2            # (m)

        self.waypoint_speed = 2.0           # (m/s)

        self.e1 = np.array([1.0, 0.0, 0.0]) # (3x1 numpy array) unit vector in the x-direction


    def get_desired(self, mode, states, x_offset, yaw_offset):
        """
        Returns the desired state of the quadrotor based on the current mode and
        the current state of the quadrotor.

        Args:
            mode (str): The current mode of the quadrotor.
            states (tuple): A tuple containing the current state of the quadrotor.
                The tuple contains the position, velocity, acceleration, attitude,
                and angular velocity of the quadrotor.
            x_offset (float): The x offset of the quadrotor.
            yaw_offset (float): The yaw offset of the quadrotor.

        Returns:
            tuple: A tuple containing the desired state of the quadrotor. The tuple
                contains the desired position, velocity, acceleration, jerk, snap,
                b1d, b1d_dot, b1d_2dot, and a boolean indicating whether the
                quadrotor is landed or not.
        """
        # Unpack the current state of the quadrotor
        self.x, self.v, self.a, self.R, self.W = states
        # Set the x and yaw offsets
        self.x_offset = x_offset
        self.yaw_offset = yaw_offset

        # Check if the current mode is the same as the input mode
        if mode == self.mode:
            self.is_mode_changed = False
        else:
            self.is_mode_changed = True
            self.mode = mode
            self.mark_traj_start()

        # Calculate the desired state of the quadrotor
        self.calculate_desired()

        # Return the desired state of the quadrotor as a tuple
        desired = (self.xd, self.xd_dot, self.xd_2dot, self.xd_3dot, \
            self.xd_4dot, self.b1d, self.b1d_dot, self.b1d_2dot, self.is_landed)
        return desired

    
    def calculate_desired(self) -> None:
        """
        Calculates the desired state of the quadrotor based on the current mode.

        If the manual mode is enabled, the desired state is calculated using the
        manual method. Otherwise, the desired state is calculated based on the
        current mode of the quadrotor.

        Returns:
            None
        """
        # If manual mode is enabled, calculate desired state using manual method
        if self.manual_mode:
            self.manual()
            return

        # Calculate desired state based on current mode
        if self.mode == 0 or self.mode == 1:  # idle and warm-up
            self.set_desired_states_to_zero()
            self.mark_traj_start()
        elif self.mode == 2:  # take-off
            self.takeoff()
        elif self.mode == 3:  # land
            self.land()
        elif self.mode == 4:  # stay
            self.stay()
        elif self.mode == 5:  # circle
            self.circle()


    def mark_traj_start(self) -> None:
        """
        Resets the state of the quadrotor and marks the start of a new trajectory.

        This function resets the flags and variables that are used to track the state
        of the quadrotor and mark the start of a new trajectory. It sets the
        trajectory_started and trajectory_complete flags to False, sets the manual_mode
        flag to False, and sets the is_landed flag to False. It also resets the time
        variables and the x and yaw offsets, and updates the initial state of the quadrotor.

        Returns:
            None
        """
        # Reset flags and variables
        self.trajectory_started = False
        self.trajectory_complete = False
        self.manual_mode = False
        self.manual_mode_init = False
        self.is_landed = False

        # Reset time variables
        self.t = 0.0
        self.t_traj = 0.0
        self.t0 = datetime.datetime.now()

        # Reset x and yaw offsets
        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        # Update initial state of the quadrotor
        self.update_initial_state()


    def mark_traj_end(self, switch_to_manual=False) -> None:
        """
        Marks the end of the current trajectory and switches to manual mode if
        specified.

        This function sets the trajectory_complete flag to True, indicating that
        the current trajectory has been completed. If the switch_to_manual flag is
        set to True, the function also sets the manual_mode flag to True, indicating
        that the quadrotor should switch to manual mode.

        Args:
            switch_to_manual (bool): A boolean indicating whether the quadrotor
                should switch to manual mode after the trajectory is complete.

        Returns:
            None
        """
        # Set trajectory_complete flag to True
        self.trajectory_complete = True

        # If switch_to_manual flag is True, set manual_mode flag to True
        if switch_to_manual:
            self.manual_mode = True


    def set_desired_states_to_zero(self) -> None:
        """
        Sets the desired states of the quadrotor to zero.

        This function sets the desired position, velocity, acceleration, jerk, snap,
        b1d, b1d_dot, and b1d_2dot of the quadrotor to zero.

        Returns:
            None
        """
        # Set desired position, velocity, acceleration, jerk, and snap to zero
        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        # Set desired b1d, b1d_dot, and b1d_2dot to default values
        self.b1d = np.array([1.0, 0.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

    
    def set_desired_states_to_current(self) -> None:
        """
        Sets the desired states of the quadrotor to the current states.

        This function sets the desired position and velocity of the quadrotor to
        its current position and velocity, respectively. It sets the desired
        acceleration, jerk, snap, b1d, b1d_dot, and b1d_2dot of the quadrotor to
        zero. It also sets the desired attitude of the quadrotor to its current
        attitude.

        Returns:
            None
        """
        # Set desired position and velocity to current position and velocity
        self.xd = np.copy(self.x)
        self.xd_dot = np.copy(self.v)

        # Set desired acceleration, jerk, and snap to zero
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        # Set desired b1d, b1d_dot, and b1d_2dot to current b1, b1_dot, and b1_2dot
        self.b1d = self.get_current_b1()
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)


    def update_initial_state(self) -> None:
        """
        Updates the initial state of the quadrotor.

        This function updates the initial position, velocity, acceleration, attitude,
        and angular velocity of the quadrotor to its current state. It also updates
        the initial b1 vector and the initial pitch angle of the quadrotor.

        Returns:
            None
        """
        # Update initial position, velocity, acceleration, attitude, and angular velocity
        self.x_init = np.copy(self.x)
        self.v_init = np.copy(self.v)
        self.a_init = np.copy(self.a)
        self.R_init = np.copy(self.R)
        self.W_init = np.copy(self.W)

        # Update initial b1 vector and pitch angle
        self.b1_init = self.get_current_b1()
        self.theta_init = np.arctan2(self.b1_init[1], self.b1_init[0])

    
    def get_current_b1(self) -> np.array:
        """
        Calculates the current b1 vector of the quadrotor.

        This function calculates the current b1 vector of the quadrotor by
        multiplying the current rotation matrix R by the unit vector e1. It
        then calculates the pitch angle of the quadrotor using the arctangent
        of the y-component of the b1 vector divided by the x-component of the
        b1 vector. Finally, it returns the b1 vector with a zero z-component.

        Returns:
            np.array: The current b1 vector of the quadrotor.
            
        Explanation:
            Takes first col from self.R, which is a 3x1 matrix corresponding to b1
            see Figure 2.2: or see 2.3 Reference Frames (in Geometric Control paper on drive/Skywalker/Reading_Materials)
            "In other words, the position of the UAV, x ∈ R3, is defined
            as the origin of the b-frame in f -frame, and the attitude of the UAV, R ≜ R_(fb) ∈ SO(3),
            is defined as the rotation of the b-frame with respect to the f-frame"
        """
        # Calculate current b1 vector
        b1 = self.R.dot(self.e1)

        # Calculate pitch angle
        theta = np.arctan2(b1[1], b1[0])

        # Return b1 vector with zero z-component
        return np.array([np.cos(theta), np.sin(theta), 0.0])


    def waypoint_reached(self, waypoint: ndarray, current: ndarray, radius: float) -> bool:
        """
        Checks if the quadrotor has reached a waypoint.

        This function checks if the quadrotor has reached a waypoint by calculating
        the distance between the current position of the quadrotor and the waypoint.
        If the distance is less than the specified radius, the function returns True,
        indicating that the quadrotor has reached the waypoint. Otherwise, the function
        returns False.

        Args:
            waypoint (np.array): A 3D numpy array representing the waypoint to be reached.
            current (np.array): A 3D numpy array representing the current position of the quadrotor.
            radius (float): A float representing the radius around the waypoint within which the quadrotor is considered to have reached the waypoint.

        Returns:
            bool: True if the quadrotor has reached the waypoint, False otherwise.
        """
        # Calculate the distance between the current position and the waypoint
        delta = waypoint - current

        # Check if the distance is less than the specified radius
        if abs(np.linalg.norm(delta) < radius):
            return True
        else:
            return False


    def update_current_time(self):
        """
        Updates the current time of the trajectory.

        This function updates the current time of the trajectory by calculating
        the time elapsed since the start time of the trajectory. It uses the
        datetime module to get the current time and subtracts the start time
        of the trajectory from it to get the elapsed time in seconds.

        Returns:
            None
        """
        # Get the current time
        t_now = datetime.datetime.now()

        # Calculate the elapsed time since the start time of the trajectory
        self.t = (t_now - self.t0).total_seconds()


    def manual(self):
        # Check if manual mode has already been initialized
        if not self.manual_mode_init:
            # If not, set desired states to current states and update initial state
            self.set_desired_states_to_current()
            self.update_initial_state()

            # Set manual_mode_init flag to True, initialize x_offset and yaw_offset to zero, and print message
            self.manual_mode_init = True
            self.x_offset = np.zeros(3)
            self.yaw_offset = 0.0
            print('Switched to manual mode')
        
        # Calculate desired position xd and desired velocity xd_dot based on current position x and x_offset
        self.xd = self.x_init + self.x_offset
        self.xd_dot = (self.xd - self.x) / 1.0

        # Calculate theta based on yaw_offset and set b1d vector to unit vector pointing in direction of theta
        theta = self.theta_init + self.yaw_offset
        self.b1d = np.array([np.cos(theta), np.sin(theta), 0.0])


    #Explanation: see comments, and for variable meanings, see control.py
    def takeoff(self):
        if not self.trajectory_started:
            self.set_desired_states_to_zero()

            # Take-off starts from the current horizontal position.
            self.xd[0] = self.x[0]
            self.xd[1] = self.x[1]
            self.x_init = self.x

            # t=v/d, where t=t_traj, d=takeoff_end_height, and v=takeoff_velocity
            self.t_traj = (self.takeoff_end_height - self.x[2]) / self.takeoff_velocity

            # Set the takeoff attitude to the current attitude.
            self.b1d = self.get_current_b1()

            self.trajectory_started = True

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + self.takeoff_velocity * self.t
            self.xd_2dot[2] = self.takeoff_velocity
        else:
            if self.waypoint_reached(self.xd, self.x, 0.04):
                self.xd[2] = self.takeoff_end_height
                self.xd_dot[2] = 0.0

                if not self.trajectory_complete:
                    print('Takeoff complete\nSwitching to manual mode')
                
                self.mark_traj_end(True)


    def land(self):
        """
        It is used to generate a landing trajectory for the drone.

        The function first checks if the trajectory has already started. 
        If not, it sets the desired states to the current states and calculates the total 
            trajectory time based on the difference between the landing_motor_cutoff_height and 
            the current height, divided by the landing_velocity. It also sets the b1d vector to 
            the current attitude and updates the trajectory_started flag.

        The function then updates the current time and calculates the desired position and 
            acceleration based on the current time. If the drone has not yet reached the 
            landing_motor_cutoff_height, the function sets the desired position and acceleration 
            based on the landing velocity. If the drone has reached the landing_motor_cutoff_height, 
            the function sets the desired position to the landing_motor_cutoff_height and the desired 
            velocity to zero. If the trajectory is complete, the function marks it as such and 
            sets the is_landed flag to True.
        """
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.t_traj = (self.landing_motor_cutoff_height - self.x[2]) / self.landing_velocity

            # Set the landing attitude to the current attitude.
            self.b1d = self.get_current_b1()

            self.trajectory_started = True

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + self.landing_velocity * self.t
            self.xd_2dot[2] = self.landing_velocity
        else:
            if self.x[2] > self.landing_motor_cutoff_height:
                self.xd[2] = self.landing_motor_cutoff_height
                self.xd_dot[2] = 0.0

                if not self.trajectory_complete:
                    print('Landing complete')

                self.mark_traj_end(False)
                self.is_landed = True
            else:
                self.xd[2] = self.landing_motor_cutoff_height
                self.xd_dot[2] = self.landing_velocity

            
    def stay(self):
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.trajectory_started = True
        
        self.mark_traj_end(True)


    def circle(self):
        # Check if trajectory has already started
        if not self.trajectory_started:
            # If not, set desired states to current states and update trajectory_started flag
            self.set_desired_states_to_current()
            self.trajectory_started = True

            # Set circle_center to current position and calculate circle_W
            self.circle_center = np.copy(self.x)
            self.circle_W = 2 * np.pi / 8

            # Calculate total trajectory time based on circle_radius, circle_linear_v, and number of circles
            num_circles = 2
            self.t_traj = self.circle_radius / self.circle_linear_v \
                + num_circles * 2 * np.pi / self.circle_W

        # Update current time
        self.update_current_time()

        # Calculate desired position and velocity based on current time
        if self.t < (self.circle_radius / self.circle_linear_v):
            # If still in linear portion of trajectory, calculate position and velocity along axis 1
            self.xd[0] = self.circle_center[0] + self.circle_linear_v * self.t
            self.xd_dot[0] = self.circle_linear_v

        elif self.t < self.t_traj:
            # If in circular portion of trajectory, calculate position and velocity along both axes
            circle_W = self.circle_W
            circle_radius = self.circle_radius

            t = self.t - circle_radius / self.circle_linear_v
            th = circle_W * t

            circle_W2 = circle_W * circle_W
            circle_W3 = circle_W2 * circle_W
            circle_W4 = circle_W3 * circle_W

            # axis 1
            self.xd[0] = circle_radius * np.cos(th) + self.circle_center[0]
            self.xd_dot[0] = - circle_radius * circle_W * np.sin(th)
            self.xd_2dot[0] = - circle_radius * circle_W2 * np.cos(th)
            self.xd_3dot[0] = circle_radius * circle_W3 * np.sin(th)
            self.xd_4dot[0] = circle_radius * circle_W4 * np.cos(th)

            # axis 2
            self.xd[1] = circle_radius * np.sin(th) + self.circle_center[1]
            self.xd_dot[1] = circle_radius * circle_W * np.cos(th)
            self.xd_2dot[1] = - circle_radius * circle_W2 * np.sin(th)
            self.xd_3dot[1] = - circle_radius * circle_W3 * np.cos(th)
            self.xd_4dot[1] = circle_radius * circle_W4 * np.sin(th)

            # Calculate b1d vector and its derivatives
            w_b1d = 2.0 * np.pi / 10.0
            th_b1d = w_b1d * t

            self.b1d = np.array([np.cos(th_b1d), np.sin(th_b1d), 0])
            self.b1d_dot = np.array([- w_b1d * np.sin(th_b1d), \
                w_b1d * np.cos(th_b1d), 0.0])
            self.b1d_2dot = np.array([- w_b1d * w_b1d * np.cos(th_b1d),
                w_b1d * w_b1d * np.sin(th_b1d), 0.0])
        else:
            # If trajectory is complete, mark it as such
            self.mark_traj_end(True)
