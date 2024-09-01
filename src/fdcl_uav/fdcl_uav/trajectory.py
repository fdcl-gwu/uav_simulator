import datetime
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import Int32

from uav_gazebo.msg import StateData, DesiredData

class TrajectoryNode(Node):

    def __init__(self):
        Node.__init__(self, 'trajectory')

        self.mode = 0
        self.is_mode_changed = False
        self.is_landed = False

        self.t0 = self.get_clock().now()
        self.t = 0.0
        self.t_traj = 0.0

        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = np.zeros(3)
        self.b1d[0] = 1.0
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

        self.x = np.zeros(3)
        self.v = np.zeros(3)
        self.a = np.zeros(3)
        self.R = np.identity(3)
        self.W = np.zeros(3)

        self.x_init = np.zeros(3)
        self.v_init = np.zeros(3)
        self.a_init = np.zeros(3)
        self.R_init = np.identity(3)
        self.W_init = np.zeros(3)
        self.b1_init = np.zeros(3)
        self.theta_init = 0.0

        self.trajectory_started = False
        self.trajectory_complete = False
        
        # Manual mode
        self.manual_mode = False
        self.manual_mode_init = False
        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        # Take-off
        self.takeoff_end_height = -1.0  # (m)
        self.takeoff_velocity = -1.0  # (m/s)

        # Landing
        self.landing_velocity = 1.0  # (m/s)
        self.landing_motor_cutoff_height = -0.25  # (m)

        # Circle
        self.circle_center = np.zeros(3)
        self.circle_linear_v = 1.0 
        self.circle_W = 1.2
        self.circle_radius = 1.2

        self.waypoint_speed = 2.0  # (m/s)

        self.e1 = np.array([1.0, 0.0, 0.0])

        self.init_subscribers()
        self.init_publishers()


    def init_subscribers(self):

        self.sub_states = self.create_subscription( \
            StateData,
            '/uav/states',
            self.states_callback,
            1)
        
        self.sub_mode = self.create_subscription( \
            Int32,
            '/uav/mode',
            self.mode_callback,
            1)
    

    def init_publishers(self):

        self.pub_trajectory = self.create_publisher( \
            DesiredData,
            '/uav/trajectory',
            1)


    def states_callback(self, msg):

        for i in range(3):
            self.x[i] = msg.position[i]
            self.v[i] = msg.velocity[i]
            self.a[i] = msg.acceleration[i]
            self.W[i] = msg.angular_velocity[i]

            for j in range(3):
                self.R[i, j] = msg.attitude[3*i + j]

        self.calculate_trajectory()
        self.publish_trajectory()

    
    def mode_callback(self, msg):
        self.mode = msg.data
        self.get_logger().info('Mode switched to {}'.format(self.mode))

        self.is_mode_changed = True
        self.mark_traj_start()


    def publish_trajectory(self):

        msg = DesiredData()

        for i in range(3):
            msg.position[i] = self.xd[i]
            msg.velocity[i] = self.xd_dot[i]
            msg.acceleration[i] = self.xd_2dot[i]
            msg.jerk[i] = self.xd_3dot[i]
            msg.snap[i] = self.xd_4dot[i]

            msg.b1[i] = self.b1d[i]
            msg.b1_dot[i] = self.b1d_dot[i]
            msg.b1_2dot[i] = self.b1d_2dot[i]

        msg.is_landed = self.is_landed

        self.pub_trajectory.publish(msg)

    
    def calculate_trajectory(self):
        if self.manual_mode:
            self.manual()
            return
        
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


    def mark_traj_start(self):
        self.trajectory_started = False
        self.trajectory_complete = False

        self.manual_mode = False
        self.manual_mode_init = False
        self.is_landed = False

        self.t = 0.0
        self.t_traj = 0.0
        self.t0 = self.get_clock().now()

        self.x_offset = np.zeros(3)
        self.yaw_offset = 0.0

        self.update_initial_state()


    def mark_traj_end(self, switch_to_manual=False):
        self.trajectory_complete = True

        if switch_to_manual:
            self.manual_mode = True


    def set_desired_states_to_zero(self):
        self.xd = np.zeros(3)
        self.xd_dot = np.zeros(3)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = np.array([1.0, 0.0, 0.0])
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)

    
    def set_desired_states_to_current(self):
        self.xd = np.copy(self.x)
        self.xd_dot = np.copy(self.v)
        self.xd_2dot = np.zeros(3)
        self.xd_3dot = np.zeros(3)
        self.xd_4dot = np.zeros(3)

        self.b1d = self.get_current_b1()
        self.b1d_dot = np.zeros(3)
        self.b1d_2dot = np.zeros(3)


    def update_initial_state(self):
        self.x_init = np.copy(self.x)
        self.v_init = np.copy(self.v)
        self.a_init = np.copy(self.a)
        self.R_init = np.copy(self.R)
        self.W_init = np.copy(self.W)

        self.b1_init = self.get_current_b1()
        self.theta_init = np.arctan2(self.b1_init[1], self.b1_init[0])

    
    def get_current_b1(self):
        b1 = self.R.dot(self.e1)
        theta = np.arctan2(b1[1], b1[0])
        return np.array([np.cos(theta), np.sin(theta), 0.0])


    def waypoint_reached(self, waypoint, current, radius):
        delta = waypoint - current
        
        if abs(np.linalg.norm(delta) < radius):
            return True
        else:
            return False


    def update_current_time(self):
        t_now = self.get_clock().now()
        self.t = float((t_now - self.t0).nanoseconds) * 1e-9


    def manual(self):
        if not self.manual_mode_init:
            self.set_desired_states_to_current()
            self.update_initial_state()

            self.manual_mode_init = True
            self.x_offset = np.zeros(3)
            self.yaw_offset = 0.0

            print('Switched to manual mode')
        
        self.xd = self.x_init + self.x_offset
        self.xd_dot = (self.xd - self.x) / 1.0

        theta = self.theta_init + self.yaw_offset
        self.b1d = np.array([np.cos(theta), np.sin(theta), 0.0])


    def takeoff(self):
        if not self.trajectory_started:
            self.set_desired_states_to_zero()

            # Take-off starts from the current horizontal position.
            self.xd = np.copy(self.x)

            self.xd_dot = np.zeros(3)
            self.xd_dot[2] = self.takeoff_velocity

            self.t_traj = abs((self.takeoff_end_height - self.x[2]) / self.takeoff_velocity)

            # Set the takeoff attitude to the current attitude.
            self.b1d = self.get_current_b1()

            self.trajectory_started = True

            takeoff_end_position = np.copy(self.x_init)
            takeoff_end_position[2] += self.takeoff_velocity * self.t_traj

            self.get_logger().info('Takeoff started')
            self.get_logger().info('Starting from ' + str(self.x))
            self.get_logger().info('Takeoff time ' + str(self.t_traj))
            self.get_logger().info('Reaching ' + str(takeoff_end_position))
            self.get_logger().info('Takeoff velocity ' + str(self.takeoff_velocity) + ' m/s')

        self.update_current_time()

        if self.t < self.t_traj:
            self.xd[2] = self.x_init[2] + self.takeoff_velocity * self.t
            self.xd_dot[2] = self.takeoff_velocity

        elif not self.waypoint_reached(self.xd, self.x, 0.04):
            self.xd[2] = self.takeoff_end_height
            xd_dot_gain = 0.1
            self.xd_dot = xd_dot_gain * (self.x - self.xd)

        else:
            self.xd[2] = self.takeoff_end_height
            self.xd_dot[2] = 0.0

            if not self.trajectory_complete:
                self.get_logger().info('Takeoff complete')
                self.get_logger().info('Switching to manual mode')

            self.mark_traj_end(True)
                

    def land(self):
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.t_traj = (self.landing_motor_cutoff_height - self.x[2]) / \
                self.landing_velocity

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
                    self.get_logger().info('Landing complete')


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
        if not self.trajectory_started:
            self.set_desired_states_to_current()
            self.trajectory_started = True

            self.circle_center = np.copy(self.x)
            self.circle_W = 2 * np.pi / 8

            num_circles = 2
            self.t_traj = self.circle_radius / self.circle_linear_v \
                + num_circles * 2 * np.pi / self.circle_W

        self.update_current_time()

        if self.t < (self.circle_radius / self.circle_linear_v):
            self.xd[0] = self.circle_center[0] + self.circle_linear_v * self.t
            self.xd_dot[0] = self.circle_linear_v

        elif self.t < self.t_traj:
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

            w_b1d = 2.0 * np.pi / 10.0
            th_b1d = w_b1d * t

            self.b1d = np.array([np.cos(th_b1d), np.sin(th_b1d), 0])
            self.b1d_dot = np.array([- w_b1d * np.sin(th_b1d), \
                w_b1d * np.cos(th_b1d), 0.0])
            self.b1d_2dot = np.array([- w_b1d * w_b1d * np.cos(th_b1d),
                w_b1d * w_b1d * np.sin(th_b1d), 0.0])
        else:
            self.mark_traj_end(True)


def main(args=None):
    print("Starting trajectory node")

    rclpy.init(args=args)

    trajectory = TrajectoryNode()

    try:
        rclpy.spin(trajectory)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory.destroy_node()

    print("Terminating trajectory node")