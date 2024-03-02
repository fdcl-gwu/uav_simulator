# from rover import rover

import datetime
import numpy as np

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry

class GpsNode(Node):

    def __init__(self):
        super().__init__('gps_sensor')
        self.subscription = self.create_subscription(
            Odometry,
            '/uav/gps',
            self.listener_callback,
            1)
        self.subscription  # prevent unused variable warning

        self.first = True

    def listener_callback(self, msg):
        # print(msg)
        pass


# def thread_gps():
#     print('GPS: thread starting ..')

#     rospy.Subscriber('uav_pos', Odometry, rover.ros_gps_callback)
#     rate = rospy.Rate(10) # 10 hz

#     freq = 10.0
#     t = datetime.datetime.now()
#     t_pre = datetime.datetime.now()
#     avg_number = 10

#     while not rospy.is_shutdown() and rover.on:

#         t = datetime.datetime.now()
#         dt = (t - t_pre).total_seconds()
#         if dt < 1e-3:
#             continue

#         freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
#         t_pre = t
#         rover.freq_gps = freq

#         rate.sleep()
    
#     print('GPS: thread closed!')
