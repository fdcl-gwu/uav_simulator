from rover import rover

import datetime
import numpy as np
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class ImuNode(Node):

    def __init__(self):
        super().__init__('imu_sensor')
        self.subscription = self.create_subscription(
            Imu,
            '/uav/imu',
            self.imu_callback,
            1)
        self.t_pre = 0.0
        self.count = 0

    def listener_callback(self, msg):
        # print(msg)
        # self.get_logger().info("IMU")
        self.count = self.count + 1
        t_now = time.time()

        if self.count > 100:
            self.count = 0
            print(t_now - self.t_pre)
        
        self.t_pre = t_now

        # pass
        # rover.ros_imu_callback(msg)



# def thread_imu():
#     print('IMU: thread starting ..')

#     max_freq = 200

#     imu_sub = ImuNode()

#     print('IMU: waiting for messages')
#     rclpy.spin(imu_sub)

#     # while rclpy.ok():
#         # rclpy.spin(imu_sub)
#         # rate.sleep()

#     # rospy.Subscriber('uav_imu', Imu, rover.ros_imu_callback)
#     # rate = rospy.Rate(100) # 100 hz

#     # freq = 100.0
#     # t = datetime.datetime.now()
#     # t_pre = datetime.datetime.now()
#     # avg_number = 100

#     # while not rospy.is_shutdown() and rover.on:
#     #     t = datetime.datetime.now()
#     #     dt = (t - t_pre).total_seconds()
#     #     if dt < 1e-6:
#     #         continue

#     #     freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
#     #     t_pre = t
#     #     rover.freq_imu = freq

#     #     rate.sleep()
    
#     print('IMU: thread closed!')
