from rover import rover

import datetime
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3


class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.publisher_ = self.create_publisher(Wrench, '/uav/fm', 10)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Wrench()

        fM = rover.run_controller()

        if (not rover.motor_on) or (rover.mode < 2):
            fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), \
                torque=Vector3(x=0.0, y=0.0, z=0.0))
        else:
            fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=fM[0][0]), \
                torque=Vector3(x=fM[1][0], y=fM[2][0], z=fM[3][0]))

        self.publisher_.publish(fM_message)


# def thread_control():
#     print('CONTROL: thread starting ..')

#     pub = rospy.Publisher('uav_fm', Wrench, queue_size=1)
#     rate = rospy.Rate(200) # 200 hz

#     freq = 0.0
#     t = datetime.datetime.now()
#     t_pre = datetime.datetime.now()
#     avg_number = 1000

#     while not rospy.is_shutdown() and rover.on:
#         t = datetime.datetime.now()
#         dt = (t - t_pre).total_seconds()
#         if dt < 1e-6:
#             continue
        
#         freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
#         t_pre = t
#         rover.freq_control = freq

#         fM = rover.run_controller()
        
#         if (not rover.motor_on) or (rover.mode < 2):
#             fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), \
#                 torque=Vector3(x=0.0, y=0.0, z=0.0))
#         else:
#             fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=fM[0]), \
#                 torque=Vector3(x=fM[1], y=fM[2], z=fM[3]))
#         pub.publish(fM_message)

#         rate.sleep()
    
#     print('CONTROL: thread closed!')
