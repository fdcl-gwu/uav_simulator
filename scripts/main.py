#!/usr/bin/env python

from rover import Rover

import numpy as np
import rospy
import std_msgs

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Twist, Wrench
from geometry_msgs.msg import Vector3, Point, Quaternion
from gazebo_msgs.msg import ModelState 
from gazebo_msgs.srv import SetModelState
from sensor_msgs.msg import Imu
from std_msgs.msg import String


rover = Rover()


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
    

def run_uav():

    rospy.init_node('uav', anonymous=True)
    reset_uav()

    pub = rospy.Publisher('uav_fm', Wrench, queue_size=1)

    rospy.Subscriber('uav_pos', Odometry, rover.ros_gps_callback)
    rospy.Subscriber('uav_imu', Imu, rover.ros_imu_callback)

    rate = rospy.Rate(100) # 10hz

    while not rospy.is_shutdown():
        fM = rover.run_controller()

        fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=fM[0]), \
            torque=Vector3(x=fM[1], y=fM[2], z=fM[3]))
        pub.publish(fM_message)

        rate.sleep()


if __name__ == '__main__':
    run_uav()