from rover import rover

import datetime
import numpy as np
import rospy

from nav_msgs.msg import Odometry


def thread_gps():
    print('GPS: thread starting ..')

    rospy.Subscriber('uav_pos', Odometry, rover.ros_gps_callback)
    rate = rospy.Rate(10) # 10 hz

    freq = 10.0
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 10

    while not rospy.is_shutdown() and rover.on:

        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()
        if dt < 1e-3:
            continue

        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        rover.freq_gps = freq

        rate.sleep()
    
    print('GPS: thread closed!')
