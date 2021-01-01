#!/usr/bin/env python

from rover import rover, reset_uav

from gui import thread_gui
from thread_imu import thread_imu
from thread_gps import thread_gps
from thread_control import thread_control
from thread_log import thread_log

import numpy as np
import rospy
import std_msgs
import threading
import time


def run_uav():

    rospy.init_node('uav', anonymous=True)
    reset_uav()

    # Create threads
    t1 = threading.Thread(target=thread_control)
    t2 = threading.Thread(target=thread_imu)
    t3 = threading.Thread(target=thread_gps)
    t4 = threading.Thread(target=thread_gui)
    t5 = threading.Thread(target=thread_log)
    
    # Start threads.
    t1.start()
    t2.start()
    t3.start()
    t4.start()
    t5.start()

    # Wait until all threads close.
    t1.join()
    t2.join()
    t3.join()
    t4.join()
    t5.join()


if __name__ == '__main__':
    run_uav()
