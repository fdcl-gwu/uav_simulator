#!/usr/bin/env python

# from rover import rover, reset_uav

# from gui import thread_gui
from thread_imu import ImuNode
from thread_gps import GpsNode
# from thread_control import thread_control
# from thread_log import thread_log

import numpy as np
import rclpy
import std_msgs
import threading
import time


def run_uav():

    print("ROS init")
    rclpy.init()
    # reset_uav()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node((ImuNode()))
    executor.add_node((GpsNode()))

    try:
        executor.spin()
        rclpy.shutdown()
    except KeyboardInterrupt:
        print("\nReceived keyboard interrupt")
        pass

    # Create threads
    # threads = []
    # threads.append(threading.Thread(target=thread_control))
    # threads.append(threading.Thread(target=thread_imu))
    # threads.append(threading.Thread(target=thread_gps))
    # threads.append(threading.Thread(target=thread_gui))
    # threads.append(threading.Thread(target=thread_log))
    
    
    print("ROS shutdown")

if __name__ == '__main__':
    run_uav()
