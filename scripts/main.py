#!/usr/bin/env python

from rover import rover, reset_uav

from gui import GuiNode
from thread_imu import ImuNode
from thread_gps import GpsNode
from thread_control import ControlNode
# from thread_log import thread_log

import numpy as np
import rclpy
import std_msgs
import threading
import time


def run_uav():

    print("ROS init")
    rclpy.init()

    reset_uav()

    # control = ControlNode()
    # imu = ImuNode()

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=4)
    # executor.add_node(control)
    # executor.add_node(imu)
    # executor.add_node((GpsNode()))
    executor.add_node((GuiNode()))

    # thread = threading.Thread(target=thread_gui)
    # thread.start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        rover.on = False
        print("\nReceived keyboard interrupt")
        rclpy.try_shutdown()
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()

    # thread.join()

    # Create threads
    # threads = []
    # threads.append(threading.Thread(target=thread_gui))
    # threads.append(threading.Thread(target=thread_log))
    
    
    print("ROS shutdown")

if __name__ == '__main__':
    run_uav()
