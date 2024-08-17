#!/usr/bin/env python

from rover import rover, reset_uav

from gui import GuiNode
from thread_imu import ImuNode
from thread_gps import GpsNode
from thread_control import ControlNode
from estimator import EstimatorNode
# from thread_log import thread_log

import numpy as np
import rclpy
import std_msgs
import threading
import time
from PyQt5.QtWidgets import QApplication

# run_uav_completed = False

# def run_uav():
#     global run_uav_completed

#     print("UAV thread started")

#     reset_uav()

#     executor = rclpy.executors.MultiThreadedExecutor(num_threads=3)
#     executor.add_node(ControlNode())
#     executor.add_node(ImuNode())
#     executor.add_node((GpsNode()))
    
#     try:
#         while rclpy.ok():
#             executor.spin_once()
#     finally:
#         print("UAV thread stopping")
#         executor.shutdown()
    
#     run_uav_completed = True
#     print("UAV thread closed")


if __name__ == '__main__':
    print("Main program started")

    rclpy.init()

    app = QApplication([])
    app.processEvents()

    nodes = []
    nodes.append(GuiNode())
    nodes.append(ControlNode())
    nodes.append(EstimatorNode())
    nodes.append(GpsNode())

    # Display the GUI
    nodes[0].show()

    num_nodes = len(nodes)

    executor = rclpy.executors.MultiThreadedExecutor(num_threads=num_nodes)
    for node in nodes:
        executor.add_node(node)
    # executor.add_node(gui)
    # executor.add_node(ControlNode())
    # executor.add_node(ImuNode())
    # executor.add_node(GpsNode())

    # threads = []
    # threads.append(threading.Thread(target=run_uav))

    # for thread in threads:
    #     thread.start()
    
    while rclpy.ok():
        executor.spin_once()
        app.processEvents() 
        app.quit()
    # try:
    #     while rclpy.ok():
    #         executor.spin_once()
    #         app.processEvents() 
    #     app.quit()
    # except Exception as e:
    #     print(e)
    #     print("Closing nodes")
        
    # print("GUI node closed")
    # gui.destroy_node()
    

    # for thread in threads:
    #     thread.join()
    
    # print("Waiting for all threads to end")
    # while run_uav_completed:
    #     time.sleep(1)

    # rclpy.shutdown()

    print("Main program ended")
    
