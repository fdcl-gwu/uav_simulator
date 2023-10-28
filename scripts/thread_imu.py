from rover import rover

import datetime
import numpy as np
import rospy

from sensor_msgs.msg import Imu


def thread_imu():
    """
    The function first subscribes to the uav_imu topic and sets the callback function to 
        rover.ros_imu_callback. It then sets the rate to 100 Hz.

    The function then initializes variables for calculating the frequency of the IMU updates. 
        It enters a loop that runs until the node is shutdown or the rover is turned off. 
        In each iteration of the loop, it calculates the time since the last iteration and 
        checks if it is too small. If it is not too small, it calculates the frequency of the 
        IMU updates and updates the freq_imu variable in the rover object. It then sleeps for 
        the remainder of the loop period.

    Finally, the function prints a message indicating that the thread has been closed.
    """
    
    print('IMU: thread starting ..')

    # Subscribe to the 'uav_imu' topic and set the callback function to rover.ros_imu_callback
    rospy.Subscriber('uav_imu', Imu, rover.ros_imu_callback)

    # Set the rate to 100 Hz
    rate = rospy.Rate(100)

    # Initialize variables for calculating the frequency of the IMU updates
    freq = 100.0
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 100

    # Loop until the node is shutdown or the rover is turned off
    while not rospy.is_shutdown() and rover.on:

        # Calculate the time since the last loop iteration
        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()

        # If the time difference is too small, skip this iteration
        if dt < 1e-6:
            continue

        # Calculate the frequency of the IMU updates
        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        rover.freq_imu = freq

        # Sleep for the remainder of the loop period
        rate.sleep()
    
    print('IMU: thread closed!')