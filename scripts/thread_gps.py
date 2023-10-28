from rover import rover

import datetime
import numpy as np
import rospy

from nav_msgs.msg import Odometry


def thread_gps():
    """
    The function called thread_gps that runs in a 
        separate thread and is used to handle GPS data for the rover.

    The function first subscribes to the uav_pos topic and sets the callback function 
        to rover.ros_gps_callback. It then sets the rate to 10 Hz.

    The function then initializes variables for calculating the frequency of the GPS updates. 
        It enters a loop that runs until the node is shutdown or the rover is turned off. 
        In each iteration of the loop, it calculates the time since the last iteration and 
        checks if it is too small. If it is not too small, it calculates the frequency of 
        the GPS updates and updates the freq_gps variable in the rover object. It then sleeps 
        for the remainder of the loop period.

    Finally, the function prints a message indicating that the thread has been closed.
    """
    
    print('GPS: thread starting ..')

    # Subscribe to the 'uav_pos' topic and set the callback function to rover.ros_gps_callback
    rospy.Subscriber('uav_pos', Odometry, rover.ros_gps_callback)

    # Set the rate to 10 Hz
    rate = rospy.Rate(10)

    # Initialize variables for calculating the frequency of the GPS updates
    freq = 10.0
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 10

    # Loop until the node is shutdown or the rover is turned off
    while not rospy.is_shutdown() and rover.on:

        # Calculate the time since the last loop iteration
        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()

        # If the time difference is too small, skip this iteration
        if dt < 1e-3:
            continue

        # Calculate the frequency of the GPS updates
        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        rover.freq_gps = freq

        # Sleep for the remainder of the loop period
        rate.sleep()
    
    print('GPS: thread closed!')