from rover import rover

import datetime
import numpy as np
import rospy

from geometry_msgs.msg import Wrench
from geometry_msgs.msg import Vector3


def thread_control():
    """
    The function first creates a publisher for the uav_fm topic and sets the rate to 200 Hz. 
        It then initializes variables for calculating the frequency of the control updates.

    The function enters a loop that runs until the node is shutdown or the rover is turned off. 
        In each iteration of the loop, it calculates the time since the last iteration and checks 
        if it is too small. If it is not too small, it calculates the frequency of the control updates 
        and updates the freq_control variable in the rover object. It then runs the controller and 
        gets the resulting force and moment. If the motor is off or the mode is less than 2, it sets 
        the force and moment to zero. Otherwise, it sets the force and moment to the calculated values. 
        It then publishes the force and moment message and sleeps for the remainder of the loop period.

    Finally, the function prints a message indicating that the thread has been closed.
    """
    
    print('CONTROL: thread starting ..')

    # Create a publisher for the 'uav_fm' topic
    pub = rospy.Publisher('uav_fm', Wrench, queue_size=1)

    # Set the rate to 200 Hz
    rate = rospy.Rate(200)

    # Initialize variables for calculating the frequency of the control updates
    freq = 0.0
    t = datetime.datetime.now()
    t_pre = datetime.datetime.now()
    avg_number = 1000

    # Loop until the node is shutdown or the rover is turned off
    while not rospy.is_shutdown() and rover.on:

        # Calculate the time since the last loop iteration
        t = datetime.datetime.now()
        dt = (t - t_pre).total_seconds()

        # If the time difference is too small, skip this iteration
        if dt < 1e-6:
            continue
        
        # Calculate the frequency of the control updates
        freq = (freq * (avg_number - 1) + (1 / dt)) / avg_number
        t_pre = t
        rover.freq_control = freq

        # Run the controller and get the resulting force and moment
        fM = rover.run_controller()
        
        # If the motor is off or the mode is less than 2, set the force and moment to zero
        if (not rover.motor_on) or (rover.mode < 2):
            fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=0.0), \
                torque=Vector3(x=0.0, y=0.0, z=0.0))
        # Otherwise, set the force and moment to the calculated values
        else:
            fM_message = Wrench(force=Vector3(x=0.0, y=0.0, z=fM[0]), \
                torque=Vector3(x=fM[1], y=fM[2], z=fM[3]))
        
        # Publish the force and moment message
        pub.publish(fM_message)

        # Sleep for the remainder of the loop period
        rate.sleep()
    
    print('CONTROL: thread closed!')