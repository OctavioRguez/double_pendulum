#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

# Import ROS messages
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

# Constants
G = 9.81

class Simple_Pendulum:
    def __init__(self):
        # Declare the links constants
        self._l = rospy.get_param("/pendulum/length", default = 1.0)

        # Setup variables to be used
        self._states = {"x1": rospy.get_param("/pendulum/theta", default = 0.0), 
                        "x2": rospy.get_param("/pendulum/omega", default = 0.0)}
        self._last_time = 0.0

        # Model matrix
        self._A = np.array([
            [0, 1], # x_dot1
            [-G/self._l, 0] # x_dot2
        ])
        self._B = np.array([
            [0],
            [1]
        ])
        self._k = np.array([[G/self._l - 2, -3]])

        # Declare the joints message
        self._joints = JointState()
        self._joints.name = ['joint1']

        # Declare the points messages
        self._pos = Point32()
    
    # Function for shutdown hook
    def _stop(self):
        print("Stopping")

    # Wrap to pi function
    def _wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi

    # Get the time difference for dt
    def _getDt(self):
        current_time = rospy.Time.now()
        dt = (current_time - self._last_time).to_sec()
        self._last_time = current_time
        return dt

    # Get the joints message
    def _setJoints(self):
        self._joints.header.stamp = rospy.Time.now()
        self._joints.position = [self._states["x1"]] # Set the angles
        self._joints.velocity = [self._states["x2"]] # Set the velocities  
    
    # Get the points for the pendulum
    def _setPosition(self):
        self._pos.x = self._l * np.sin(self._states["x1"])
        self._pos.y = - self._l * np.cos(self._states["x1"])