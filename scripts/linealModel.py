#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import JointState

# Constants
G = 9.81

class DoublePendulum_Lineal:
    def __init__(self, l1, m1, l2, m2):
        # Declare the links constants
        self.__m1 = m1
        self.__m2 = m2
        self.__l1 = l1
        self.__l2 = l2

        # Setup Variables to be used
        self.__states = {"theta1": np.pi/16, "theta2": 0.0, "theta_dot1": 0.0, "theta_dot2": 0.0}
        self._last_time = 0.0

        # Declare the joints message
        self.__joints = JointState()
        self.__joints.name = ['joint1', 'joint2']

    # Wrap to pi function
    def __wrap_to_Pi(self, theta):
        result = np.fmod((theta + np.pi),(2 * np.pi))
        if (result < 0):
            result += 2 * np.pi
        return result - np.pi

    # Get the time difference for dt
    def getDt(self):
        current_time = rospy.Time.now()
        self.__dt = (current_time - self._last_time).to_sec()
        self._last_time = current_time

    # Get the lineal model
    def solveEquations(self):
        # Wrap the angles to pi and integrate the velocities
        self.__states["theta1"] = self.__wrap_to_Pi(self.__states["theta1"] + self.__states["theta_dot1"]*self.__dt)
        self.__states["theta2"] = self.__wrap_to_Pi(self.__states["theta2"] + self.__states["theta_dot2"]*self.__dt)
        
        # Equations
        omega1_theta1 = ((-self.__l1*self.__m2 * self.__states["theta_dot1"] - self.__l2 * self.__m2 * 
                        self.__states["theta_dot2"] - G*(self.__m1 + self.__m2)) * (self.__l1*(self.__m1+self.__m2) - 
                        self.__l1 * self.__m2)) / (self.__l1 * (self.__m1 + self.__m2)- self.__l1 * self.__m2)**2

        omega1_theta2 = ((self.__l2*self.__m2*self.__states["theta_dot2"] + G*self.__m2 + self.__l1 * 
                        self.__m2*self.__states["theta_dot1"]) * ((self.__m1+self.__m2)*self.__l1 - 
                        self.__m2*self.__l1)) / ((self.__m1+self.__m2)*self.__l1 - self.__m2*self.__l1)**2

        omega2_theta1 = ((self.__l2*self.__m2*self.__states["theta_dot2"] + (self.__m1 + self.__m2) * 
                        (self.__l1*self.__states["theta_dot1"] + G)) * (self.__l2*(self.__m1+self.__m2) - 
                        self.__l2*self.__m2)) / (self.__l2*(self.__m1+self.__m2) - self.__l2 * self.__m2)**2

        omega2_theta2 = ((-self.__l2*self.__m2 * self.__states["theta_dot2"] + (self.__m1 + self.__m2) * 
                        (-self.__l1*self.__states["theta_dot1"] - G))*(self.__l2*(self.__m1 + self.__m2) - 
                        self.__l2*self.__m2)) / (self.__l2 * (self.__m1 + self.__m2) - self.__l2 * self.__m2)**2

        # Linear model with first grade equations
        matrix = np.array([
            [0, 0, 1, 0], # theta_dot1
            [0, 0, 0, 1], # theta_dot2
            [omega1_theta1, omega1_theta2, 0, 0],# omega_dot1 
            [omega2_theta1, omega2_theta2, 0, 0] # omega_dot2
        ])
        theta = np.array([
            [self.__states["theta1"]], 
            [self.__states["theta2"]], 
            [self.__states["theta_dot1"]], 
            [self.__states["theta_dot2"]]
        ])
        results = np.dot(matrix, theta)

        self.__states["theta_dot1"] += results[2][0]*self.__dt # Integrate the acceleration for joint1
        self.__states["theta_dot2"] += results[3][0]*self.__dt # Integrate the acceleration for joint2

    # Set the joints message
    def setJoints(self):
        self.__joints.header.stamp = rospy.Time.now()
        self.__joints.position = [self.__states["theta1"], self.__states["theta2"]] # Set the angles
        self.__joints.velocity = [self.__states["theta_dot1"], self.__states["theta_dot2"]] # Set the velocities
        return self.__joints

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("SLM_Sim")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~node_rate", 1000))

    # Setup de publishers
    pub = rospy.Publisher("/lineal_joints", JointState, queue_size = 10)

    # Classes
    pendulum = DoublePendulum_Lineal(l1=0.36, m1=0.75, l2=0.36, m2=0.75)

    print("The SLM sim is Running")
    try:
        while not rospy.is_shutdown():
            if not pendulum._last_time:
                pendulum._last_time = rospy.Time.now()
            else:
                pendulum.getDt()
                pendulum.solveEquations()

                # Publish the joint states
                pub.publish(pendulum.setJoints())
            # Wait and repeat
            rate.sleep()

    except rospy.ROSInterruptException:
        pass