#!/usr/bin/env python3
import rospy
import numpy as np
from geometry_msgs.msg import Polygon, Point32
from sensor_msgs.msg import JointState

# Constants
G = 9.81

class DoublePendulum_NoLineal:
    def __init__(self, l1, m1, l2, m2):
        # Declare the links constants
        self.__m1 = m1
        self.__m2 = m2
        self.__l1 = l1
        self.__l2 = l2

        # Setup Variables to be used
        self.__states = {"theta1": 0.0, "theta2": np.pi/4, "theta_dot1": 0.0, "theta_dot2": 0.0}
        self._last_time = 0.0

        # Declare the joints message
        self.__joints = JointState()
        self.__joints.name = ['joint1', 'joint2']

        # Declare the points messages
        self.__position = Polygon()
        self.__mass1 = Point32()
        self.__mass2 = Point32()

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

    # Get the no lineal model
    def solveEquations(self):
        # Wrap the angles to pi and integrate the velocities
        self.__states["theta1"] = self.__wrap_to_Pi(self.__states["theta1"] + self.__states["theta_dot1"]*self.__dt)
        self.__states["theta2"] = self.__wrap_to_Pi(self.__states["theta2"] + self.__states["theta_dot2"]*self.__dt)
        delta = self.__states["theta2"] - self.__states["theta1"]

        # Equations of motion
        theta_double_dot1 = ((self.__m2*self.__l1*self.__states["theta_dot1"]**2*np.sin(delta)*np.cos(delta) + 
                    self.__m2*G*np.sin(self.__states["theta2"])*np.cos(delta) + 
                    self.__m2*self.__l2*self.__states["theta_dot2"]**2*np.sin(delta) - 
                    (self.__m1+self.__m2)*G*np.sin(self.__states["theta1"])) / 
                    ((self.__m1+self.__m2)*self.__l1 - self.__m2*self.__l1*np.cos(delta)**2))

        theta_double_dot2 = ((-self.__m2*self.__l2*self.__states["theta_dot2"]**2*np.sin(delta)*np.cos(delta) + 
                    (self.__m1+self.__m2)*(G*np.sin(self.__states["theta1"])*np.cos(delta) 
                    - self.__l1*self.__states["theta_dot1"]**2*np.sin(delta) - 
                    G*np.sin(self.__states["theta2"]))) / 
                    ((self.__m1+self.__m2)*self.__l2 - self.__m2*self.__l2*np.cos(delta)**2))

        # Non linear model with first grade equations
        model = {
            "omega1" : self.__states["theta_dot1"],
            "omega_dot1" : theta_double_dot1,
            "omega2" : self.__states["theta_dot2"],
            "omega_dot2" : theta_double_dot2
        }

        self.__states["theta_dot1"] += model["omega_dot1"]*self.__dt # Integrate the acceleration for joint1
        self.__states["theta_dot2"] += model["omega_dot2"]*self.__dt # Integrate the acceleration for joint2
    
    # Get the joints message
    def getJoints(self):
        self.__joints.header.stamp = rospy.Time.now()
        self.__joints.position = [self.__states["theta1"], self.__states["theta2"]] # Set the angles
        return self.__joints

    # Get the points for the pendulum
    def getPoints(self):
        # Set the points for the first mass
        self.__mass1.x = self.__l1 * np.sin(self.__states["theta1"])
        self.__mass1.y = - self.__l1 * np.cos(self.__states["theta1"])

        # Set the points for the second mass
        self.__mass2.x = self.__mass1.x + self.__l2*np.sin(self.__states["theta2"])
        self.__mass2.y = self.__mass1.y - self.__l2*np.cos(self.__states["theta2"])

        self.__position.points = [self.__mass1, self.__mass2]
        return self.__position

if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("No_Lineal_Model_Pendulum")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~node_rate", 1000))

    # Setup de publishers
    joints_pub = rospy.Publisher("/noLineal_joints", JointState, queue_size = 10)
    points_pub = rospy.Publisher("/noLineal_points", Polygon, queue_size = 10)

    # Classes
    pendulum = DoublePendulum_NoLineal(l1=0.36, m1=0.75, l2=0.36, m2=0.75)

    # Wait to the rqt_multiplot to be ready
    rospy.sleep(5)
    print("The Non Lineal model pendulum is Running")
    try:
        while not rospy.is_shutdown():
            if not pendulum._last_time:
                pendulum._last_time = rospy.Time.now()
            else:
                pendulum.getDt()
                pendulum.solveEquations()

                # Publish the joint states
                joints_pub.publish(pendulum.getJoints())
                points_pub.publish(pendulum.getPoints())
            # Wait and repeat
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
