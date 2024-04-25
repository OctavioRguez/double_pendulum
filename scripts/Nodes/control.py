#!/usr/bin/env python3

# Import python libraries
import rospy
import numpy as np

# Import ROS messages
from geometry_msgs.msg import Point32
from sensor_msgs.msg import JointState

# Import the Classes
from Pendulum import Simple_Pendulum

class Control(Simple_Pendulum):
    def __init__(self):
        # Call the parent class
        super(Control, self).__init__()

        # Setup de publishers
        self.__joints_pub = rospy.Publisher("/control_joints", JointState, queue_size = 10)
        self.__position_pub = rospy.Publisher("/control_position", Point32, queue_size = 10)
    
    # Control the lineal model
    def control(self):
        # Get the time difference
        dt = self._getDt()
        # Wrap the angles to pi and integrate the velocities
        self._states["x1"] = self._wrap_to_Pi(self._states["x1"] + self._states["x2"]*dt)

        # Solve model
        x = np.array([
            [self._states["x1"]], 
            [self._states["x2"]]
        ])
        x_dot = np.dot(self._A + np.dot(self._B, self._k), x) + np.random.normal(self._mean, self._std)
        self._states["x2"] += x_dot[1][0]*dt # Integrate the acceleration

        # Publish Joints
        self._setJoints()
        self.__joints_pub.publish(self._joints)

        # Publish Position
        self._setPosition()
        self.__position_pub.publish(self._pos)


if __name__=='__main__':
    # Initialise and Setup node
    rospy.init_node("Simple pendulum controller")

    # Configure the Node
    rate = rospy.Rate(rospy.get_param("~node_rate", 1000))

    # Classes
    pendulum = Control()

    # Wait to the rqt_multiplot to be ready
    rospy.sleep(5)

    rospy.on_shutdown(pendulum._stop)
    print("The Simple Pendulum Controller is Running")
    try:
        while not rospy.is_shutdown():
            if not pendulum._last_time:
                pendulum._last_time = rospy.Time.now()
            else:
                pendulum.control()

            # Wait and repeat
            rate.sleep()

    except rospy.ROSInterruptException:
        pass
