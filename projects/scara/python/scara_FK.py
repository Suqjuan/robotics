#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import PointStamped

import time
import math

class ScaraArm:
    def __init__(self):
        # Define constants
        self.H = 0.5
        self.L2 = 0.5
        self.L3 = 0.25
        
        self.goal = [.5, 0.0, 0.0]

        # Initialize node and publisher
        rospy.init_node('scara', anonymous=True)
        self.pub = rospy.Publisher('/scara_arm_controller/command', Float64MultiArray, queue_size=10)
        self.sub = rospy.Subscriber('/clicked_point', PointStamped, self.clickedPointCB)

    def clickedPointCB(self, msg):
        self.goal = [msg.point.x, msg.point.y, msg.point.z]

    def forward_kinematics(self, goal):
        
        # Extract goal position
        print(goal)
        px, py, pz = goal
        print(px, py, pz)

        # Solve equations
        r = math.sqrt(px**2 + py**2)
        D = (pz - self.H)**2 + r**2 - self.L2**2 - self.L3**2
        theta2 = math.atan2(-math.sqrt(1-D**2/(4*self.L2**2)), D/(2*self.L2))
        theta1 = math.atan2(py, px) - math.atan2(self.L3*math.sin(theta2), self.L2 + self.L3*math.cos(theta2))
        d3 = self.H - pz

        return theta1, theta2, d3

    def run(self):
        # Get user inputs

        # Perform forward kinematics

        theta1, theta2, d3 = self.forward_kinematics(self.goal)

        print(theta1, theta2, d3)

        # Publish to topic
        joint_angles = Float64MultiArray()
        joint_angles.data = [theta1, theta2, d3, 0]
        self.pub.publish(joint_angles)

        rospy.sleep(0.1)


if __name__ == '__main__':
    scara_arm = ScaraArm()
    while not rospy.is_shutdown():
        time.sleep(0.1)
        scara_arm.run()