#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState


class Controller:
    def __init__(self):
        rospy.init_node('array_publisher', anonymous=True)
        self.sub = rospy.Subscriber('/joint_states', JointState, self.joint_states_callback)
        self.pub = rospy.Publisher('/p1_arm_controller/command', Float64MultiArray, queue_size=10)
        self.jointCmd = [0.0, 2.0, 0.0 , 0.0]
        self.jointStates  = JointState()
        self.jointStates.position = [0.0, 0.0, 0.0, 0.0]
        self.controllerCmd = Float64MultiArray()


    #Call back that updates the controller commands with what ever is in the GUI
    def joint_states_callback(self, msg):
        # Update the Float32MultiArray message with the joint positions
        self.jointStates.position = msg.position

    def updateControllerCmd(self):
        # Update the Float32MultiArray message with the joint positions
        self.controllerCmd.data = self.jointCmd
        self.pub.publish(self.controllerCmd)
    
    

if __name__ == '__main__':
    controller = Controller()
    while True:
        controller.updateControllerCmd()
        #print("CMD: ", controller.jointCmd, "CurrPos ", controller.jointStates.position)
