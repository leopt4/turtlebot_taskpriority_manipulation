#!/usr/bin/python
import rospy
import math
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState

class SwiftproController:
    def __init__(self) -> None:
        # Joint state subscriber
        self.joint_state_sub = rospy.Subscriber("joint_states", JointState, self.joint_state_callback)

        # Passive joint position publisher (ROS control command)
        self.passive_joint_pub = rospy.Publisher("command", Float64MultiArray, queue_size=10)
        
    def joint_state_callback(self, msg):
        if len(msg.name) == 4:
            # Convert angles to degrees
            joint2_angle = msg.position[1]/math.pi*180
            joint3_angle = msg.position[2]/math.pi*180
            
            # Calculate passive joint angles
            alpha2 = 90 - joint2_angle
            alpha3 = joint3_angle - 3.8
            passive_joint1_angle = (alpha2 + alpha3) - 176.11 + 90
            passive_joint2_angle = -90 + alpha2
            passive_joint3_angle = joint2_angle
            passive_joint5_angle = 90 - (alpha2 + alpha3 + 3.8)
            passive_joint7_angle = 176.11 - 180 - alpha3
            passive_joint8_angle = 48.39 + alpha3 - 44.55

            #Publish passive joint angles
            msgOut = Float64MultiArray()
            msgOut.data = [passive_joint1_angle/180.0 * math.pi,
                            passive_joint2_angle/180.0 * math.pi, 
                            passive_joint3_angle/180.0 * math.pi,
                            passive_joint5_angle/180.0 * math.pi,
                            passive_joint7_angle/180.0 * math.pi,
                            passive_joint8_angle/180.0 * math.pi]
            self.passive_joint_pub.publish(msgOut)

if __name__ == '__main__':
    rospy.init_node("test1_controller")
    robot = SwiftproController()
    rospy.spin()