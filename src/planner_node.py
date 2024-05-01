#!/usr/bin/env python
import rospy 

class plannerNode:
    def __init__(self):
        rospy.init_node("plannerNode")
        rospy.loginfo("Starting plannerNode.")

        pass


if __name__ == "__main__":
    ros_node = plannerNode()
    rospy.spin()