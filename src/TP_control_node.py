#!/usr/bin/env python
import rospy 
from utils.task import *
from utils.mobile_manipulator import *
from utils.TurtlebotJointState import *
from utils.taskhandler import *
from utils.controller import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist


class TP_controller:
    def __init__(self, joint_state_topic, task_topic, cmd_vel_topic, cmd_dq_topic):
        rospy.init_node("TP_control_node")
        rospy.loginfo("Starting Task Priority Controller....")
        
        self.tasks = [ 
            EEPosition3D("End-effector position", np.array([0.2, 0.3, -0.1]).reshape(3,1), np.zeros((3,1)), np.eye(3,3))
        ] 

        self.robot              = MobileManipulator()
        # self.taskhandler        = taskHandler(self.robot)
        self.jointState         = TurtlebotJointState()
        self.controller         = Controller(self.tasks, self.robot)
        
        self.swiftpro_joint_state_sub   = rospy.Subscriber(joint_state_topic, JointState, self.jointstateCB)
        # self.odom_sub                   = rospy.Subscriber(odom_topic, type, self.odomCB)
        # self.task_sub                   = rospy.Subscriber(task_topic, type, self.taskCB)
        
        # PUBLISHERS
        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(1.0 / 10), self.controllerCallback)

        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(10), self.randomTasksCallback)

        # Error Publisher
        # self.err_pub = rospy.Publisher("/task_error", type, queue_size=1)
        
        # Command Velocity Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.dq_pub = rospy.Publisher(cmd_dq_topic, Float64MultiArray, queue_size=10)
        self.err_pub = rospy.Publisher("/error_topic", Float64MultiArray, queue_size=10)
        
    
    
    def jointstateCB(self, state):
        self.jointState.update(state)

        self.robot.update(self.jointState)
        
    def odomCB(self, odom):
        pass
    
    def taskCB(self,task_info):
        pass
    
    def controllerCallback(self, event):
        dq = self.controller.compute()

        # Create a Float64MultiArray message
        msg     = Float64MultiArray()
        err_msg = Float64MultiArray()
        # Fill the message with data
        msg.data = dq  # Example data, replace with your actual data

        err_msg.data  = ((self.tasks[0].err).reshape((1,3))).tolist()[0]
        # Publish the message
        self.dq_pub.publish(msg)

        self.err_pub.publish(err_msg)

    def randomTasksCallback(self, event):
        self.tasks = [ 
            EEPosition3D("End-effector position", np.array([-0.2 + 0.4*np.random.rand(), -0.3 + 0.6*np.random.rand(), -0.1]).reshape(3,1), np.zeros((3,1)), np.eye(3,3))
        ] 

        self.controller         = Controller(self.tasks, self.robot)

        
    def __send_base_command__(self,v,w):
        pass
    
    def __send_manipulator_command__(self,dq):
        pass
        

if __name__ == "__main__":
    ros_node = TP_controller("/turtlebot/joint_states", "abc","/cmd_vel", "/turtlebot/swiftpro/joint_velocity_controller/command")
    rospy.spin()