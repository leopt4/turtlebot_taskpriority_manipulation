#!/usr/bin/env python
import rospy 
import tf
from utils.task import *
from utils.mobile_manipulator import *
from utils.taskhandler import *
from utils.controller import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math

class TP_controller:
    def __init__(self):
        rospy.init_node("TP_control_node")
        rospy.loginfo("Starting Task Priority Controller....")

        self.readParams()

        self.tasks = []
            # Limit2D("Manipulator Joint 1 Limitation", self.limit_range_joint1, self.threshold_joint, np.zeros((1,1)), 0.5*np.eye(1,1), 1),
            # Limit2D("Manipulator Joint 2 Limitation", self.limit_range_joint2, self.threshold_joint, np.zeros((1,1)), 0.5*np.eye(1,1), 2),
            # Limit2D("Manipulator Joint 3 Limitation", self.limit_range_joint3, self.threshold_joint, np.zeros((1,1)), 0.5*np.eye(1,1), 3),
            # Limit2D("Manipulator Joint 4 Limitation", self.limit_range_joint4, self.threshold_joint, np.zeros((1,1)), 0.5*np.eye(1,1), 4),
            # EEPosition3D("End-effector position", np.array([2.0, 4.0, -0.2]).reshape(3,1), np.zeros((3,1)), 0.3*np.eye(3,3)),
            # EEConfiguration3D("End-effector configuration", np.array([-2.0, 2.3, -0.3, 0.3]).reshape(4,1), np.zeros((4,1)), 0.4*np.eye(4,4))
            # 
            # MMOrientation("Mobile Base orientation", np.array([1.57]).reshape(1,1), np.zeros((1,1)), 0.3*np.eye(1,1))
            # MMPosition("Mobile base position", np.array([1.0, 2.0]).reshape(2,1), np.zeros((2,1)), 0.2*np.eye(2,2))
            # MMConfiguration("Mobile base configuration", np.array([3.0, 2.0, 1.57]).reshape(3,1), np.zeros((3,1)), a)
            
        # ] 

        gain = 0.05*np.eye(3,3)
        gain[2,2] = 0.1

        self.tasks = [ 
            MMConfiguration("Mobile base configuration", np.array([2, 0, 0]).reshape(3,1), np.zeros((3,1)), gain)
        ] 

        self.robot              = MobileManipulator()
        # self.taskhandler        = taskHandler(self.robot)
        self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
        
        # SUBCRIBE
        # Subcribe to get manipulator state
        self.swiftpro_joint_state_sub   = rospy.Subscriber(self.joint_state_topic, JointState, self.swiftProJointCB)
        
        # Subcribe to get aruco pose
        self.aruco_pose_object_sub      = rospy.Subscriber(self.aruco_pose_topic, PoseStamped, self.arucoPoseCB)
        # self.task_sub                   = rospy.Subscriber(task_topic, type, self.taskCB)
        
        self.listener = tf.TransformListener()

        # PUBLISHERS
        # Error Publisher
        # self.err_pub = rospy.Publisher("/task_error", type, queue_size=1)
        self.point_marker_pub   = rospy.Publisher('~desierd_point_marker', Marker, queue_size=1)
        self.EEposition_marker_pub   = rospy.Publisher('~EEposition_point_marker', Marker, queue_size=1)
        
        # Command Velocity Publishers
        self.cmd_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=10)
        self.dq_pub = rospy.Publisher(self.cmd_dq_topic, Float64MultiArray, queue_size=10)

        self.err_pub = rospy.Publisher("/error_topic", Float64MultiArray, queue_size=10)
        
        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(0.1), self.controllerCallback)

        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(0.1), self.mobileBaseJointCB)

    def readParams(self):
        # Retrieve parameter from the parameter server
        limit_joint1_upper = rospy.get_param('limit_joint1_upper', default= 1.571)
        limit_joint1_lower = rospy.get_param('limit_joint1_lower', default=-1.571)
        limit_joint2_upper = rospy.get_param('limit_joint2_upper', default= 0.050)
        limit_joint2_lower = rospy.get_param('limit_joint2_lower', default=-1.571)
        limit_joint3_upper = rospy.get_param('limit_joint3_upper', default= 0.050)
        limit_joint3_lower = rospy.get_param('limit_joint3_lower', default=-1.571)
        limit_joint4_upper = rospy.get_param('limit_joint4_upper', default= 1.571)
        limit_joint4_lower = rospy.get_param('limit_joint4_lower', default=-1.571)

        limit_joint_threshold_activate      = rospy.get_param('limit_joint_threshold_activate', default=0.05)
        limit_joint_threshold_deactivate    = rospy.get_param('limit_joint_threshold_deactivate', default=0.10)

        # Task hierarchy definition
        self.limit_range_joint1   = np.array([limit_joint1_lower, limit_joint1_upper]).reshape(1,2)
        self.limit_range_joint2   = np.array([limit_joint2_lower, limit_joint2_upper]).reshape(1,2)
        self.limit_range_joint3   = np.array([limit_joint3_lower, limit_joint3_upper]).reshape(1,2)
        self.limit_range_joint4   = np.array([limit_joint4_lower, limit_joint4_upper]).reshape(1,2)

        self.threshold_joint      = np.array([limit_joint_threshold_activate, limit_joint_threshold_deactivate]).reshape(2,1)

        weight_base_rotate      = rospy.get_param('weight_base_rotate', default= 1.000)
        weight_base_translate   = rospy.get_param('weight_base_translate', default= 1.000)
        weight_joint1           = rospy.get_param('weight_joint1', default= 1.000)
        weight_joint2           = rospy.get_param('weight_joint2', default= 1.000)
        weight_joint3           = rospy.get_param('weight_joint3', default= 1.000)
        weight_joint4           = rospy.get_param('weight_joint4', default= 1.000)

        self.weight_matrix           = np.diag([weight_base_rotate,
                                                weight_base_translate, 
                                                weight_joint1, 
                                                weight_joint2, 
                                                weight_joint3,
                                                weight_joint4])
        
        self.joint_state_topic  = rospy.get_param('joint_state_topic', default= "/turtlebot/joint_states")
        self.aruco_pose_topic   = rospy.get_param('aruco_pose_topic', default= "/aruco_pose")
        self.task_topic         = rospy.get_param('task_topic', default= "None")
        self.cmd_vel_topic      = rospy.get_param('cmd_vel_topic', default= "/cmd_vel")
        self.cmd_dq_topic       = rospy.get_param('cmd_dq_topic', default= "/turtlebot/swiftpro/joint_velocity_controller/command")

    def mobileBaseJointCB(self, event):
        # Wait for the TFs to become available  
        (translation, rotation) = self.listener.lookupTransform("map", "turtlebot/kobuki/base_footprint", rospy.Time())
        # Extract x, y, and theta (in radians) from translation and rotation
        x       = translation[0]
        y       = translation[1]
        z       = translation[2]
        euler   = tf.transformations.euler_from_quaternion(rotation)
        theta   = euler[2]

        eta     = np.array([x,y,z,theta]).reshape((4,1))

        self.robot.updateMobileBaseState(eta)
    
    def swiftProJointCB(self, state):
        if len(state.name) == 4:
            self.robot.updateManipulatorState(state.position)
        
    def odomCB(self, odom):
        pass
    
    def taskCB(self,task_info):
        pass
    
    def controllerCallback(self, event):
        # if np.linalg.norm(self.robot.getMMposition()-self.tasks[-1].getDesired()[0:2]) < 0.2:
        #     self.controller.set_weightMatrix(np.diag([1.0, 1.0, 0.1, 0.3, 0.3, 0.3]))

        dq = self.controller.compute()

        # Create a Float64MultiArray message
        manipulator_msg     = Float64MultiArray()
        mobileBase_msg      = Twist()
        err_msg             = Float64MultiArray()

        # Fill the message with data
        mobileBase_msg.angular.z    = -dq[0]
        mobileBase_msg.linear.x     = dq[1]
        manipulator_msg.data        = dq[2:6]

        err_msg.data  = [np.linalg.norm((self.tasks[-1].err[0:3]))]
        

        # Publish the message
        self.cmd_pub.publish(mobileBase_msg)
        # if np.linalg.norm(self.robot.getMMposition()-self.tasks[4].getDesired()[0:2]) < 0.3:
        self.dq_pub.publish(manipulator_msg)
            # self.controller.set_weightMatrix(np.diag([1.0, 1.0, 0.1, 0.1, 0.1, 0.1]))

        self.err_pub.publish(err_msg)
        # Plot on the Rviz
        self.plot()

    def plot(self):
        desiredPoint = self.tasks[-1].getDesired()
        EEpositionPoint = self.robot.getEEposition()
        EEorietation = self.robot.getEEorientation()

        self.publish_point(((desiredPoint).flatten()).tolist())
        self.publish_EEpoint(((EEpositionPoint).flatten()).tolist())

        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (-(EEpositionPoint[0]*math.cos(EEorietation) + EEpositionPoint[1]*math.sin(EEorietation)), 
                       -(-EEpositionPoint[0]*math.sin(EEorietation) + EEpositionPoint[1]*math.cos(EEorietation)), 
                       -EEpositionPoint[2]) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, -EEorietation)  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            "map",
            "EE"
        )

    # Publish markers
    def publish_point(self,p):
        if p is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'point'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = 0.0#[2]
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.a = 1.0
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            self.EEposition_marker_pub.publish(m)
    
    # Publish markers
    def publish_EEpoint(self,p):
        if p is not None:
            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = rospy.Time.now()
            m.ns = 'point'
            m.id = 0
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = p[0]
            m.pose.position.y = p[1]
            m.pose.position.z = p[2]
            m.pose.orientation.x = 0
            m.pose.orientation.y = 0
            m.pose.orientation.z = 0
            m.pose.orientation.w = 1
            m.scale.x = 0.05
            m.scale.y = 0.05
            m.scale.z = 0.05
            m.color.a = 1.0
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            self.point_marker_pub.publish(m)
    
    def arucoPoseCB(self, arucoPose):
        obj_pos_x = float(arucoPose.pose.position.x)
        obj_pos_y = float(arucoPose.pose.position.y)
        obj_pos_z = float(arucoPose.pose.position.z)

        relX = obj_pos_x - self.robot.eta[0]
        relY = obj_pos_y - self.robot.eta[1]
        relDis = np.sqrt(relX*relX+relY*relY)
        relAngle = math.atan2(relY, relX)

        self.tasks[0].setDesired(np.array([obj_pos_x, obj_pos_y, relAngle]).reshape(3,1))

        

    def __send_base_command__(self,v,w):
        pass
    
    def __send_manipulator_command__(self,dq):
        pass
        

if __name__ == "__main__":

    ros_node = TP_controller()
    rospy.spin()