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


class TP_controller:
    def __init__(self, joint_state_topic, task_topic, cmd_vel_topic, cmd_dq_topic):
        rospy.init_node("TP_control_node")
        rospy.loginfo("Starting Task Priority Controller....")
        
        # Task hierarchy definition
        limit_range_joint1   = np.array([-np.pi/2.0, np.pi/2.0]).reshape(1,2)
        threshold_joint1     = np.array([0.1, 0.2]).reshape(2,1)

        self.tasks = [ 
            # Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint1, np.zeros((1,1)), 0.1*np.eye(1,1), 1),
            # Limit2D("Manipulator Joint 2 Limitation", limit_range, threshold),
            # Limit2D("Manipulator Joint 3 Limitation", limit_range, threshold),
            # Limit2D("Manipulator Joint 4 Limitation", limit_range, threshold),
            # EEPosition3D("End-effector position", np.array([1.0, -0.2, -0.3]).reshape(3,1), np.zeros((3,1)), 0.4*np.eye(3,3))
            # EEPosition3D("End-effector position", np.array([0.4, 0.4, -0.3]).reshape(3,1), np.zeros((3,1)), 0.5*np.eye(3,3))
            # EEOrientation3D("End-effector orientation", np.array([1.7]).reshape(1,1), np.zeros((1,1)), 0.5*np.eye(1,1))
            # MMOrientation("Mobile Base orientation", np.array([1.7]).reshape(1,1), np.zeros((1,1)), 1.0*np.eye(1,1))
            MMPosition("Mobile base position", np.array([2.0, -0.2]).reshape(2,1), np.zeros((2,1)), 0.4*np.eye(2,2))
        ] 

        self.robot              = MobileManipulator()
        # self.taskhandler        = taskHandler(self.robot)
        weight_matrix           = np.diag([0.5, 1.0, 0.1, 1.0, 1.0, 1.0])
        self.controller         = Controller(self.tasks, self.robot, weight_matrix)
        
        self.swiftpro_joint_state_sub   = rospy.Subscriber(joint_state_topic, JointState, self.swiftProJointCB)
        
        # self.odom_sub                   = rospy.Subscriber(odom_topic, type, self.odomCB)
        # self.task_sub                   = rospy.Subscriber(task_topic, type, self.taskCB)
        
        self.listener = tf.TransformListener()
        # self.listener.waitForTransform("map", "turtlebot/kobuki/base_footprint", rospy.Time(), rospy.Duration(0.2))

        # PUBLISHERS
        # Error Publisher
        # self.err_pub = rospy.Publisher("/task_error", type, queue_size=1)
        self.point_marker_pub   = rospy.Publisher('~desierd_point_marker', Marker, queue_size=1)
        self.EEposition_marker_pub   = rospy.Publisher('~EEposition_point_marker', Marker, queue_size=1)
        
        # Command Velocity Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.dq_pub = rospy.Publisher(cmd_dq_topic, Float64MultiArray, queue_size=10)

        self.err_pub = rospy.Publisher("/error_topic", Float64MultiArray, queue_size=10)
        
        # Timer for TP controller (Velocity Commands)
        rospy.Timer(rospy.Duration(1.0 / 10), self.controllerCallback)

        # Timer for TP controller (Velocity Commands)
        # rospy.Timer(rospy.Duration(10), self.randomTasksCallback)
        rospy.Timer(rospy.Duration(01.0), self.mobileBaseJointCB)

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
        dq = self.controller.compute()

        # Create a Float64MultiArray message
        manipulator_msg     = Float64MultiArray()
        mobileBase_msg      = Twist()
        err_msg             = Float64MultiArray()

        # Fill the message with data
        mobileBase_msg.angular.z    = dq[1]
        # if mobileBase_msg.angular.z > 0.5:
        #     mobileBase_msg.linear.x     = 0.0
        # else:
        #     mobileBase_msg.angular.z    = 0.0
        mobileBase_msg.linear.x     = dq[0]
        
        manipulator_msg.data        = dq[2:6]

        err_msg.data  = ((self.tasks[0].err).reshape((2,1))).tolist()[0]


        # Publish the message
        self.cmd_pub.publish(mobileBase_msg)
        # self.dq_pub.publish(manipulator_msg)


        self.err_pub.publish(err_msg)

        desiredPoint = self.tasks[0].getDesired()
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
            m.pose.position.z = 0.0#p[2]
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

    def randomTasksCallback(self, event):
        self.tasks = [ 
            EEPosition3D("End-effector position", np.array([-0.2 + 0.4*np.random.rand(), -0.3 + 0.6*np.random.rand(), -0.4]).reshape(3,1), np.zeros((3,1)), np.eye(3,3))
        ] 

        self.controller         = Controller(self.tasks, self.robot)

        
    def __send_base_command__(self,v,w):
        pass
    
    def __send_manipulator_command__(self,dq):
        pass
        

if __name__ == "__main__":

    # ros_node = TP_controller("/turtlebot/joint_states", "abc","/cmd_vel", "/turtlebot/swiftpro/joint_velocity_controller/command")
    ros_node = TP_controller("/turtlebot/joint_states", "abc","/cmd_vel", "/turtlebot/swiftpro/joint_velocity_controller/command")

    
    rospy.spin()