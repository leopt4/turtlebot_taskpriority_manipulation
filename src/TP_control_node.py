#!/usr/bin/env python
import rospy 
import tf
from utils.task import *
from utils.mobile_manipulator import *
from utils.controller import *
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped
import math
from turtlebot_taskpriority_manipulation.msg import TaskMsg

class TP_controller:
    def __init__(self):
        rospy.init_node("TP_control_node")
        rospy.loginfo("Starting Task Priority Controller....")

        self.tasks = []

        # gain = 0.1*np.eye(3,3)
        # gain[2,2] = 0.4

        gain = 0.2*np.eye(3,3)
        gain[2,2] = 0.02

        self.taskID = "0"
        self.robot              = MobileManipulator()
        self.controller         = Controller(self.tasks, self.robot, np.ones((6,1)))

        # task_desired    = np.array([1.0, 1.0, 1.0*np.pi/2]).reshape(3,1)
        # task_gain       = np.array([BASE_CONFIG_GAIN_X, BASE_CONFIG_GAIN_Y, BASE_CONFIG_GAIN_HEADING]).reshape(3,1)
        # task_feedForward= np.diag([0.0])
        # self.weight_matrix           = np.diag([BASE_CONFIG_WEIGHT_BASE_ROTATE,
        #                                         BASE_CONFIG_WEIGHT_BASE_TRANSLATE, 
        #                                         BASE_CONFIG_WEIGHT_JOINT_1, 
        #                                         BASE_CONFIG_WEIGHT_JOINT_2,
        #                                         BASE_CONFIG_WEIGHT_JOINT_3,
        #                                         BASE_CONFIG_WEIGHT_JOINT_4])
        # self.tasks = [
        #     MMConfiguration("Base Configuration", task_desired, task_feedForward, task_gain)
        # ] 
        # self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
        # self.taskID = "2"

        
        # SUBCRIBE
        # Subcribe to get manipulator state
        self.swiftpro_joint_state_sub   = rospy.Subscriber(joint_state_topic, JointState, self.swiftProJointCB)
        # Subcribe to get aruco pose
        self.goal2D_sub      = rospy.Subscriber(rviz_goal_topic, PoseStamped, self.set2DGoal)
        # Subcribe to get aruco pose
        self.goal3D_sub      = rospy.Subscriber(goal_topic, PoseStamped, self.set3DGoal)
        # Subcribe to get task
        self.task_sub      = rospy.Subscriber(task_topic, TaskMsg, self.setTask)

        self.listener = tf.TransformListener()

        # PUBLISHERS
        # Error Publisher
        self.point_marker_pub   = rospy.Publisher(point_marker_topic, Marker, queue_size=1)
        self.EEposition_marker_pub   = rospy.Publisher(EEposition_marker_topic, Marker, queue_size=1)
        
        # Command Velocity Publishers
        self.cmd_pub    = rospy.Publisher(cmd_vel_topic, Twist, queue_size=10)
        self.dq_pub     = rospy.Publisher(cmd_dq_topic, Float64MultiArray, queue_size=10)

        self.err_pub    = rospy.Publisher(task_error_topic, Float64MultiArray, queue_size=10)
        
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
        limit_joint_threshold_deactivate    = rospy.get_param('limit_joint_threshold_deactivate', default=0.08)

        # Task hierarchy definition
        self.limit_range_joint1   = np.array([limit_joint1_lower, limit_joint1_upper]).reshape(1,2)
        self.limit_range_joint2   = np.array([limit_joint2_lower, limit_joint2_upper]).reshape(1,2)
        self.limit_range_joint3   = np.array([limit_joint3_lower, limit_joint3_upper]).reshape(1,2)
        self.limit_range_joint4   = np.array([limit_joint4_lower, limit_joint4_upper]).reshape(1,2)

        self.threshold_joint      = np.array([limit_joint_threshold_activate, limit_joint_threshold_deactivate]).reshape(2,1)

        weight_base_rotate      = rospy.get_param('weight_base_rotate', default= 10.000)
        weight_base_translate   = rospy.get_param('weight_base_translate', default= 50.000)
        weight_joint1           = rospy.get_param('weight_joint1', default= 0.500)
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
        (translation, rotation) = self.listener.lookupTransform(FRAME_MAP, FRAME_BASE_FOOTPRINT, rospy.Time())
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
    
    def controllerCallback(self, event):
        dq = self.controller.compute()

        # Create a Float64MultiArray message
        manipulator_msg     = Float64MultiArray()
        mobileBase_msg      = Twist()
        err_msg             = Float64MultiArray()

        mobileBase_msg.angular.z    = -dq[0]
        mobileBase_msg.linear.x     = dq[1]
        manipulator_msg.data        = dq[2:6]

        # Publish the control message
        self.cmd_pub.publish(mobileBase_msg)
        self.dq_pub.publish(manipulator_msg)

        # Publish the task error message

        if self.taskID == "2" or self.taskID == "3":
            err_msg.data = []
            for i in range(0, (self.tasks[-1].error_task).shape[0]):
                err_msg.data.append(self.tasks[-1].error_task[i])

            self.err_pub.publish(err_msg)
        elif self.taskID == "4":
            err_msg.data = []
            for i in range(1, 4):
                err_msg.data.append(self.tasks[-i].error_task)

            self.err_pub.publish(err_msg)
        
        # Plot EE position and orientation Markers on the Rviz
        self.plot()

        # Plot task error
        if self.taskID == "2" or self.taskID == "3": 
            desiredPoint = np.zeros((3,1))
            desiredPoint[0] = self.tasks[-1].getDesired()[0]
            desiredPoint[1] = self.tasks[-1].getDesired()[1]
            if self.taskID == "2":
                desiredPoint[2] = 0.0
            elif self.taskID == "3":
                desiredPoint[2] = self.tasks[-1].getDesired()[2]
            self.publish_point(((desiredPoint).flatten()).tolist())
            
    def plot(self):
        EEpositionPoint = self.robot.getEEposition()
        EEorietation = self.robot.getEEorientation()
        
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
            FRAME_MAP,
            FRAME_EE
        )

    def plotMM(self):
        EEpositionPoint = self.robot.getMMposition()
        EEorietation = self.robot.getMMorientation()
        
        self.publish_EEpoint(((EEpositionPoint).flatten()).tolist())

        # Define the translation and rotation for the inverse TF (base_footprint to world)
        translation = (-(EEpositionPoint[0]*math.cos(EEorietation) + EEpositionPoint[1]*math.sin(EEorietation)), 
                    -(-EEpositionPoint[0]*math.sin(EEorietation) + EEpositionPoint[1]*math.cos(EEorietation)), 
                    0.0) # Set the x, y, z coordinates

        quaternion = tf.transformations.quaternion_from_euler(0, 0, -EEorietation)  # Convert euler angles to quaternion
        rotation = (quaternion[0], quaternion[1], quaternion[2], quaternion[3])
        
        # Publish the inverse TF from world to base_footprint
        tf.TransformBroadcaster().sendTransform(
            translation,
            rotation,
            rospy.Time.now(),
            FRAME_MAP,
            FRAME_EE
        )

    # Publish markers desired point
    def publish_point(self,p):
        if p is not None:
            m = Marker()
            m.header.frame_id = FRAME_MAP
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
            m.color.r = 1.0
            m.color.g = 0.0
            m.color.b = 0.0
            self.EEposition_marker_pub.publish(m)
    
    # Publish markers EE position and orientation
    def publish_EEpoint(self,p):
        if p is not None:
            m = Marker()
            m.header.frame_id = FRAME_MAP
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

    def set2DGoal(self, goalMsg):
        goal_x = goalMsg.pose.position.x
        goal_y = goalMsg.pose.position.y
        goal_z = MANI_SAFE_HEIGHT

        task_desired    = np.array([goal_x, goal_y, goal_z]).reshape(3,1)
        task_gain       = np.diag([EE_POS_GAIN_X, EE_POS_GAIN_Y, EE_POS_GAIN_Z])
        task_feedForward= np.array([EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]).reshape(3,1)
        self.weight_matrix           = np.diag([10.0,
                                                5.0, 
                                                EE_POS_WEIGHT_JOINT_1, 
                                                EE_POS_WEIGHT_JOINT_2,
                                                EE_POS_WEIGHT_JOINT_3,
                                                EE_POS_WEIGHT_JOINT_4])
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.eye(1,1), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.eye(1,1), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.eye(1,1), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.eye(1,1), 4),
            EEPosition3D("End-effector position", task_desired, task_feedForward, task_gain)
        ] 
        self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
        self.taskID = "3"
        # relX = goal_x - self.robot.eta[0]
        # relY = goal_y - self.robot.eta[1]
        # relAngle = math.atan2(relY, relX)
        
        # task_desired    = np.array([goal_x, goal_y, relAngle]).reshape(3,1)
        # task_gain       = np.diag([BASE_CONFIG_GAIN_X, BASE_CONFIG_GAIN_Y, BASE_CONFIG_GAIN_HEADING])
        # task_feedForward= np.array([BASE_CONFIG_FEEDFORWARD_X, BASE_CONFIG_FEEDFORWARD_Y, BASE_CONFIG_FEEDFORWARD_HEADING]).reshape(3,1)
        # self.weight_matrix           = np.diag([BASE_CONFIG_WEIGHT_BASE_ROTATE,
        #                                                 BASE_CONFIG_WEIGHT_BASE_TRANSLATE, 
        #                                                 BASE_CONFIG_WEIGHT_JOINT_1, 
        #                                                 BASE_CONFIG_WEIGHT_JOINT_2,
        #                                                 BASE_CONFIG_WEIGHT_JOINT_3,
        #                                                 BASE_CONFIG_WEIGHT_JOINT_4])
        # self.tasks = [
        #     MMConfiguration("Mobile Base configuration", task_desired, task_feedForward, task_gain)
        # ] 
        # self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
        # self.taskID = "2"
        # print("Point Mode: Move to point: " + str(goal_x) + str(goal_y) + str(goal_z))
        
    def set3DGoal(self, goalMsg):
        goal_x = goalMsg.pose.position.x
        goal_y = goalMsg.pose.position.y
        goal_z = goalMsg.pose.position.z

        task_desired    = np.array([goal_x, goal_y, goal_z]).reshape(3,1)
        task_gain       = np.diag([EE_POS_GAIN_X, EE_POS_GAIN_Y, EE_POS_GAIN_Z])
        task_feedForward= np.array([EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]).reshape(3,1)
        self.weight_matrix           = np.diag([10.0,
                                                5.0, 
                                                EE_POS_WEIGHT_JOINT_1, 
                                                EE_POS_WEIGHT_JOINT_2,
                                                EE_POS_WEIGHT_JOINT_3,
                                                EE_POS_WEIGHT_JOINT_4])
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.eye(1,1), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.eye(1,1), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.eye(1,1), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.eye(1,1), 4),
            EEPosition3D("End-effector position", task_desired, task_feedForward, task_gain)
        ] 
        self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
        self.taskID = "3"
        print("Point Mode: Move to point: " + str(goal_x) + str(goal_y) + str(goal_z))

    def setTask(self, taskMsg):
        # BASE ORIENTATION TASK
        if taskMsg.ids == "1":
            if self.taskID == "1":
                task_desired    = np.array([taskMsg.desired[0]]).reshape(1,1)
                self.tasks[0].setDesired(task_desired)
            else:
                task_name       = taskMsg.name
                task_desired    = np.array([taskMsg.desired[0]]).reshape(1,1)
                task_gain       = np.array([taskMsg.gain[0]]).reshape(1,1)
                task_feedForward= np.diag([taskMsg.feedForward[0]])
                self.weight_matrix           = np.diag([BASE_CONFIG_WEIGHT_BASE_ROTATE,
                                                        BASE_CONFIG_WEIGHT_BASE_TRANSLATE, 
                                                        BASE_CONFIG_WEIGHT_JOINT_1, 
                                                        BASE_CONFIG_WEIGHT_JOINT_2,
                                                        BASE_CONFIG_WEIGHT_JOINT_3,
                                                        BASE_CONFIG_WEIGHT_JOINT_4])
                self.tasks = [
                    MMOrientation(task_name, task_desired, task_feedForward, task_gain)
                ] 
                self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
                self.taskID = "1"
                print("Controller: Scan Object -> Create ID: " + self.taskID)

        # BASE CONFIGURATION TASK
        elif taskMsg.ids == "2":
            if self.taskID == "2":
                relX = np.array([taskMsg.desired[0]]) - self.robot.eta[0]
                relY = np.array([taskMsg.desired[1]]) - self.robot.eta[1]
                relAngle = math.atan2(relY, relX)

                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], relAngle]).reshape(3,1)
                self.tasks[0].setDesired(task_desired)
            else:
                task_name       = taskMsg.name

                relX = np.array([taskMsg.desired[0]]) - self.robot.eta[0]
                relY = np.array([taskMsg.desired[1]]) - self.robot.eta[1]
                relAngle = math.atan2(relY, relX)

                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], relAngle]).reshape(3,1)
                task_gain       = np.diag([taskMsg.gain[0], taskMsg.gain[1], taskMsg.gain[2]])
                task_feedForward= np.array([taskMsg.feedForward[0], taskMsg.feedForward[1], taskMsg.feedForward[2]])
                self.weight_matrix           = np.diag([BASE_CONFIG_WEIGHT_BASE_ROTATE,
                                                        BASE_CONFIG_WEIGHT_BASE_TRANSLATE, 
                                                        BASE_CONFIG_WEIGHT_JOINT_1, 
                                                        BASE_CONFIG_WEIGHT_JOINT_2,
                                                        BASE_CONFIG_WEIGHT_JOINT_3,
                                                        BASE_CONFIG_WEIGHT_JOINT_4])
                self.tasks = [
                    MMConfiguration(task_name, task_desired, task_feedForward, task_gain)
                ] 
                self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
                self.taskID = "2"
                print("Controller: Approach Base Object -> Create ID: " + self.taskID)
        
        # EE POSITION TASK
        elif taskMsg.ids == "3":
            if self.taskID == "3":
                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], taskMsg.desired[2]]).reshape(3,1)
                self.tasks[-1].setDesired(task_desired)
            else:
                task_name       = taskMsg.name
                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], taskMsg.desired[2]]).reshape(3,1)
                task_gain       = np.diag([taskMsg.gain[0], taskMsg.gain[1], taskMsg.gain[2]])
                task_feedForward= np.array([taskMsg.feedForward[0], taskMsg.feedForward[1], taskMsg.feedForward[2]]).reshape(3,1)
                self.weight_matrix           = np.diag([EE_POS_WEIGHT_BASE_ROTATE,
                                                        EE_POS_WEIGHT_BASE_TRANSLATE, 
                                                        EE_POS_WEIGHT_JOINT_1, 
                                                        EE_POS_WEIGHT_JOINT_2,
                                                        EE_POS_WEIGHT_JOINT_3,
                                                        EE_POS_WEIGHT_JOINT_4])
                self.tasks = [
                    Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.eye(1,1), 1),
                    Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.eye(1,1), 2),
                    Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.eye(1,1), 3),
                    Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.eye(1,1), 4),
                    EEPosition3D("End-effector position", task_desired, task_feedForward, task_gain)
                ] 
                self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
                self.taskID = "3"
                print("Controller: Approach Maipulator Object -> Create ID: " + self.taskID)

        # JOINT POSITION TASK
        elif taskMsg.ids == "4":
            if self.taskID == "4":
                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], taskMsg.desired[2]]).reshape(3,1)
                self.tasks[-1].setDesired(task_desired[0])
                self.tasks[-2].setDesired(task_desired[1])
                self.tasks[-3].setDesired(task_desired[2])
            else:
                task_name       = taskMsg.name
                task_desired    = np.array([taskMsg.desired[0], taskMsg.desired[1], taskMsg.desired[2]]).reshape(3,1)
                task_gain       = np.diag([taskMsg.gain[0], taskMsg.gain[1], taskMsg.gain[2]])
                task_feedForward= np.array([taskMsg.feedForward[0], taskMsg.feedForward[1], taskMsg.feedForward[2]]).reshape(3,1)
                self.weight_matrix           = np.diag([EE_POS_WEIGHT_BASE_ROTATE,
                                                        EE_POS_WEIGHT_BASE_TRANSLATE, 
                                                        EE_POS_WEIGHT_JOINT_1, 
                                                        EE_POS_WEIGHT_JOINT_2,
                                                        EE_POS_WEIGHT_JOINT_3,
                                                        EE_POS_WEIGHT_JOINT_4])
                self.tasks = [
                    Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.eye(1,1), 1),
                    Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.eye(1,1), 2),
                    Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.eye(1,1), 3),
                    Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.eye(1,1), 4),
                    JointPosition("Joint position", 3, task_desired[2], task_feedForward[2], task_gain[2,2]),
                    JointPosition("Joint position", 2, task_desired[1], task_feedForward[1], task_gain[1,1]),
                    JointPosition("Joint position", 1, task_desired[0], task_feedForward[0], task_gain[0,0])
                ] 
                self.controller         = Controller(self.tasks, self.robot, self.weight_matrix)
                self.taskID = "4"
                print("Controller: Approach Maipulator Object -> Create ID: " + self.taskID)
        
if __name__ == "__main__":

    ros_node = TP_controller()
    rospy.spin()