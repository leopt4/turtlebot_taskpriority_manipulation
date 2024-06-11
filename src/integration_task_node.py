#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped
import time
from std_srvs.srv import SetBool

import py_trees
import py_trees.decorators
import py_trees.display
from py_trees.blackboard import Blackboard

from utils.task import *
from turtlebot_taskpriority_manipulation.msg import TaskMsg
from turtlebot_taskpriority_manipulation.msg import CmdMsg
from std_msgs.msg import Float64MultiArray
from config import *
from nav_msgs.msg import Odometry
import tf
import operator 


class ScanObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ScanObject, self).__init__(name)
        self.blackboard = Blackboard()
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "wpt", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "wpt", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "detect_goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.WRITE)
        

    def setup(self):
        self.logger.debug("  %s [ScanObject::setup()]" % self.name)

        self.blackboard.n_object = 0

        self.logger.debug("  %s [ScanObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ScanObject::initialise()]" % self.name)

        self.blackboard.detect_enable = True
        self.blackboard.detect_goal = False

        # SUBSCRIBERS
        # Subcribe to get aruco pose
        self.aruco_pose_object_sub      = rospy.Subscriber(aruco_pose_topic, PoseStamped, self.arucoPoseCB)

        # Subcribe to get aruco pose
        self.approach_wpt_sub           = rospy.Subscriber(wpt_topic, PoseStamped, self.wptCB)

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

    def update(self):
        cmd_msg = CmdMsg()
        cmd_msg.ids = "1"
        cmd_msg.name = "ScanObject"
        cmd_msg.desired = []
        self.cmd_publisher.publish(cmd_msg)

        if self.blackboard.detect_goal == True and self.blackboard.n_object < N_OBJECT:
            time.sleep(0.5)
            self.blackboard.detect_enable = False
            self.logger.debug("  %s [ScanObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ScanObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ScanObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    # Aruco pose detector callback
    def arucoPoseCB(self, arucoPose):
        obj_pos_x = float(arucoPose.pose.position.x)
        obj_pos_y = float(arucoPose.pose.position.y)
        obj_pos_z = float(arucoPose.pose.position.z)

        if math.sqrt((obj_pos_x-GOAL_PLACE_X)**2 + (obj_pos_y-GOAL_PLACE_Y)**2) > 0.2 and self.blackboard.detect_enable == True:
            self.blackboard.goal = [obj_pos_x, obj_pos_y]# [1.18, 0.02]
            self.blackboard.detect_goal = True

    def wptCB(self, wpt):
        wpt_pos_x = float(wpt.pose.position.x)
        wpt_pos_y = float(wpt.pose.position.y)
        wpt_pos_z = float(wpt.pose.position.z)

        # if self.blackboard.detect_enable == True and self.blackboard.detect_goal == True:
        #     self.blackboard.wpt = [wpt_pos_x, wpt_pos_y]# [1.18, 0.02]
        #     self.blackboard.detect_enable = False

class ApproachBaseObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachBaseObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "wpt", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "wpt", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.WRITE)

    def setup(self):
        self.logger.debug("  %s [ApproachBaseObject::setup()]" % self.name)

        self.err = np.array([np.inf, np.inf])
        self.logger.debug("  %s [ApproachBaseObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBaseObject::initialise()]" % self.name)  
    
        # SUBSCRIBERS
        if MODE == "SIL":
            self.odom_sub = rospy.Subscriber(odom_SIL_topic, Odometry, self.odomCallback) 
        elif MODE == "HIL":
            self.odom_sub = rospy.Subscriber(odom_HIL_topic, Odometry, self.odomCallback) 

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # Publisher for sending task to the TP control node
        self.goal_publisher = rospy.Publisher(rviz_goal_topic, PoseStamped, queue_size=10)

        # Wait 0.2s to init pub and sub
        time.sleep(1.0)
           
    def update(self):
        cmd_msg = CmdMsg()
        cmd_msg.ids = "2"
        cmd_msg.name = "ApproachBaseObject"
        dx = self.blackboard.goal[0] - self.eta[0]
        dy = self.blackboard.goal[1] - self.eta[1]
        d = np.sqrt(dx*dx + dy*dy)
        cmd_msg.desired = [self.blackboard.goal[0] - dx/d*0.3, self.blackboard.goal[1] - dy/d*0.3]
        self.cmd_publisher.publish(cmd_msg)

        wpt_pos = PoseStamped()
        wpt_pos.pose.position.x = self.blackboard.goal[0] - dx/d*0.3
        wpt_pos.pose.position.y = self.blackboard.goal[1] - dy/d*0.3
        self.goal_publisher.publish(wpt_pos)

        relX = wpt_pos.pose.position.x - self.eta[0]
        relY = wpt_pos.pose.position.y - self.eta[1]
        error = np.sqrt(relX**2 + relY**2)

        if  error < BASE_CONFIG_DIS_ERROR_FINISH:
            self.blackboard.detect_enable = True
            for i in range(0,10000):
                a = 1
            self.logger.debug("  %s [ApproachBaseObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBaseObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBaseObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def odomCallback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw]).reshape(-1,1)  

class ApproachManipulatorObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachManipulatorObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "detect_enable", access=py_trees.common.Access.WRITE)
        

    def setup(self):
        self.logger.debug("  %s [ApproachManipulatorObject::setup()]" % self.name)
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 4), 
            EEPosition3D("End-effector position", np.array([0.0, 0.0, 0.0]).reshape(3,1), np.zeros((3,1)), np.zeros((3,3)))
        ] 

        self.logger.debug("  %s [ApproachManipulatorObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachManipulatorObject::initialise()]" % self.name)    

        self.blackboard.detect_enable == False

        # self.blackboard.goal = [0.6, 0.1]
        # self.blackboard.detect_enable = False
        
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

        self.desired_pos_z = MANI_SAFE_HEIGHT
        self.err = np.array([np.inf, np.inf, np.inf])
        
    def update(self):
        task_msg = TaskMsg()
        task_msg.ids = "3"
        task_msg.name = self.tasks[-1].name
        task_msg.desired = [self.blackboard.goal[0], self.blackboard.goal[1], self.desired_pos_z]
        task_msg.gain = [EE_POS_GAIN_X, EE_POS_GAIN_Y, EE_POS_GAIN_Z]
        # task_msg.gain = [0, 0, 0]
        task_msg.feedForward = [EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]
        self.task_publisher.publish(task_msg)

        cmd_msg = CmdMsg()
        cmd_msg.ids = "3"
        cmd_msg.name = "ApproachManipulatorObject"
        cmd_msg.desired = []
        self.cmd_publisher.publish(cmd_msg)

        if np.linalg.norm(self.err) < EE_POS_ERROR_FINISH and self.desired_pos_z < -0.20:
            self.desired_pos_z = MANI_PICK_HEIGHT
            self.err = np.array([np.inf, np.inf, np.inf])
            time.sleep(2)
        
        if np.linalg.norm(self.err) < EE_POS_ERROR_PICK_OBJ and self.desired_pos_z > -0.20: 
            self.logger.debug("  %s [ApproachManipulatorObject::Update() SUCCESS]" % self.name)

            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachManipulatorObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachManipulatorObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 3:
            self.err = np.array([err.data[0], err.data[1], err.data[2]])

class PickObject (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PickObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [PickObject::setup()]" % self.name)
    
    def initialise(self):
        self.logger.debug("  %s [PickObject::initialise()]" % self.name)

    def update(self):
        succes = self.enable_suction()
        if succes:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def enable_suction(self):
        rospy.logwarn("Calling enable suction")
        rospy.wait_for_service(suction_service)
        path = []
        try:
            enable_suction = rospy.ServiceProxy(suction_service, SetBool)
            resp = enable_suction(True)
            
            return resp.success
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

class HandleManipulatorObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(HandleManipulatorObject, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [HandleManipulatorObject::setup()]" % self.name)
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 4), 
            JointPosition("Joint position", 3, np.array([0.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1))),
            JointPosition("Joint position", 2, np.array([0.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1))),
            JointPosition("Joint position", 1, np.array([np.pi/2.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1)))
        ] 

        self.logger.debug("  %s [HandleManipulatorObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [HandleManipulatorObject::initialise()]" % self.name)     
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

        self.err = np.array([np.inf, np.inf, np.inf])
        
    def update(self):
        task_msg = TaskMsg()
        task_msg.ids = "4"
        task_msg.name = self.tasks[-1].name
        task_msg.desired = [self.tasks[-1].getDesired(), self.tasks[-2].getDesired(), self.tasks[-3].getDesired()]
        task_msg.gain = [JOINT_POS_GAIN_1, JOINT_POS_GAIN_2, JOINT_POS_GAIN_3]
        task_msg.feedForward = [JOINT_POS_FEEDFORWARD_1, JOINT_POS_FEEDFORWARD_2, JOINT_POS_FEEDFORWARD_3]
        self.task_publisher.publish(task_msg)

        cmd_msg = CmdMsg()
        cmd_msg.ids = "4"
        cmd_msg.name = "HandleObject"
        cmd_msg.desired = []
        self.cmd_publisher.publish(cmd_msg)

        if abs(self.err[0]) < JOINT_POS_ERROR_FINISH and abs(self.err[1]) < JOINT_POS_ERROR_FINISH and abs(self.err[2]) < JOINT_POS_ERROR_FINISH:
            time.sleep(1000)
            self.logger.debug("  %s [HandleManipulatorObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [HandleManipulatorObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [HandleManipulatorObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 3:
            self.err = np.array([err.data[0], err.data[1], err.data[2]])

class LetObject (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(LetObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.WRITE)
    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
    
    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
        self.blackboard.n_object = 1
        succes = self.disable_suction()
        if succes:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def disable_suction(self):
        rospy.logwarn("Calling enable suction")
        rospy.wait_for_service(suction_service)
        path = []
        try:
            enable_suction = rospy.ServiceProxy(suction_service, SetBool)
            resp = enable_suction(False)
            
            return resp.success
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

class ApproachBasePlace(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachBasePlace, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [ApproachBasePlace::setup()]" % self.name)

        self.err = np.array([np.inf, np.inf])
        self.logger.debug("  %s [ApproachBasePlace::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBasePlace::initialise()]" % self.name)  

        # SUBSCRIBERS
        if MODE == "SIL":
            self.odom_sub = rospy.Subscriber(odom_SIL_topic, Odometry, self.odomCallback) 
        elif MODE == "HIL":
            self.odom_sub = rospy.Subscriber(odom_HIL_topic, Odometry, self.odomCallback) 

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # Publisher for sending task to the TP control node
        self.goal_publisher = rospy.Publisher(rviz_goal_topic, PoseStamped, queue_size=10)

        # Wait 0.2s to init pub and sub
        time.sleep(1.0)
           
    def update(self):
        cmd_msg = CmdMsg()
        cmd_msg.ids = "2"
        cmd_msg.name = "ApproachBasePlace"
        cmd_msg.desired = [GOAL_PLACE_X, GOAL_PLACE_Y]
        self.cmd_publisher.publish(cmd_msg)

        obj_pos = PoseStamped()
        obj_pos.pose.position.x = GOAL_PLACE_X
        obj_pos.pose.position.y = GOAL_PLACE_Y - 0.1 + 0.2 * self.blackboard.n_object
        self.goal_publisher.publish(obj_pos)

        relX = GOAL_PLACE_X - self.eta[0]
        relY = GOAL_PLACE_Y - self.eta[1]
        error = np.sqrt(relX**2 + relY**2)

        if  error < BASE_CONFIG_PLACE_DIS_ERROR_FINISH:
            self.logger.debug("  %s [ApproachBasePlace::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBasePlace::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBasePlace::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))

    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def odomCallback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw]).reshape(-1,1)  

class HandleManipulatorPlace(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(HandleManipulatorPlace, self).__init__(name)

    def setup(self):
        self.logger.debug("  %s [HandleManipulatorPlace::setup()]" % self.name)
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 4), 
            JointPosition("Joint position", 3, np.array([0.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1))),
            JointPosition("Joint position", 2, np.array([0.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1))),
            JointPosition("Joint position", 1, np.array([np.pi/2.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1)))
        ] 

        self.logger.debug("  %s [HandleManipulatorPlace::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [HandleManipulatorPlace::initialise()]" % self.name)     
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # Publisher for sending task to the TP control node
        self.cmd_publisher = rospy.Publisher(cmd_topic, CmdMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

        self.err = np.array([np.inf, np.inf, np.inf])
        
    def update(self):
        task_msg = TaskMsg()
        task_msg.ids = "4"
        task_msg.name = self.tasks[-1].name
        task_msg.desired = [self.tasks[-1].getDesired(), self.tasks[-2].getDesired(), self.tasks[-3].getDesired()]
        task_msg.gain = [JOINT_POS_GAIN_1, JOINT_POS_GAIN_2, JOINT_POS_GAIN_3]
        task_msg.feedForward = [JOINT_POS_FEEDFORWARD_1, JOINT_POS_FEEDFORWARD_2, JOINT_POS_FEEDFORWARD_3]
        self.task_publisher.publish(task_msg)

        cmd_msg = CmdMsg()
        cmd_msg.ids = "4"
        cmd_msg.name = "HandleObject"
        cmd_msg.desired = []
        self.cmd_publisher.publish(cmd_msg)

        if abs(self.err[0]) < JOINT_POS_ERROR_FINISH and abs(self.err[1]) < JOINT_POS_ERROR_FINISH and abs(self.err[2]) < JOINT_POS_ERROR_FINISH:
            self.logger.debug("  %s [HandleManipulatorPlace::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [HandleManipulatorPlace::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [HandleManipulatorPlace::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 3:
            self.err = np.array([err.data[0], err.data[1], err.data[2]])

class ApproachManipulatorPlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachManipulatorPlaceObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [ApproachManipulatorPlaceObject::setup()]" % self.name)
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 4), 
            EEPosition3D("End-effector position", np.array([0.0, 0.0, 0.0]).reshape(3,1), np.zeros((3,1)), np.zeros((3,3)))
        ] 

        self.logger.debug("  %s [ApproachManipulatorPlaceObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachManipulatorPlaceObject::initialise()]" % self.name)     
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

        self.desired_pos_z = MANI_PICK_HEIGHT

        self.err = np.array([np.inf, np.inf, np.inf])
        
    def update(self):
        task_msg = TaskMsg()
        task_msg.ids = "3"
        task_msg.name = self.tasks[-1].name
        task_msg.desired        = [GOAL_PLACE_X, GOAL_PLACE_Y - 0.1 + 0.2 * self.blackboard.n_object, MANI_PLACE_HEIGHT]
        task_msg.gain           = [EE_POS_GAIN_X, EE_POS_GAIN_Y, 0.09]
        task_msg.feedForward    = [EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]
        self.task_publisher.publish(task_msg)

        if abs(self.err[2]) < EE_POS_ERROR_PICK_OBJ:
            self.logger.debug("  %s [ApproachManipulatorPlaceObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachManipulatorPlaceObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachManipulatorPlaceObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 3:
            self.err = np.array([err.data[0], err.data[1], err.data[2]])

#   Create Behavior trees function
def create_tree():
    # Special py_trees behavior
    # Check number of object the robot already went to
    n_object_lt_1 = py_trees.behaviours.CheckBlackboardVariableValue(
        name="n_object_lt_1",
        check=py_trees.common.ComparisonExpression(
            variable = "n_object",
            value = 1,
            operator=operator.lt
        )
    )

    # Create Behaviors
    scan_object = ScanObject(name="scan_object")

    approach_base_to_object = ApproachBaseObject(name="approach_base_object")

    approach_manipulator_to_object = ApproachManipulatorObject(name="approach_manipulator_object")

    pick_object = PickObject(name="pick_object")

    handle_manipulator_object= HandleManipulatorObject(name="handle_manipulator_object")

    approach_base_to_place = ApproachBasePlace(name="approach_base_place")

    handle_manipulator_to_place_object= HandleManipulatorPlace(name="handle_manipulator_place_object")

    approach_manipulator_to_place_object = ApproachManipulatorPlaceObject(name="approach_manipulator_place_object")

    let_object = LetObject(name="let_object")

    finish_object = HandleManipulatorObject(name="finish_object")

    root = py_trees.composites.Sequence(name="Life", memory=True)    
    root.add_children([
                       scan_object,
                       approach_base_to_object, 
                       approach_manipulator_to_object, 
                       pick_object, 
                       handle_manipulator_object,
                       approach_base_to_place, 
                       handle_manipulator_to_place_object,
                       approach_manipulator_to_place_object, 
                       let_object,
                       finish_object])
    
    return root

def run(it=200):
    root = create_tree()

    try:
        print("Call setup for all tree children")
        root.setup_with_descendants() 
        print("Setup done!\n\n")
        py_trees.display.ascii_tree(root)
        
        for _ in range(it):
            root.tick_once()
            time.sleep(1)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
   
    rospy.init_node('behavior_trees')

    # Create behavior tree
    root = create_tree()
    # py_trees.display.render_dot_tree(root)
    run()