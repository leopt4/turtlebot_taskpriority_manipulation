#!/usr/bin/python3
import rospy
from geometry_msgs.msg import PoseStamped
import time
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool
import actionlib


import py_trees
import py_trees.decorators
import py_trees.display
from py_trees.blackboard import Blackboard

from utils.task import *
from turtlebot_taskpriority_manipulation.msg import TaskMsg
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
            "detect_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "detect_goal", access=py_trees.common.Access.READ)
        self.blackboard.register_key(
            "n_object", access=py_trees.common.Access.WRITE)
        
        self.blackboard.detect_goal = False

    def setup(self):
        self.logger.debug("  %s [ScanObject::setup()]" % self.name)
        self.tasks = [
            MMOrientation("Mobile Base orientation", np.array([0.0]).reshape(1,1), np.zeros((1,1)), np.zeros((1,1)))
        ] 

        self.err = np.inf

        self.blackboard.n_object = 0

        self.logger.debug("  %s [ScanObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ScanObject::initialise()]" % self.name)

        self.desired = SCAN_INIT_HEADING 

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=1)

        # SUBSCRIBERS
        # Subcribe to get aruco pose
        self.aruco_pose_object_sub      = rospy.Subscriber(aruco_pose_topic, PoseStamped, self.arucoPoseCB)
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        if MODE == "SIL":
            self.odom_sub = rospy.Subscriber(odom_SIL_topic, Odometry, self.odomCallback) 
        elif MODE == "HIL":
            self.odom_sub = rospy.Subscriber(odom_HIL_topic, Odometry, self.odomCallback) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)

    def update(self):
        success = False
        if self.blackboard.detect_goal == True:
            relX = self.blackboard.goal[0] - self.eta[0]
            relY = self.blackboard.goal[1] - self.eta[1]
            relAngle = self.eta[3] - np.arctan2(relY, relX)

            if relAngle > SCAN_HEADING_RANGE_LOWER and relAngle < SCAN_HEADING_RANGE_UPER:
                self.desired = self.eta[3]
                success = True

        elif abs(self.err) < SCAN_HEADING_ERROR:
            self.desired += SCAN_HEADING_STEP

        task_msg = TaskMsg()
        task_msg.ids = "1"
        task_msg.name = self.tasks[0].name
        task_msg.desired = [self.desired]
        task_msg.gain = [BASE_ORI_GAIN]
        task_msg.feedForward = [BASE_ORI_FEEDFORWARD]
        self.task_publisher.publish(task_msg)

        if success == True and self.blackboard.n_object < N_OBJECT:
            time.sleep(1.0)
            self.logger.debug("  %s [ScanObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ScanObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ScanObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def odomCallback(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                    odom.pose.pose.orientation.y,
                                                    odom.pose.pose.orientation.z,
                                                    odom.pose.pose.orientation.w])
        self.eta = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z, yaw]).reshape(-1,1)

    def get_err(self, err):
        if len(err.data) == 1:
            self.err = np.array([err.data[0]])

    # Aruco pose detector callback
    def arucoPoseCB(self, arucoPose):
        obj_pos_x = float(arucoPose.pose.position.x)
        obj_pos_y = float(arucoPose.pose.position.y)
        obj_pos_z = float(arucoPose.pose.position.z)

        self.blackboard.goal = [obj_pos_x, obj_pos_y]# [1.18, 0.02]
        self.blackboard.detect_goal = True

class ApproachBaseObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachBaseObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [ApproachBaseObject::setup()]" % self.name)

        self.tasks = [ 
            MMConfiguration("Mobile base configuration", np.array([0, 0, 0]).reshape(3,1), np.zeros((3,1)), np.zeros((3,3)))
        ] 

        self.err = np.array([np.inf, np.inf])
        self.logger.debug("  %s [ApproachBaseObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBaseObject::initialise()]" % self.name)  
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(1.0)
           
    def update(self):
        
        task_msg = TaskMsg()
        task_msg.ids = "2"
        task_msg.name = self.tasks[0].name
        task_msg.desired = [self.blackboard.goal[0], self.blackboard.goal[1], 0.0]
        task_msg.gain = [BASE_CONFIG_GAIN_X, BASE_CONFIG_GAIN_Y, BASE_CONFIG_GAIN_HEADING]
        task_msg.feedForward = [BASE_CONFIG_FEEDFORWARD_X, BASE_CONFIG_FEEDFORWARD_Y, BASE_CONFIG_FEEDFORWARD_HEADING]
        self.task_publisher.publish(task_msg)

        if  abs(self.err[0]) < BASE_CONFIG_DIS_ERROR_FINISH and abs(self.err[1]) < BASE_CONFIG_HEADING_ERROR_FINISH:
            self.logger.debug("  %s [ApproachBaseObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBaseObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBaseObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 2:
            self.err = np.array([err.data[0], err.data[1]])    

class ApproachManipulatorObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachManipulatorObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)

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
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

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
        task_msg.feedForward = [EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]
        self.task_publisher.publish(task_msg)

        if np.linalg.norm(self.err) < EE_POS_ERROR_FINISH and self.desired_pos_z < -0.20:
            self.desired_pos_z = MANI_PICK_HEIGHT

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
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [HandleManipulatorObject::setup()]" % self.name)
        self.tasks = [
            Limit2D("Manipulator Joint 1 Limitation", limit_range_joint1, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 1),
            Limit2D("Manipulator Joint 2 Limitation", limit_range_joint2, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 2),
            Limit2D("Manipulator Joint 3 Limitation", limit_range_joint3, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 3),
            Limit2D("Manipulator Joint 4 Limitation", limit_range_joint4, threshold_joint, np.zeros((1,1)), np.zeros((1,1)), 4), 
            EEPosition3D("End-effector position", np.array([0.0, 0.0, 0.0]).reshape(3,1), np.zeros((3,1)), np.zeros((3,3)))
        ] 

        self.logger.debug("  %s [HandleManipulatorObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [HandleManipulatorObject::initialise()]" % self.name)     
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

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
        task_msg.feedForward = [EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]
        self.task_publisher.publish(task_msg)

        if np.linalg.norm(self.err) < EE_POS_ERROR_PICK_OBJ:
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
        self.blackboard.n_object = 1
    def setup(self):
        self.logger.debug("  %s [LetObject::setup()]" % self.name)
    
    def initialise(self):
        self.logger.debug("  %s [LetObject::initialise()]" % self.name)

    def update(self):
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

    def setup(self):
        self.logger.debug("  %s [ApproachBasePlace::setup()]" % self.name)

        self.tasks = [ 
            MMConfiguration("Mobile base configuration", np.array([0, 0, 0]).reshape(3,1), np.zeros((3,1)), np.zeros((3,3)))
        ] 
        self.err = np.array([np.inf, np.inf])
        self.logger.debug("  %s [ApproachBasePlace::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBasePlace::initialise()]" % self.name)  
        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher(task_topic, TaskMsg, queue_size=10)

        # SUBSCRIBERS
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber(task_error_topic, Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)
           
    def update(self):

        task_msg = TaskMsg()
        task_msg.ids = "2"
        task_msg.name = self.tasks[0].name
        task_msg.desired        = [3.0, 1.0]
        task_msg.gain           = [0.1, 0.1, 0.4]
        task_msg.feedForward    = [0.0, 0.0, 0.0]
        self.task_publisher.publish(task_msg) 

        if  abs(self.err[0]) < BASE_CONFIG_DIS_ERROR_FINISH and abs(self.err[1]) < BASE_CONFIG_HEADING_ERROR_FINISH:
            self.logger.debug("  %s [ApproachBaseObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBaseObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBaseObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    def get_err(self, err):
        if len(err.data) == 2:
            self.err = np.array([err.data[0], err.data[1]])

class ApproachManipulatorPlaceObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachManipulatorPlaceObject, self).__init__(name)

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
        task_msg.desired        = [GOAL_PLACE_X, GOAL_PLACE_Y, MANI_PICK_HEIGHT]
        task_msg.gain           = [EE_POS_GAIN_X, EE_POS_GAIN_Y, EE_POS_GAIN_Z]
        task_msg.feedForward    = [EE_POS_FEEDFORWARD_X, EE_POS_FEEDFORWARD_Y, EE_POS_FEEDFORWARD_Z]
        self.task_publisher.publish(task_msg)

        if abs(self.err[2]) < 0.03:
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

    approach_base_to_place = ApproachBasePlace(name="approach_base_place")

    handle_manipulator_object= HandleManipulatorObject(name="handle_manipulator_object")

    approach_manipulator_to_place_object = ApproachManipulatorPlaceObject(name="approach_manipulator_place_object")

    let_object = LetObject(name="let_object")

    root = py_trees.composites.Sequence(name="Life", memory=True)    
    root.add_children([n_object_lt_1,
                       scan_object,
                       approach_base_to_object, 
                       approach_manipulator_to_object, 
                       pick_object, 
                       handle_manipulator_object, 
                       approach_base_to_place, 
                       approach_manipulator_to_place_object, 
                       let_object])
    # py_trees.display.render_dot_tree(root)
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
    run()