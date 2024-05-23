#!/usr/bin/python3
import py_trees
import rospy
from geometry_msgs.msg import PoseStamped
import time
from tf.transformations import quaternion_from_euler
from std_srvs.srv import SetBool
import actionlib


from utils.task import *
from turtlebot_taskpriority_manipulation.msg import TaskMsg
from std_msgs.msg import Float64MultiArray


class setGoal (py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super().__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("position", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("position", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [setGoal::setup()]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [setGoal::initialise()]" % self.name)        

    def update(self):
        goal = PoseStamped()
        goal.pose.position.x = 0.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0

        q = quaternion_from_euler(0,0,1.57)
        goal.pose.orientation.x = q[0]
        goal.pose.orientation.y = q[1]
        goal.pose.orientation.z = q[2]
        goal.pose.orientation.w = q[3]
        self.blackboard.goal = goal

        pickup_goal = PoseStamped()
        pickup_goal.pose.position.x = 0.0
        pickup_goal.pose.position.y = 0.0
        pickup_goal.pose.position.z = 0.0

        q = quaternion_from_euler(0,0,1.57)
        pickup_goal.pose.orientation.x = q[0]
        pickup_goal.pose.orientation.y = q[1]
        pickup_goal.pose.orientation.z = q[2]
        pickup_goal.pose.orientation.w = q[3]
        self.blackboard.pickup_goal = pickup_goal
       

        return py_trees.common.Status.SUCCESS
class moveTurtlebot (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name): 
        super(moveTurtlebot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)
        
    def setup(self):
        self.logger.debug("  %s [moveTurtlebot::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_turtlebot', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveTurtlebot::initialise()]" % self.name)

    def update(self):
        print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            
class moveSwiftpro (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(moveSwiftpro, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [moveSwiftpro::setup()]" % self.name)
        self.goal_sent = False
        self.client = actionlib.SimpleActionClient('move_swiftpro', MoveToGoalAction)
        rospy.loginfo("Waiting for move turtlebot action...")
        self.client.wait_for_server()


    def initialise(self):
        self.logger.debug("  %s [moveSwiftpro::initialise()]" % self.name)

    def update(self):
        print(self.client.get_state())
        if self.client.get_state() == actionlib.GoalStatus.PENDING or self.client.get_state() == actionlib.GoalStatus.LOST:
            rospy.loginfo('Send goal to swiftpro')
            goal = MoveToGoalActionGoal()
            goal.goal = self.blackboard.pickup_goal
            self.client.send_goal(goal)
            self.goal_sent = True
        if self.client.get_state() == actionlib.GoalStatus.PREEMPTED :
            rospy.logerr('Move swiftpro Fail')
            self.goal_sent = False
            return py_trees.common.Status.FAILURE
        
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED :
            rospy.loginfo('Move swiftpro Success')
            self.goal_sent = False
            return py_trees.common.Status.SUCCESS
    
        else:
            rospy.loginfo('Move swiftpro working...')
            return py_trees.common.Status.RUNNING
        
class EnableSuction (py_trees.behaviour.Behaviour):
    # this one only moves the base of the robot. no arm behavior here.
    def __init__(self, name):
        super(EnableSuction, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)

        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("pickup_goal", access=py_trees.common.Access.READ)

    def setup(self):
        self.logger.debug("  %s [EnableSuction::setup()]" % self.name)
        

    def initialise(self):
        self.logger.debug("  %s [EnableSuction::initialise()]" % self.name)

    def update(self):
        succes = self.enable_suction()
        if succes:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE
        
    def enable_suction(self):
        rospy.logwarn("Calling enable suction")
        rospy.wait_for_service('/turtlebot/swiftpro/vacuum_gripper/set_pump')
        path = []
        try:
            enable_suction = rospy.ServiceProxy('/turtlebot/swiftpro/vacuum_gripper/set_pump', SetBool)
            resp = enable_suction(True)
            
            return resp.success
        
        except rospy.ServiceException as e:
            print(f"Service call failed: {e}")
            return False

class ApproachBaseObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachBaseObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_points", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None

    def setup(self):
        self.logger.debug("  %s [ApproachBaseObject::setup()]" % self.name)
        self.tasks = [ 
            MMConfiguration("Mobile base configuration", np.array([0, 0, 0]).reshape(3,1), np.zeros((3,1)), gain)
        ] 

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.task_publisher = rospy.Publisher('/task', TaskMsg, queue_size=10)

        # SUBSCRIBERS
        # Subcribe to get aruco pose
        self.aruco_pose_object_sub      = rospy.Subscriber("/aruco_pose", PoseStamped, self.arucoPoseCB)
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber('/error_topic', Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)
        self.logger.debug("  %s [ApproachBaseObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBaseObject::initialise()]" % self.name)     

    def update(self):

        if self.err < 0.35:
            self.logger.debug("  %s [ApproachBaseObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBaseObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBaseObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_err(self, err):
        self.err = np.array(err)

    # Aruco pose detector callback
    def arucoPoseCB(self, arucoPose):
        obj_pos_x = float(arucoPose.pose.position.x)
        obj_pos_y = float(arucoPose.pose.position.y)
        obj_pos_z = float(arucoPose.pose.position.z)

        self.tasks[0].setDesired(np.array([obj_pos_x, obj_pos_y, 0.0]).reshape(3,1))

        task_msg = TaskMsg
        task_msg.name = self.tasks[0].name
        task_msg.desired = [self.tasks[0].getDesired()[0], self.tasks[0].getDesired()[1], self.tasks[0].getDesired()[2]]
        task_msg.gain = [0.1, 0.1, 0.2]
        task_msg.feedForward = [0.0, 0.0, 0.0]
        self.task_publisher.publish(task_msg)

class ApproachManipulatorObject(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(ApproachBaseObject, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key(
            "n_points", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key(
            "goal", access=py_trees.common.Access.READ)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None

    def setup(self):
        self.logger.debug("  %s [ApproachBaseObject::setup()]" % self.name)
        self.tasks = [ 
            MMConfiguration("Mobile base configuration", np.array([0, 0, 0]).reshape(3,1), np.zeros((3,1)), gain)
        ] 

        # PUBLISHERS
        # Publisher for sending task to the TP control node
        self.goal_publisher = rospy.Publisher('/task', TaskMsg, queue_size=10)

        # SUBSCRIBERS
        # Subcribe to get aruco pose
        self.aruco_pose_object_sub      = rospy.Subscriber("/aruco_pose", PoseStamped, self.arucoPoseCB)
        #subscriber to task error 
        self.task_err_sub = rospy.Subscriber('/error_topic', Float64MultiArray, self.get_err) 

        # Wait 0.2s to init pub and sub
        time.sleep(0.2)
        self.logger.debug("  %s [ApproachBaseObject::setup() SUCCESS]" % self.name)

    def initialise(self):
        self.logger.debug("  %s [ApproachBaseObject::initialise()]" % self.name)     

    def update(self):

        if self.err < 0.35:
            self.logger.debug("  %s [ApproachBaseObject::Update() SUCCESS]" % self.name)
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug("  %s [ApproachBaseObject::Update() RUNNING]" % self.name)
            return py_trees.common.Status.RUNNING


    def terminate(self, new_status):
        self.logger.debug("  %s [ApproachBaseObject::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
        
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_err(self, err):
        self.err = np.array(err)

    # Aruco pose detector callback
    def arucoPoseCB(self, arucoPose):
        obj_pos_x = float(arucoPose.pose.position.x)
        obj_pos_y = float(arucoPose.pose.position.y)
        obj_pos_z = float(arucoPose.pose.position.z)

        self.tasks[0].setDesired(np.array([obj_pos_x, obj_pos_y, 0.0]).reshape(3,1))


#   Create Behavior trees function
def create_tree():
    # Create Behaviors
    scan_object = ScanObject(name="scan_object")

    approach_base_to_object = ApproachBaseObject(name="approach_base_object")

    approach_manipulator_to_object = ApproachManipulatorObject(name="approach_manipulator_object")

    pick_object = PickObject(name="pick_object")

    approach_base_to_place = ApproachBasePlace(name="approach_base_place")

    let_object = LetObject(name="let_object")


    # # Special py_trees behavior
    # # Check number of points in the list the robot already went to
    # n_points_lt_5 = py_trees.behaviours.CheckBlackboardVariableValue(
    #     name="n_points_lt_5",
    #     check=py_trees.common.ComparisonExpression(
    #         variable="n_points",
    #         value=len(points_list),
    #         operator=operator.lt
    #     )
    # )

    # # Check number of objects the robot already collected
    # n_collected_objects_lt_2 = py_trees.behaviours.CheckBlackboardVariableValue(
    #     name="n_collected_objects_lt_2",
    #     check=py_trees.common.ComparisonExpression(
    #         variable="n_collected_objects",
    #         value=2,
    #         operator=operator.lt
    #     )
    # )
    # # Behavior checking end condition of the behavior tree
    # check_end = py_trees.composites.Sequence(name="check_end", memory=True)
    # check_end.add_children([n_collected_objects_lt_2, n_points_lt_5])

    root = py_trees.composites.Sequence(name="Life", memory=True)    
    root.add_children([scan_object, approach_base_to_object, approach_manipulator_to_object, pick_object, approach_base_to_place, let_object])
    # py_trees.display.render_dot_tree(root)
    return root




if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("intervention_behavior_trees")

    # Create Behaviors
    set_goal = setGoal("set_goal")
    move_turtlebot = moveTurtlebot("move_turtlebot")
    move_turtlebot_up = moveTurtlebot("move_turtlebot_up")

    move_swiftpro = moveSwiftpro("move_swiftpro")
    enable_suction = EnableSuction("enable_suction")


    # go to pickup spot sequence
    go_to_seq = py_trees.composites.Sequence(name="go_to_seq", memory=True)
    go_to_seq.add_children([set_goal,move_turtlebot,move_swiftpro,enable_suction,move_turtlebot_up])
    # retry_pickup = py_trees.decorators.Retry(name="retry_pickup",child=go_to_seq, num_failures=6)

    # pick_place_seq = py_trees.composites.Sequence(name="Pick and Place Object", memory=True)
    # pick_place_seq.add_children([is_finished,retry_pickup,get_object,find_drop_spot,move_to_drop,let_object])
    # repeat_pick_place = py_trees.decorators.Repeat(name="repeat_pick_place",child=pick_place_seq,num_success=100)

    print("Call setup for all tree children")
    go_to_seq.setup_with_descendants() # call setup() of all behaviors in the tree (set up ROS topic, service ...)
    print("Setup done!\n\n")
    time.sleep(1)    
    while not rospy.is_shutdown():
        go_to_seq.tick_once() # untill the root return sth.
        time.sleep(1)
### create behaviors 
#set goal 
#move offset = move robot 
#move up =  move robot 
#move down = move robot 

#
# move turtlebot 