#!/usr/bin/python
import rospy
import tf2_ros
import numpy as np
from math import cos, sin
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TransformStamped
from visualization_msgs.msg import Marker
from tf.transformations import quaternion_from_euler


class SwiftproController:
    def __init__(self) -> None:
        # Joint state subscriber
        self.joint_state_sub = rospy.Subscriber("/swiftpro/joint_states", JointState, self.joint_state_callback)
        
        # Passive joint position publisher (ROS control command)
        self.joint_controller_pub = rospy.Publisher("/turtlebot/joint_states_controller", JointState, queue_size=10)
        
        # Marker publisher
        self.marker_pub = rospy.Publisher("/turtlebot/ee_pose", Marker, queue_size=10)
        
        # TF publisher
        self.publish_tf = True 
        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster() #this line makes sure that the tf is published
            # self.ee_tf_static_publisher()
        
        # robot kinematic parameters
        self.dof = 4
        self.q = np.zeros(self.dof).reshape(-1,1)
        self.update_kinematics()
        
        # robot arm dimensions
        self.bx = 13.2
        self.bz = 108.0  #74.7 + 33.33
        self.d1 = 142.0
        self.d2 = 158.8  
        self.mz = 72.2
        self.mx = 56.5
        
    def joint_state_callback(self, msg):
        if msg.name[0] == "swiftpro/joint1": 
            q1 = msg.position[0]
            q2 = msg.position[1]
            q3 = msg.position[2]
            q4 = msg.position[3]
        
            print(q1)
            print(q2)
            print(q3)
            print(q4)
            
            self.update_kinematics()
    
    def update_kinematics(self):
        
        #update robot kinematics according to q
        q1, q2, q3, q4 = self.q
        
        #forward kinematics to get ee position     
        self.x = float((self.bx + self.d1*sin(-q2) + self.d2*cos(q3) + self.mx)*cos(q1) /1000)
        self.y = float((self.bx + self.d1*sin(-q2) + self.d2*cos(q3) + self.mx)*sin(q1) /1000)
        self.z = float(-(self.bz + self.d1*cos(-q2) + self.d2*sin(q3) - self.mz) /1000)
        self.yaw = float(q1 + q4)
        print(self.x)
        print(self.y)
        print(self.z)
        print(self.yaw) 
        
        #transformation matrix
        self.T = np.array([[np.cos(self.yaw), -np.sin(self.yaw), 0, self.x],
                            [np.sin(self.yaw), np.cos(self.yaw), 0, self.y],
                            [0, 0, 1, self.z],
                            [1, 0, 0, 1]])
        
        #ee updated pose publisher
        if self.publish_tf:
        
            transform = TransformStamped()
            transform.header.stamp = rospy.Time.now()
            transform.header.frame_id = "swiftpro/manipulator_base"
            transform.child_frame_id = "swiftpro/manipulator_ee"
            
            #translation
            transform.transform.translation.x = self.x
            transform.transform.translation.y = self.y
            transform.transform.translation.z = self.z            
            
            #rotation (yaw to quaternion)
            quaternion = quaternion_from_euler(0, 0, self.yaw)
            transform.transform.rotation.x = quaternion[0]
            transform.transform.rotation.y = quaternion[1]
            transform.transform.rotation.z = quaternion[2]
            transform.transform.rotation.w = quaternion[3]
            self.tf_broadcaster.sendTransform(transform)
            
    
    def getEEJacobian(self):
        J = np.zeros((6, self.dof))
        
        q1, q2, q3, q4 = self.q
        
        d1p = -self.d1*np.sin(q2) #projection of d1 on x-axis 
        d2p = self.d2*np.cos(q3) #projection of d2 on x-axis
        l = self.bx + self.mx + d1p + d2p #total length from base to ee top projection 
        
        J[:,0] = np.array([l*np.sin(q1),                    l*np.cos(q1),                   0,                0, 0, 1]) #q1 derivatives
        J[:,1] = np.array([-self.d1*np.cos(q2)*np.cos(q1), -self.d1*np.cos(q2)*np.sin(q1), self.d1*np.sin(q2, 0, 0, 0)]) #q2 derivatives
        J[:,2] = np.array([-self.d2*np.sin(q3)*np.cos(q1), -self.d2*np.sin(q3)*np.sin(q1), -self.d2*np.cos(q3, 0, 0, 0)]) #q3 derivatives
        J[:,3] = np.array([0, 0, 0, 0, 0, 1]) #q4 derivatives
        
        return J
        
    
            
    # def ee_tf_static_publisher(self):
        
    #     """
    #     Publish a static TF from the base_link 
    #     """
        
    #     static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
    #     static_transformStamped = TransformStamped()
        
    #     static_transformStamped.header.stamp = rospy.Time.now()
    #     static_transformStamped.header.frame_id = "/turtlebot/swiftpro/manipulator_base_link"
    #     static_transformStamped.child_frame_id = "swiftpro_base"
        

    def ee_publisher(self, x, y, z, yaw):
        mark = Marker()
        mark.header.frame_id = "swiftpro/manipulator_base_link"
        mark.header.stamp = rospy.Time.now()
        mark.pose.position.x = x
        mark.pose.position.y = y
        mark.pose.position.z = z
        mark.pose.orientation = "yaw"
        mark.scale.x = 0.01
        mark.scale.y = 0.01
        mark.scale.z = 0.01
        mark.color.a = 1.0
        mark.color.r = 0.0
        mark.color.g = 1.0
        mark.color.b = 0.0
    
        self.marker_pub.publish(mark)
        
        
          

if __name__ == '__main__':
    rospy.init_node("test_controller")
    robot = SwiftproController()
    rospy.spin()