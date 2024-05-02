import numpy as np
import rospy

wheelBase   = 0.235
wheelRadius = 0.035

class Encoder:
    def __init__(self, tag:str):
        self.tag       = tag        # Name tag pf encoder [left/right]
        self.position   = 0.0       # Position          [met]
        self.velocity   = 0.0       # Angular velocity  [rad/sec]
        self.stamp      = None      # Time stamp        [ros time]

class SwiftProJoint:
    def __init__(self, num_joint:int):
        self.num_joint  = num_joint                                   # Number of joints
        self.position   = [0.0 for _ in range(self.num_joint)]        # Position          [met]
        self.velocity   = [0.0 for _ in range(self.num_joint)]        # Angular velocity
        self.stamp      = None                                        # Time stamp        [ros time]

class TurtlebotJointState:
    def __init__(self) -> None:
        """
        Constructor of the jointStateData class.

        :param:
        """
        self.newData    = False     # Flag presenting got new synchronized data

        # Init left and right encoders
        self.rightEncoder   = Encoder('turtlebot/kobuki/wheel_right_joint')
        self.leftEncoder    = Encoder('turtlebot/kobuki/wheel_left_joint')
        self.SwiftProJoint  = SwiftProJoint(4)

        self.synchronized_velocity  = [0.0, 0.0]    # Synchronized angular velocity including the left and right encoders [rad/sec]
        self.synchronized_stamp     = None          # Synchronized time stamp [ros time]
        self.deltaT                 = None          # The length of period between this sync data and the last sync data
        self.displacement           = [0.0, 0.0]    # Displacement between this sync data and the last sync data

        self.Qk         = np.diag(np.array([0.01 ** 2, 0.001 ** 2, np.deg2rad(0.1) ** 2]))  # covariance of displacement noise

    def update_encoder_reading(self, jointState):
        """
        Parse encoder measurements
        :param: jointState
        :return True: 
        """
        # Check if encoder data is for the left wheel
        if self.leftEncoder.tag in jointState.name:
            self.leftEncoder.velocity   = jointState.velocity[0]
            self.leftEncoder.stamp      = rospy.Time.now() 
            return True
        # Check if encoder data is for the right wheel
        elif self.rightEncoder.tag in jointState.name:
            self.rightEncoder.velocity  = jointState.velocity[0]
            self.rightEncoder.stamp     = rospy.Time.now() 
            return True
        
        return False

    def synchronize_encoder_reading(self):
        """
        Synchronize data between the left and right encoders. Because we can not get encoder measurement of both encoders at the same time
        :param:
        :return True: got synchronized measurement
        :retuen False: not get
        """
        # Synchronize encoder data if readings for both wheels are available
        if self.leftEncoder.stamp is not None and self.rightEncoder.stamp is not None:
            next_synchronized_stamp     = 0.5 * ((self.leftEncoder.stamp.secs + self.rightEncoder.stamp.secs) + (self.leftEncoder.stamp.nsecs + self.rightEncoder.stamp.nsecs)/1e9)  
            # Compute period
            if self.synchronized_stamp is not None:
                self.deltaT                 = next_synchronized_stamp - self.synchronized_stamp

            self.synchronized_stamp     = next_synchronized_stamp
            # Synchronize encoder readings here
            # For demonstration, let's assume the readings are already synchronized
            self.synchronized_velocity  = [self.leftEncoder.velocity, self.rightEncoder.velocity]
            # Publish synchronized data or use it in your control algorithm

            # Reset readings for next iteration
            self.leftEncoder.stamp      = None
            self.rightEncoder.stamp     = None
            
            # Get synchronized encoder data
            if self.deltaT is not None:
                return True
        # Not get synchronized encoder data
        return False
    
    def compute_displacement(self):
        """
        Compute displacement between this sync data and the last sync data

        :param: 
        :return uk: displacement
        """
        # Compute displacements of the left and right wheels
        d_L = self.synchronized_velocity[0] * wheelRadius * self.deltaT
        d_R = self.synchronized_velocity[1] * wheelRadius * self.deltaT
        # Compute displacement of the center point of robot between k-1 and k
        d       = (d_L + d_R) / 2.
        # Compute rotated angle of robot around the center point between k-1 and k
        delta_theta_k   = (-d_R + d_L) / wheelBase

        # Compute xk from xk_1 and the travel distance and rotated angle. Got the equations from chapter 1.4.1: jointStateetry 
        uk              = np.array([[d],
                                    [0],
                                    [delta_theta_k]])
        
        return uk
    
    def read_encoder(self, jointState):
        """
        Read encoder method includes updating encoder reading, synchronizing them and computing displacement

        :param jointState: jointState mess 
        :return True: if get displacement
        :return False: not enough encoder reading to compute displacement
        """
        self.update_encoder_reading(jointState)

        if self.synchronize_encoder_reading():
            self.displacement = self.compute_displacement()
            return True
        
        return False

    def get_displacement(self):
        """
        Get displacement

        :return displacement, Qk: mean displacement vector and its covariance matrix.
        """
        return self.displacement, self.Qk
    
    def read_swiftpro_joint(self, jointState):
        """
        Parse encoder measurements
        :param: jointState
        :return True: 
        """
        # Check if encoder data is for the left wheel
        if len(jointState.name) == 4:
            self.SwiftProJoint.position[0] = jointState.position[0]
            self.SwiftProJoint.position[1] = jointState.position[1]
            self.SwiftProJoint.position[2] = jointState.position[2] 
            self.SwiftProJoint.position[3] = jointState.position[3] 

            self.SwiftProJoint.velocity[0] = jointState.velocity[0]
            self.SwiftProJoint.velocity[1] = jointState.velocity[1] 
            self.SwiftProJoint.velocity[2] = jointState.velocity[2] 
            self.SwiftProJoint.velocity[3] = jointState.velocity[3]

            self.SwiftProJoint.stamp     = rospy.Time.now() 

            return True
        
        return False

    def update(self, jointState):
        self.read_encoder(jointState)
        self.read_swiftpro_joint(jointState)
        return True