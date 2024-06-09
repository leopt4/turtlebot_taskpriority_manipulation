from .common import *
import math
import numpy as np
from config import *

class ManipulatorParams:
    def __init__(self) -> None:
        self.revolute = [True, True, True, True]
        self.dof = len(self.revolute)     
        self.bx = MANI_BX       
        self.bz = MANI_BZ         
        self.d1 = MANI_D1        
        self.d2 = MANI_D2       
        self.mz = MANI_MZ     
        self.mx = MANI_MX        
        
        if MODE == "SIL":
            self.alpha = MANI_ALPHA_SIL
        elif MODE == "HIL":
            self.alpha = MANI_ALPHA_HIL

class MobileBaseParams:
    def __init__(self) -> None:
        self.bmx = 0.0507       # [met]
        self.bmz = -0.198       # [met]

        self.theta    = np.array([   deg90,        0])
        self.d        = np.array([self.bmz, self.bmx])
        self.a        = np.array([       0,        0])
        self.alpha    = np.array([   deg90,   -deg90])

        self.revolute   = [True, False]
        self.dof        = len(self.revolute)
class MobileManipulator:
    '''
        Constructor.

        Arguments:
        d (Numpy array): list of displacements along Z-axis
        theta (Numpy array): list of rotations around Z-axis
        a (Numpy array): list of displacements along X-axis
        alpha (Numpy array): list of rotations around X-axis
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    '''
    def __init__(self):

        self.manipulatorParams = ManipulatorParams()

        # List of joint types with base joints
        self.baseParams = MobileBaseParams()

        self.dof            = self.manipulatorParams.dof + self.baseParams.dof

        # Vector of joint positions (manipulator)
        self.q              = np.zeros((self.manipulatorParams.dof, 1))
        self.q_hist         = []

        # Vector of base pose (position & orientation)
        self.eta            = np.zeros((4, 1))


    '''
        Method that returns the end-effector transformation.
    '''
    def getEETransform(self):
        return self.T[-1]

    '''
        Method that returns the position of a selected joint.

        Argument:
        joint (integer): index of the joint

        Returns:
        (double): position of the joint
    '''
    def getJointPos(self, joint):
        return self.q[joint-1]


    def getBasePose(self):
        return self.eta

    '''
        Method that returns number of DOF of the mobile base manipulator.
    '''
    def getDOF(self):
        return self.dof
    
    '''
        Method that returns number of DOF of the mobile base.
    '''
    def getMMDOF(self):
        return self.mm_dof

    
    def track_q(self):
        # self.q_hist.append(((np.rad2deg(self.q))% 360 + 360) % 360)
        # self.q_hist.append((np.rad2deg(self.theta)))
        self.q_hist.append(self.q)
    
    def reset(self):
        # Reset the robot's state to the initial conditions
        self.q = np.zeros((len(self.revolute), 1))
        self.eta = np.zeros((4, 1))
        self.update(np.zeros((self.dof, 1)), 0.0)

    '''
        Method that returns the end-effector Jacobian.
    '''
    def getEEJacobian(self): 
        J = np.zeros((6, self.dof))
        q1, q2, q3, q4  = self.q
    
        l1p     = -self.manipulatorParams.d1 * math.sin(q2)               #projection of d1 on x-axis 
        l2p     =  self.manipulatorParams.d2 * math.cos(q3)               #projection of d2 on x-axis
        l       =  self.manipulatorParams.bx + l1p + l2p + self.manipulatorParams.mx    #total length from base to ee top projection 

        dx_dq1  = -l * math.sin(q1+self.eta[3]+self.manipulatorParams.alpha)
        dy_dq1  =  l * math.cos(q1+self.eta[3]+self.manipulatorParams.alpha)
        
        
        dx_dq2  = -self.manipulatorParams.d1 * math.cos(q2) * math.cos(q1+self.eta[3]+self.manipulatorParams.alpha)
        dy_dq2  = -self.manipulatorParams.d1 * math.cos(q2) * math.sin(q1+self.eta[3]+self.manipulatorParams.alpha)
        dz_dq2  =  self.manipulatorParams.d1 * math.sin(q2)


        dx_dq3  = -self.manipulatorParams.d2 * math.sin(q3) * math.cos(q1+self.eta[3]+self.manipulatorParams.alpha)
        dy_dq3  = -self.manipulatorParams.d2 * math.sin(q3) * math.sin(q1+self.eta[3]+self.manipulatorParams.alpha)
        dz_dq3  = -self.manipulatorParams.d2 * math.cos(q3)

        # Base kinematics
        x = float(self.eta[0])
        y = float(self.eta[1])
        yaw = float(self.eta[3])
        Tb = translation2D(x, y) @ rotation2D(yaw)

        # Modify the theta of the base joint, to account for an additional Z rotation
        theta = float(q1-deg90)

        # Combined system kinematics (DH parameters extended with base DOF)
        thetaExt    = np.concatenate([np.array([deg90,                                 0]), np.array([theta])])
        dExt        = np.concatenate([np.array([self.baseParams.bmz, self.baseParams.bmx]),    np.array([0])])
        aExt        = np.concatenate([np.array([0,                                     0]),    np.array([l])])
        alphaExt    = np.concatenate([np.array([deg90,                            -deg90]),    np.array([0])])

        self.T      = kinematics(dExt, thetaExt, aExt, alphaExt, Tb)

        T  = kinematics(dExt, thetaExt, aExt, alphaExt, Tb)
        JB = jacobian(T, self.baseParams.revolute + [True])

        J[:, 0] = JB[:,0].reshape((1,6))                        # derivertive by m1
        J[:, 1] = JB[:,1].reshape((1,6))                        # derivertive by m2
        J[:, 2] = np.array([dx_dq1, dy_dq1,      0, 0, 0, 1])   # derivertive by q1
        J[:, 3] = np.array([dx_dq2, dy_dq2, dz_dq2, 0, 0, 0])   # derivertive by q2
        J[:, 4] = np.array([dx_dq3, dy_dq3, dz_dq3, 0, 0, 0])   # derivertive by q3
        J[:, 5] = np.array([     0,      0,      0, 0, 0, 1])   # derivertive by q4

        return J
    
    '''
        Method that returns the moblie base Jacobian.
    '''
    def getMMJacobian(self): 
        J = np.zeros((6, self.dof))

        # Base kinematics
        x = float(self.eta[0])
        y = float(self.eta[1])
        yaw = float(self.eta[3])
        Tb = translation2D(x, y) @ rotation2D(yaw)

        T  = kinematics(self.baseParams.d, self.baseParams.theta, self.baseParams.a, self.baseParams.alpha, Tb)
        JB = jacobian(T, self.baseParams.revolute)

        J[:, 0] = JB[:,0].reshape((1,6))                        # derivertive by m1
        J[:, 1] = JB[:,1].reshape((1,6))                        # derivertive by m2
        J[:, 2] = np.array([     0,      0,      0, 0, 0, 0])   # derivertive by q4
        J[:, 3] = np.array([     0,      0,      0, 0, 0, 0])   # derivertive by q4
        J[:, 4] = np.array([     0,      0,      0, 0, 0, 0])   # derivertive by q4
        J[:, 5] = np.array([     0,      0,      0, 0, 0, 0])   # derivertive by q4

        return J
    
    '''
        Method that returns the End Effector position
    '''
    def getEEposition(self):
        q1, q2, q3, q4 = self.q

        l1p     = -self.manipulatorParams.d1 * math.sin(q2)               #projection of d1 on x-axis 
        l2p     =  self.manipulatorParams.d2 * math.cos(q3)               #projection of d2 on x-axis
        l       =  self.manipulatorParams.bx + l1p + l2p + self.manipulatorParams.mx    #total length from base to ee top projection 
 
        #forward kinematics to get ee position     
        x = l * math.cos(q1+self.eta[3]+self.manipulatorParams.alpha) + self.eta[0] + self.baseParams.bmx * math.cos(self.eta[3])
        y = l * math.sin(q1+self.eta[3]+self.manipulatorParams.alpha) + self.eta[1] + self.baseParams.bmx * math.sin(self.eta[3])
        z =-(self.manipulatorParams.bz + self.manipulatorParams.d1 * math.cos(-q2) + self.manipulatorParams.d2 * math.sin(q3) - self.manipulatorParams.mz) + self.eta[2] + self.baseParams.bmz

        return np.array([x, y, z]).reshape(3,1)
    
    '''
        Method that returns the End Effector orientation
    '''
    def getEEorientation(self):
        q1, q2, q3, q4 = self.q
 
        # forward kinematics to get ee orietation``     
        yaw_wtee = q1 + q4 + self.eta[3] + self.manipulatorParams.alpha

        return np.array([yaw_wtee]).reshape(1,1)
    
    '''
        Method that returns the moblie base position.
    '''
    def getMMposition(self):
        return np.array([self.eta[0], self.eta[1]]).reshape(2,1)
    
    '''
        Method that returns the moblie base position.
    '''
    def getMMorientation(self):
        return np.array([self.eta[3]]).reshape(1,1)
    
    '''
        Method that update the manipulator states from the sensor.
    '''
    def updateManipulatorState(self, swiftProJoint):
        self.q[0]      = swiftProJoint[0]
        self.q[1]      = swiftProJoint[1]
        self.q[2]      = swiftProJoint[2]
        self.q[3]      = swiftProJoint[3]
    
    '''
        Method that returns the moblie base state from the sensor.
    '''
    def updateMobileBaseState(self, eta):
        self.eta    = eta


           
        