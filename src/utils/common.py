import numpy as np
from scipy.spatial.transform import Rotation as R

deg90 = np.pi / 2

def getTransformation(transformation: str, axis: str, value):
    if transformation == "lin":
        # Linear transformation
        if axis == "x":
            T = np.array([[1, 0, 0, value],
                          [0, 1, 0, 0],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        elif axis == "y":
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, value],
                          [0, 0, 1, 0],
                          [0, 0, 0, 1]])
        elif axis == "z":
            T = np.array([[1, 0, 0, 0],
                          [0, 1, 0, 0],
                          [0, 0, 1, value],
                          [0, 0, 0, 1]])
        else:
            raise ValueError("Invalid axis for linear transformation")
    elif transformation == "ang":
        # Angular transformation
        if axis == "x":
            r = R.from_rotvec(value * np.array([1, 0, 0]))
        elif axis == "y":
            r = R.from_rotvec(value * np.array([0, 1, 0]))
        elif axis == "z":
            r = R.from_rotvec(value * np.array([0, 0, 1]))
        else:
            raise ValueError("Invalid axis for angular transformation")
        T = np.round(np.block([[r.as_matrix(),np.zeros((3,1))],
                      [np.zeros((1,3)),np.array([1])]]).reshape(4,4),6)
    else:
        raise ValueError("Invalid transformation type")
    
    return T

def getTotalTransformation(transformation_list, axis_list, value):
    T= np.eye(4)
    for i in range(len(transformation_list)):
        T=T@getTransformation(transformation_list[i],axis_list[i],value[i])
    return T



def jacobianLink(T, revolute, link):  # Needed in Exercise 2
    """
    Function builds a Jacobian for the end-effector of a robot,
    described by a list of kinematic transformations and a list of joint types.

    Arguments:
    T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
    revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint
    link(integer): index of the link for which the Jacobian is computed

    Returns:
    (Numpy array): end-effector Jacobian
    """
    J_main = jacobian(T, revolute)
    J_link = np.copy(J_main)
    if link < len(revolute):
        J_link[:, link:] = 0
        return J_link
    else:
        return J_main


def weighted_DLS(A, damping, W):
    """
    Function computes the Weighted damped least-squares (DLS) solution to the matrix inverse problem.

    Arguments:
    A (Numpy array): matrix to be inverted
    damping (double): damping factor
    W (Numpy array): Weighting Diagonal Matrix

    Returns:
    (Numpy array): inversion of the input matrix
    """
    return (
        (np.linalg.inv(W))
        @ (A.T)
        @ np.linalg.inv(
            ((A @ (np.linalg.inv(W)) @ (A.T)) + (damping**2) * np.eye(A.shape[0]))
        )
    )


def kinematics(d, theta, a, alpha,Tb):
    '''
        Functions builds a list of transformation matrices, for a kinematic chain,
        descried by a given set of Denavit-Hartenberg parameters. 
        All transformations are computed from the base frame.

        Arguments:
        d (list of double): list of displacements along Z-axis
        theta (list of double): list of rotations around Z-axis
        a (list of double): list of displacements along X-axis
        alpha (list of double): list of rotations around X-axis

        Returns:
        (list of Numpy array): list of transformations along the kinematic chain (from the base frame)
    '''
    T = [Tb] 
    n=theta.shape[0]
    for i in range(n):
        T.append(T[-1]@DH(d[i],theta[i],a[i],alpha[i])) #appending each transformation
    return T

# Inverse kinematics
def jacobian(T, revolute):  
    '''
        Function builds a Jacobian for the end-effector of a robot,
        described by a list of kinematic transformations and a list of joint types.

        Arguments:
        T (list of Numpy array): list of transformations along the kinematic chain of the robot (from the base frame)
        revolute (list of Bool): list of flags specifying if the corresponding joint is a revolute joint

        Returns:
        (Numpy array): end-effector Jacobian
    '''
    n=len(revolute)
    
    J=np.zeros([6,0]) #Defining an empty Jacobian
    O_n= T[-1][0:3,3]
    for i in range(n):
        z=T[i][0:3,2].reshape(3,1)  #Indexing the right z axis
        o= (O_n - T[i][0:3,3]).reshape(3,1) #Indexing the righ O point
        
        if revolute[i]: #If the joint is revolute
            Jiv= np.cross(z.T,o.T).reshape(3,1)
            Ji=np.block([[Jiv],[z]])
            J=np.block([J,Ji])

        else:
            J=np.block([J,np.block([[z],[np.zeros((3,1))]])])
    return J



# Damped Least-Squares
def DLS(A, damping):
    '''
        Function computes the damped least-squares (DLS) solution to the matrix inverse problem.

        Arguments:
        A (Numpy array): matrix to be inverted
        damping (double): damping factor

        Returns:
        (Numpy array): inversion of the input matrix
    '''
    return (A.T)@ np.linalg.inv(((A@(A.T)) + (damping**2)* np.eye(A.shape[0])))