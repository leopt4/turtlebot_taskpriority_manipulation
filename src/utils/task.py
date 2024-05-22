from .common import *

class Task:
    """
    Task class represents a generic task for a manipulator.

    Attributes:
        name (str): Title of the task.
        sigma_d (numpy array): Desired sigma (goal).
        FeedForward: Feedforward component for the task.
        K: Gain for the task.
        err_hist (list): History of task errors.
        activation (int): Activation status of the task (1 for active, 0 for inactive).
    """

    def __init__(self, name: str, desired, feedforward, gain):
        """
        Initializes a Task object with the given parameters.

        Args:
            name (str): Title of the task.
            desired: Desired sigma (goal).
            feedforward: Feedforward component for the task.
            gain: Gain for the task.
        """
        self.name           = name                  # task title
        self.sigma_d        = desired               # desired sigma
        self.task_dim       = np.shape(desired)[0]  # Get task dimension
        self.FeedForward    = feedforward           # Feed forward velocity
        self.K              = gain                  # Gain feed forward controller      
        self.err_hist = []

        self.err    = np.zeros((self.task_dim, 1))              # Initialize with proper dimensions

        self.active = 1                             # Activation function

    def update(self, robot):
        """
        Abstract method for updating the task variables.

        Args:
            robot (Manipulator): Reference to the manipulator object.
        """
        pass # Overridden in child class

    def setDesired(self, value):
        """
        Sets the desired sigma for the task.

        Args:
            value: Value of the desired sigma (goal).
        """
        self.sigma_d = value

    def getDesired(self):
        """
        Gets the desired sigma for the task.

        Returns:
            numpy array: Desired sigma (goal).
        """
        return self.sigma_d

    def getJacobian(self):
        """
        Gets the task Jacobian.

        Returns:
            object: Task Jacobian.
        """
        return self.J

    def getError(self):
        """
        Gets the task error (tilde sigma).

        Returns:
            object: Task error.
        """
        return self.err

    def track_err(self):
        """ 
        Tracks the task error by appending it to the error history.
        """
        self.err_hist.append(self.getError())

    def setFeedForward(self, value):
        """
        Sets the feedforward component for the task.

        Args:
            value: Feedforward component value.
        """
        self.FeedForward = value

    def getFeedForward(self):
        """
        Gets the feedforward component for the task.

        Returns:
            object: Feedforward component value.
        """
        return self.FeedForward

    def setGain(self, value):
        """
        Sets the gain for the task.

        Args:
            value: Gain value.
        """
        self.K = value

    def getGain(self):
        """
        Gets the gain for the task.

        Returns:
            object: Gain value.
        """
        return self.K

    def isActive(self):
        """
        Checks if the task is active.

        Returns:
            int: Activation status (1 for active, 0 for inactive).
        """
        return self.active


    def setActive(self, value):
        """
        Sets the activation status of the task.

        Args:
            value: Activation status (1 for active, 0 for inactive).
        """
        self.active = value

"""
    Subclass of Task, representing the 3D Position of the End Effector task.
"""
class EEPosition3D(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)
        self.J = np.zeros((self.task_dim, 6))

    def update(self, robot):
        DoF     = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J = robot.getEEJacobian()[0:3, :]
        # Update task error
        self.err = self.K @ (self.getDesired() - robot.getEEposition()) + self.getFeedForward().reshape(self.task_dim, 1)


    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))

"""
    Subclass of Task, representing the 3D Orientation of the End Effector task.
"""
class EEOrientation3D(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)

    def update(self, robot):
        DoF     = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J  = (robot.getEEJacobian()[-1, :]).reshape((self.task_dim, DoF)) 
        # Update task error
        self.err = self.K @ (self.getDesired() - robot.getEEorientation()) + self.getFeedForward().reshape(self.task_dim, 1)
       

    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))

"""
    Subclass of Task, representing the 2D configuration task.
"""
class EEConfiguration3D(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)

    def update(self, robot):
        DoF = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J = (robot.getEEJacobian()[[0,1,2,5]]).reshape((self.task_dim, DoF))

        # Update task error
        self.err_pos = (self.getDesired()[0:3, 0]).reshape(3, 1) - robot.getEEposition()

        self.err_ori = normalize_angle(self.getDesired()[-1] - robot.getEEorientation())
        
        self.err = self.K @ (
            np.block([[self.err_pos], [self.err_ori]]).reshape(self.task_dim, 1)
        ) + self.getFeedForward().reshape(self.task_dim, 1)

    def track_err(self):
        self.err_hist.append(
            (np.linalg.norm(self.err_pos), np.linalg.norm(self.err_ori))
        )

"""
    Subclass of Task, representing the Heading Orientation of the mobile base task.
"""
class MMOrientation(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)
        
    def update(self, robot):
        DoF     = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J  = (robot.getMMJacobian()[-1, :]).reshape((self.task_dim, DoF)) 
        # Update task error
        self.err = self.K @ (normalize_angle(self.getDesired() - robot.getMMorientation())) + self.getFeedForward().reshape(self.task_dim, 1)

    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))


"""
    Subclass of Task, representing the position of the mobile base task.
"""
class MMPosition(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)

    def update(self, robot):
        DoF     = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J  = (robot.getMMJacobian()[0:2, :]).reshape((self.task_dim, DoF)) 
        # Update task error
        self.err = self.K @ (self.getDesired() - robot.getMMposition()) + self.getFeedForward().reshape(self.task_dim, 1)
       

    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))

"""
    Subclass of Task, representing the position of the mobile base task.
"""
class MMConfiguration(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)

    def update(self, robot):
        DoF     = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J  = (robot.getMMJacobian()[[0,1,5]]).reshape((self.task_dim, DoF)) 

        # Update task error
        self.err_pos = (self.getDesired()[0:2, 0]).reshape(2, 1) - robot.getMMposition()

        self.err_ori = self.getDesired()[-1] - robot.getMMorientation()
        
        self.err = self.K @ (
            np.block([[self.err_pos], [self.err_ori]]).reshape(self.task_dim, 1)
        ) + self.getFeedForward().reshape(self.task_dim, 1)


    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))

'''
    Subclass of Task, representing joint limits (inequality task).
'''
class Limit2D(Task):
  
    def __init__(self, name, limit_range, threshold, feedforward, gain, link_index:int):
        super().__init__(name, limit_range, feedforward, gain)
        self.threshold  = threshold         # Threshold [alpha, sigma].reshape(2,1)
        self.active     = 0                 # Initialise activation function is 0
        self.link_index = link_index

    def update(self, robot):
        DoF = robot.getDOF()
        # Update Jacobean matrix - task Jacobian
        self.J  = np.zeros((1,DoF))
        self.J[0,self.link_index+1] = 1
        # Update task error
        q_i = robot.getJointPos(self.link_index)
        
        self.err = np.array([1.0]).reshape(1,1)
        # Compute activation function
        if self.active == 0 and q_i >= self.getDesired()[0,1] - self.threshold[0]:
            self.active = -1
        elif self.active == 0 and q_i <= self.getDesired()[0,0] + self.threshold[0]:
            self.active = 1
        elif self.active == -1 and q_i <= self.getDesired()[0,1] - self.threshold[1]:
            self.active = 0
        elif self.active == 1 and q_i >= self.getDesired()[0,0] + self.threshold[1]:
            self.active = 0

        return True
