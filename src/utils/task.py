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
        self.name = name
        self.sigma_d = desired
        self.FeedForward = feedforward
        self.K = gain
        self.err_hist = []
        self.activation = 1

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
        return self.activation

    def updateActivation(self, robot):
        """
        Abstract method for updating the activation status of the task.

        Args:
            robot (Manipulator): Reference to the manipulator object.
        """
        pass  # Overridden in child class

    def setActivation(self, value):
        """
        Sets the activation status of the task.

        Args:
            value: Activation status (1 for active, 0 for inactive).
        """
        self.activation = value


class EEPosition3D(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)
        self.J = np.zeros((desired.shape[0], 4))
        self.err = np.zeros(desired.shape)
        # self.link = link
        self.FeedForward = feedforward
        self.K = gain

    def update(self, robot):
        self.J = robot.getEEJacobian()[0:3, :]
        self.err = self.K @ (self.getDesired() - robot.getEEposition()) + self.getFeedForward().reshape(self.getDesired().shape)


    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))




class EEOrientation3D(Task):
    def __init__(self, name, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)
        self.J = np.zeros((1, 2))  # Initialize with proper dimensions
        self.err = np.zeros((1, 1))  # Initialize with proper dimensions
        # self.link = link
        self.FeedForward = feedforward
        self.K = gain

    def update(self, robot):
        self.J = (robot.getEEJacobian()[-1, :]).reshape((1,6))  # Update task Jacobian
        self.err = self.K @ (self.getDesired() - robot.getEEorientation()) + self.getFeedForward().reshape(self.getDesired().shape[0], 1)
       

    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))


"""
    Subclass of Task, representing the 2D configuration task.
"""


class Configuration3D(Task):
    def __init__(self, name, desired, feedforward, gain, link):
        super().__init__(name, desired, feedforward, gain)
        self.J = np.zeros((5, 0))
        self.err = np.zeros((5, 0))
        self.link = link
        self.FeedForward = feedforward
        self.K = gain

    def update(self, robot):
        self.J = np.delete(robot.getLinkJacobian(self.link), 2, 0)
        self.err_pos = (
            (self.getDesired()[0:2, 0]).reshape(2, 1)
            - (
                (robot.getLinkTransform(self.link)[0:2, 3]).reshape(
                    (self.getDesired().shape[0]) - 3, 1
                )
            )
        ).reshape((self.getDesired().shape[0]) - 3, 1)
        r = R.from_matrix(robot.getLinkTransform(self.link)[0:3, 0:3])
        euler = (r.as_euler("xyz")).reshape(3, 1)
        self.err_ori = (self.getDesired()[2:, 0]).reshape(3, 1) - euler
        self.err = self.K @ (
            np.block([[self.err_pos], [self.err_ori]]).reshape(5, 1)
        ) + self.getFeedForward().reshape(5, 1)
        pass  # to remove

    def track_err(self):
        self.err_hist.append(
            (np.linalg.norm(self.err_pos), np.linalg.norm(self.err_ori))
        )


"""
    Subclass of Task, representing the joint position task.
"""


class JointPosition(Task):
    def __init__(self, name, desired_joint_number, desired, feedforward, gain):
        super().__init__(name, desired, feedforward, gain)
        self.J = np.zeros((1, 0))
        self.err = np.zeros((1, 1))
        self.desired_joint_number = desired_joint_number
        self.FeedForward = feedforward
        self.K = gain

    def update(self, robot):
        self.J = np.array(
            [
                1 if i == self.desired_joint_number - 1 else 0
                for i in range(robot.getDOF())
            ]
        ).reshape((1, robot.getDOF()))
        self.err = self.K @ (
            np.array(
                [self.getDesired() - robot.getJointPos(self.desired_joint_number)]
            ).reshape((1, 1))
        ) + self.FeedForward.reshape(1, 1)
        pass


""" 
    Subclass of Task, representing the joint limit task.
"""


class JointLimits(Task):
    def __init__(
        self,
        name,
        desired,
        lower_limits,
        upper_limits,
        activation_thresh,
        deactivation_thresh,
        feedforward,
        gain,
    ):
        super().__init__(name, desired, feedforward, gain)
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits
        self.J = np.zeros((len(desired), 4))
        self.err = np.zeros(len(desired))
        self.setActivation(np.zeros((6, 1)))
        self.activation_thresh = activation_thresh
        self.deactivation_thresh = deactivation_thresh

    def update(self, robot):
        # # Check if the current joint positions are within the limits
        self.updateActivation(robot)
        # Update the Jacobian and error based on the adjusted desired joint positions
        self.J = np.eye(robot.getDOF())
        self.err = self.getGain()

    def track_err(self):
        self.err_hist.append(np.linalg.norm(self.getError()))

    def updateActivation(self, robot):
        for i in range(robot.getDOF() - 2):
            if (
                robot.getJointPos(i + 1)
                >= (self.upper_limits[i] - self.activation_thresh)
            ) and (self.activation[i + 2] == 0):
                self.activation[i + 2] = -1

            elif (self.activation[i + 2] == 0) and (
                robot.getJointPos(i + 1)
                <= (self.lower_limits[i] + self.activation_thresh)
            ):
                self.activation[i + 2] = 1
            elif (self.activation[i + 2] == -1) and (
                robot.getJointPos(i + 1)
                <= (self.upper_limits[i] - self.deactivation_thresh)
            ):
                self.activation[i + 2] = 0
            elif (self.activation[i + 2] == 1) and (
                robot.getJointPos(i + 1)
                >= (self.lower_limits[i] + self.deactivation_thresh)
            ):
                self.activation[i + 2] = 0

    def isActive(self):
        # return 1 if there is  1 in activation array 0 if all zeros
        if (self.activation == 0).all():
            return 0
        else:
            return 1


class Obstacle3D(Task):
    def __init__(
        self,
        name,
        desired,
        obstacle,
        activation_thresh,
        deactivation_thresh,
        feedforward,
        gain,
    ):
        super().__init__(name, desired, feedforward, gain)
        self.obstacle = obstacle  # List of obstacle points
        self.J = np.zeros((3, 4))  # Initialize Jacobian
        self.err = np.zeros(3)  # Initialize error
        self.activation_thresh = activation_thresh
        self.deactivation_thresh = deactivation_thresh
        self.dist_to_obstacle = 0.0
        self.setActivation(False)

    def update(self, robot):
        # Calculate the error as the distance to the obstacle
        self.err = (
            robot.getEETransform()[0:2, 3].reshape(2, 1) - self.obstacle.reshape(2, 1)
        ) / (
            np.linalg.norm(
                self.obstacle.reshape(2, 1)
                - robot.getEETransform()[0:2, 3].reshape(2, 1)
            )
        )
        self.err = np.block([[self.err], [0.0]]).reshape(3, 1)
        self.J = robot.getEEJacobian()[0:3, :].reshape(3, robot.getDOF())

    def track_err(self):
        pass

    def updateActivation(self, robot):
        if (not self.isActive()) and (
            (
                np.linalg.norm(
                    self.obstacle.reshape(2, 1)
                    - robot.getEETransform()[0:2, 3].reshape(2, 1)
                )
            )
            <= self.activation_thresh
        ):
            self.setActivation(True)
        elif (self.isActive()) and (
            (
                np.linalg.norm(
                    self.obstacle.reshape(2, 1)
                    - robot.getEETransform()[0:2, 3].reshape(2, 1)
                )
            )
            >= self.deactivation_thresh
        ):
            self.setActivation(False)

        self.err_hist.append(
            np.linalg.norm(
                self.obstacle.reshape(2, 1)
                - robot.getEETransform()[0:2, 3].reshape(2, 1)
            )
        )
