from pyrep.robots.arms.arm import Arm
from pyrep.robots.end_effectors.gripper import Gripper


class Robot(object):
    """Simple container for the robot components.
    """

    def __init__(self, arm: Arm, gripper: Gripper):
        self.arm = arm
        self.gripper = gripper
