from enum import Enum


class ArmActionMode(Enum):

    # Absolute arm joint velocities
    ABS_JOINT_VELOCITY = 0

    # Change in arm joint velocities
    DELTA_JOINT_VELOCITY = 1

    # Absolute arm joint positions/angles (in radians)
    ABS_JOINT_POSITION = 2

    # Change in arm joint positions/angles (in radians)
    DELTA_JOINT_POSITION = 3

    # Absolute arm joint forces/torques
    ABS_JOINT_TORQUE = 4

    # Change in arm joint forces/torques
    DELTA_JOINT_TORQUE = 5

    # Absolute end-effector velocity (position (3) and quaternion (4))
    ABS_EE_VELOCITY = 6

    # Change in end-effector velocity (position (3) and quaternion (4))
    DELTA_EE_VELOCITY = 7

    # Absolute end-effector pose (position (3) and quaternion (4))
    ABS_EE_POSE = 8

    # Change in end-effector pose (position (3) and quaternion (4))
    DELTA_EE_POSE = 9

    # Absolute end-effector pose (position (3) and quaternion (4))
    # But does path planning between these points
    ABS_EE_POSE_PLAN = 10

    # Change in end-effector pose (position (3) and quaternion (4))
    # But does path planning between these points
    DELTA_EE_POSE_PLAN = 11


class GripperActionMode(Enum):

    # The open amount (0 >= x <= 1) of the gripper. 0 is close, 1 is open.
    OPEN_AMOUNT = 0


class ActionMode(object):

    def __init__(self,
                 arm: ArmActionMode=ArmActionMode.ABS_JOINT_VELOCITY,
                 gripper: GripperActionMode=GripperActionMode.OPEN_AMOUNT):
        self.arm = arm
        self.gripper = gripper
