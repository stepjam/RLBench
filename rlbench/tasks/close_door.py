from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition, NothingGrasped


class CloseDoor(Task):

    def init_task(self) -> None:
        door_joint = Joint('door_frame_joint')
        self.register_success_conditions(
            [JointCondition(door_joint, np.deg2rad(25)),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['close the door',
                'shut the door',
                'grip the handle and slide the door shut',
                'use the handle to move the door closed']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
