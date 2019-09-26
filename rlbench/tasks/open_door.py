from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class OpenDoor(Task):

    def init_task(self) -> None:
        self.door_main = Shape('door_main')
        self.door_main.set_dynamic(False)
        door_joint = Joint('door_frame_joint')
        handle_joint = Joint('door_handle_joint')
        self.register_success_conditions(
            [JointCondition(door_joint, np.deg2rad(25))])
        self.door_unlock_cond = JointCondition(
            handle_joint, np.deg2rad(25))

    def init_episode(self, index: int) -> List[str]:
        self.door_unlocked = False
        return ['open the door',
                'grip the handle and slide the door open',
                'use the handle to move the door open']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self.door_unlocked:
            self.door_unlocked = self.door_unlock_cond.condition_met()[0]
            if self.door_unlocked:
                self.door_main.set_dynamic(True)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
