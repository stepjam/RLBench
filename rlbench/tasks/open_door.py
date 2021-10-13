from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class OpenDoor(Task):

    def init_task(self) -> None:
        self._door_joint = Joint('door_frame_joint')
        self.register_success_conditions([
            JointCondition(self._door_joint, np.deg2rad(25))])
        self._door_unlock_cond = JointCondition(
            Joint('door_handle_joint'), np.deg2rad(25))

    def init_episode(self, index: int) -> List[str]:
        self._door_unlocked = False
        self._door_joint.set_motor_locked_at_zero_velocity(True)
        return ['open the door',
                'grip the handle and push the door open',
                'use the handle to open the door']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self._door_unlocked:
            self._door_unlocked = self._door_unlock_cond.condition_met()[0]
            if self._door_unlocked:
                self._door_joint.set_motor_locked_at_zero_velocity(False)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 4.], [0, 0, np.pi / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
