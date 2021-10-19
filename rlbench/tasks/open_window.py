from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class OpenWindow(Task):

    def init_task(self):
        self._left_unlocked_cond = JointCondition(
            Joint('left_handle_joint'), np.deg2rad(80))
        self._window_joint = Joint('left_window_joint')
        self.register_success_conditions([JointCondition(
            self._window_joint, np.deg2rad(30))])

    def init_episode(self, index: int) -> List[str]:
        self._left_unlocked = False
        self._window_joint.set_motor_locked_at_zero_velocity(True)
        return ['open left window',
                'rotate the handle to unlock the left window, then open it',
                'push the left window open',
                'use the handle to open the left window']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self._left_unlocked:
            self._left_unlocked = self._left_unlocked_cond.condition_met()[0]
            if self._left_unlocked:
                self._window_joint.set_motor_locked_at_zero_velocity(False)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 4.], [0, 0, np.pi / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
