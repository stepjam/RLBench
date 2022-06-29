from typing import List

import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class SlideBlockToTarget(Task):

    def init_task(self) -> None:
        self._block = Shape('block')
        self._target = ProximitySensor('success')
        self.register_success_conditions([
            DetectedCondition(self._block, self._target)])

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        return ['slide the block to target',
                'slide the block onto the target',
                'push the block until it is sitting on top of the target',
                'slide the block towards the green target',
                'cover the target with the block by pushing the block in its'
                ' direction']

    def variation_count(self) -> int:
        return 1

    def get_low_dim_state(self) -> np.ndarray:
        # One of the few tasks that have a custom low_dim_state function.
        return np.concatenate([
            self._block.get_position(), self._target.get_position()])

    def reward(self) -> float:
        grip_to_block = -np.linalg.norm(
            self._block.get_position() - self.robot.arm.get_tip().get_position())
        block_to_target = -np.linalg.norm(
            self._block.get_position() - self._target.get_position())
        return grip_to_block + block_to_target
