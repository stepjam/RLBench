from typing import List, Tuple
import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PutShoesInBox(Task):

    def init_task(self):
        shoe1, shoe2 = Shape('shoe1'), Shape('shoe2')
        self.register_graspable_objects([shoe1, shoe2])
        success_sensor = ProximitySensor('success_in_box')
        self.register_success_conditions([
            DetectedCondition(shoe1, success_sensor),
            DetectedCondition(shoe2, success_sensor),
            NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['put the shoes in the box',
                'open the box and place the shoes inside',
                'open the box lid and put the shoes inside']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
