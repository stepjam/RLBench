from typing import List, Tuple

import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape

from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task


class TakeToiletRollOffStand(Task):

    def init_task(self) -> None:
        roll = Shape('toilet_roll')
        success_sensor = ProximitySensor('success')
        self.register_graspable_objects([roll])
        self.register_success_conditions(
            [DetectedCondition(roll, success_sensor, negated=True)])

    def init_episode(self, index: int) -> List[str]:
        return ['take toilet roll off stand',
                'slide the toilet paper of its stand and set it down',
                'place the roll onto the table',
                'get the toilet paper roll and then leave it on the table',
                'remove the toilet roll from the holder',
                'get the toilet paper',
                'grasping the end of the roll, pull if off of the stand, and '
                'set it on the table']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi), (0.0, 0.0, np.pi * 0.25)
