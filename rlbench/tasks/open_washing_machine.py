from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint

from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task

from typing import List, Tuple
from math import pi

import numpy as np

class OpenWashingMachine(Task):
    def init_task(self) -> None:
        self.joint = Joint('door_joint')
        self.register_success_conditions([
            JointCondition(self.joint, np.deg2rad(40))
        ])

    def init_episode(self, index: int) -> List[str]:
        return ['open washing machine',
                'open the washing machine door',
                'pull the washing machine door open']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
