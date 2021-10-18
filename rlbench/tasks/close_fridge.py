from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class CloseFridge(Task):

    def init_task(self) -> None:
        top_joint = Joint('top_joint')
        self.register_success_conditions(
            [JointCondition(top_joint, np.deg2rad(30))])

    def init_episode(self, index: int) -> List[str]:
        return ['close fridge',
                'close the fridge',
                'shut the fridge',
                'close the fridge door',
                'swing the fridge door shut']

    def variation_count(self) -> int:
        return 1

    def boundary_root(self) -> Object:
        return Shape('fridge_root')

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi / 4), (0.0, 0.0, np.pi / 4)
