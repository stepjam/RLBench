from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class CloseLaptopLid(Task):

    def init_task(self) -> None:
        laptop_joint = Joint('joint')
        self.register_success_conditions([JointCondition(laptop_joint, 1.79)])

    def init_episode(self, index: int) -> List[str]:
        return ['close laptop lid',
                'close the laptop',
                'shut the laptop lid']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, np.pi], [0.0, 0.0, np.pi]
