from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class OpenBox(Task):

    def init_task(self):
        self.register_success_conditions([
            JointCondition(Joint('box_joint'), np.pi / 2)])

    def init_episode(self, index: int) -> List[str]:
        return ['open box',
                'open the box lid',
                'open the box',
                'grasp the lid and open the box']

    def variation_count(self):
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
