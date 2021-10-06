from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class CloseBox(Task):

    def init_task(self) -> None:
        self.register_success_conditions([
            JointCondition(Joint('box_joint'), 2.6)])

    def init_episode(self, index: int) -> List[str]:
        return ['close box',
                'close the lid on the box',
                'shut the box',
                'shut the box lid']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
