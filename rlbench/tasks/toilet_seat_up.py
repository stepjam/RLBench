from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class ToiletSeatUp(Task):

    def init_task(self) -> None:
        self.register_success_conditions([
            JointCondition(Joint('toilet_seat_up_revolute_joint'), 1.40)])

    def init_episode(self, index: int) -> List[str]:
        return ['lift toilet seat up',
                'put the toilet seat up',
                'leave the lid of the toilet seat in a upright position',
                'grip the edge of the toilet seat and lift it up to an '
                'upright position',
                'leave the toilet lid open',
                'raise the toilet seat']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -np.pi / 4.], [0.0, 0.0, np.pi / 4.]
