from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class ToiletSeatDown(Task):

    def init_task(self) -> None:
        self.register_success_conditions([
            JointCondition(Joint('toilet_seat_up_revolute_joint'), 1.7)])

    def init_episode(self, index: int) -> List[str]:
        return ['toilet seat down',
                'put the toilet seat down',
                'grasping the top of the lid, close the toilet seat',
                'put the toilet lid down',
                'leave the toilet seat down',
                'close the lid on the toilet',
                'leave the toilet seat down',
                'lower the toilet seat',
                'grip the edge of the toilet lid and lower it flat on the '
                'toilet seat']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -np.pi / 4.], [0.0, 0.0, np.pi / 4.]
