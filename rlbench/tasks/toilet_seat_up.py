from typing import List, Tuple
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class ToiletSeatUp(Task):

    def init_task(self) -> None:
        joint = Joint('toilet_seat_up_revolute_joint')
        self.register_success_conditions([JointCondition(joint, 1.40)])

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
        return [0.0, 0.0, -3.14 / 4.], [0.0, 0.0, 3.14 / 4.]
