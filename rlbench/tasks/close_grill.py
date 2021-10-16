from typing import List, Tuple
import numpy as np
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class CloseGrill(Task):

    def init_task(self) -> None:
        self.register_success_conditions([
            JointCondition(Joint('lid_joint'), np.deg2rad(50))])

    def init_episode(self, index: int) -> List[str]:
        return ['close the grill',
                'grasp the handle and lower the grill cover to close it',
                'close the bbq',
                'close the barbecue']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi / 2), (0.0, 0.0, np.pi / 2)

    def boundary_root(self) -> Object:
        return Shape('grill_root')
