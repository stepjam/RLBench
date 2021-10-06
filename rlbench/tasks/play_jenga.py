from typing import List, Tuple
import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.task import Task


class PlayJenga(Task):

    def init_task(self) -> None:
        target = Shape('target_cuboid')
        original_detector = ProximitySensor('original_detector')
        bricks = [Shape('Cuboid%d' % i) for i in range(13)]
        conds = [DetectedCondition(b, original_detector) for b in bricks]
        conds.extend([
            DetectedCondition(target, original_detector, negated=True),
            NothingGrasped(self.robot.gripper)
        ])
        self.register_success_conditions(conds)
        self.register_graspable_objects([target])

    def init_episode(self, index: int) -> List[str]:
        return ['play jenga'
                'Take the protruding block out of the jenga tower without the '
                'tower toppling',
                'Keeping the tower from tumbling, remove the protruding '
                'jenga block',
                'Ensuring the jenga tower remains in place, slide the '
                'protruding block out']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 8], [0, 0, np.pi / 8]
