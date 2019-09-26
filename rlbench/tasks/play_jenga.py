from typing import List, Tuple
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PlayJenga(Task):

    def init_task(self) -> None:
        target = Shape('target_cuboid')
        original_detector = ProximitySensor('original_detector')
        self.register_success_conditions(
            [DetectedCondition(Shape('Cuboid0'), original_detector),
             DetectedCondition(Shape('Cuboid1'), original_detector),
             DetectedCondition(Shape('Cuboid2'), original_detector),
             DetectedCondition(Shape('Cuboid3'), original_detector),
             DetectedCondition(Shape('Cuboid4'), original_detector),
             DetectedCondition(Shape('Cuboid5'), original_detector),
             DetectedCondition(Shape('Cuboid6'), original_detector),
             DetectedCondition(Shape('Cuboid7'), original_detector),
             DetectedCondition(Shape('Cuboid8'), original_detector),
             DetectedCondition(Shape('Cuboid9'), original_detector),
             DetectedCondition(Shape('Cuboid10'), original_detector),
             DetectedCondition(Shape('Cuboid11'), original_detector),
             DetectedCondition(Shape('Cuboid12'), original_detector),
             DetectedCondition(target, original_detector, negated=True),
             NothingGrasped(self.robot.gripper)])
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
        return [0, 0, 0], [0, 0, 3.14]
