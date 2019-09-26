from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class SetTheTable(Task):

    def init_task(self) -> None:
        plate = Shape('plate')
        fork = Shape('fork')
        knife = Shape('knife')
        spoon = Shape('spoon')
        glass = Shape('glass')
        self.register_success_conditions([
            DetectedCondition(plate, ProximitySensor('plate_detector')),
            DetectedCondition(fork, ProximitySensor('fork_detector')),
            DetectedCondition(knife, ProximitySensor('knife_detector')),
            DetectedCondition(spoon, ProximitySensor('spoon_detector')),
            DetectedCondition(glass, ProximitySensor('glass_detector')),
            NothingGrasped(self.robot.gripper)])
        self.register_graspable_objects([plate, fork, knife, spoon, glass])

    def init_episode(self, index: int) -> List[str]:
        return ['set the table'
                'place the dishes and cutlery on the table in preparation for '
                'a meal',
                'pick up the plate and put it down on the table, then place '
                'the fork to its left, the knife and then the spoon to its '
                'right, and set the glass down just above them',
                'prepare the table for a meal',
                'arrange the plate, cutlery and glass neatly on the table '
                'so that a person can eat',
                'get the table ready for lunch',
                'get the table ready for dinner']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
