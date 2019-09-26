from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class StraightenRope(Task):

    def init_task(self) -> None:
        rope_head = Shape('head')
        rope_tail = Shape('tail')
        success_head = ProximitySensor('success_head')
        success_tail = ProximitySensor('success_tail')
        self.register_success_conditions(
            [DetectedCondition(rope_head, success_head),
             DetectedCondition(rope_tail, success_tail)])

    def init_episode(self, index: int) -> List[str]:
        return ['straighten rope',
                'pull the rope straight',
                'grasping each end of the rope in turn, leave the rope straight'
                ' on the table',
                'pull each end of the rope until is is straight',
                'tighten the rope',
                'pull the rope tight']

    def variation_count(self) -> int:
        return 1
