from typing import List
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
import numpy as np


class MoveHanger(Task):

    def init_task(self) -> None:
        self.hanger = Shape('clothes_hanger0')
        self.initx, self.inity, self.initz = self.hanger.get_position()
        self.register_graspable_objects([self.hanger])
        success_detector = ProximitySensor('success_detector')
        hanger_visual = Shape('clothes_hanger_visual0')
        self.register_success_conditions([
            DetectedCondition(hanger_visual, success_detector),
            NothingGrasped(self.robot.gripper)
        ])

    def init_episode(self, index: int) -> List[str]:
        self.hanger.set_position(
            [self.initx + np.random.uniform(-0.1, 0.3), self.inity, self.initz])
        return ['move hanger onto the other rack'
                'move the hanger from one rack to the other',
                'put the hanger on the other rack',
                'pick up the hanger and place it on the other rack']

    def variation_count(self) -> int:
        return 1

    def is_static_workspace(self) -> bool:
        return True
