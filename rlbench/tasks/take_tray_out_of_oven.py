from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class TakeTrayOutOfOven(Task):

    def init_task(self) -> None:
        success_detector = ProximitySensor('success')
        tray = Shape('tray')
        self.register_graspable_objects([tray])
        self.register_success_conditions(
            [DetectedCondition(tray, success_detector, negated=True),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['take tray out of oven',
                'open the oven and take the baking tray out',
                'grasp the handle on the over door, open it, and remove the '
                'tray from the oven',
                'get the baking tray from the oven',
                'get the tray',
                'take out the tray',
                'retrieve the oven tray']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('oven_boundary_root')
