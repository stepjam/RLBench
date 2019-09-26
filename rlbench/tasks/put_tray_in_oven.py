from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PutTrayInOven(Task):

    def init_task(self) -> None:
        success_detector = ProximitySensor('success')
        self.tray = Shape('tray')
        self.register_graspable_objects([self.tray])
        self.register_success_conditions(
            [DetectedCondition(self.tray, success_detector),
             NothingGrasped(self.robot.gripper)])
        self.boundary = SpawnBoundary([Shape('oven_tray_boundary')])

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        self.boundary.sample(
            self.tray, min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        return ['put tray in oven',
                'place the tray in the oven',
                'open the oven, then slide the tray in',
                'open the oven door, pick up the tray, and put it down on the '
                'oven shelf']


    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('oven_boundary_root')
