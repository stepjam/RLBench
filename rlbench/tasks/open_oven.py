from typing import List, Tuple
import numpy as np
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.task import Task


class OpenOven(Task):

    def init_task(self) -> None:
        self.register_success_conditions(
            [DetectedCondition(Shape('oven_door'), ProximitySensor('success')),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['open the oven',
                'open the oven door',
                'grab hold of the the handle and pull the oven door open']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -np.pi / 4.], [0, 0, np.pi / 4.]

    def boundary_root(self) -> Object:
        return Shape('oven_boundary_root')
