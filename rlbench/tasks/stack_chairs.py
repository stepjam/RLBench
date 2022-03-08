
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape

from rlbench.backend.conditions import DetectedCondition, NothingGrasped, Condition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from rlbench.const import colors

from typing import List

import numpy as np

class ChairsOrientedCondition(Condition):
    def __init__(self, objs: List[Object], error: float):
        self.objs = objs
        self.error = error

    def condition_met(self):
        for obj in self.objs[:-1]:
            x, y, z = obj.get_orientation(self.objs[-1])
            if abs(x) > self.error or abs(y) > self.error or abs(z) > self.error:
                return False, False
        return True, False

class StackChairs(Task):
    def init_task(self) -> None:
        self.chair1 = Shape('chair1')
        self.chair2 = Shape('chair2')
        self.chair3 = Shape('chair3')

        self.cushions1 = Shape('cushions1')
        self.cushions2 = Shape('cushions2')
        self.cushions3 = Shape('cushions3')

        self.boundary = SpawnBoundary([Shape('boundary')])
        self.detector = ProximitySensor('success')

        self.register_graspable_objects([self.chair1, self.chair2, self.chair3])
        self.register_success_conditions([
            DetectedCondition(self.chair1, self.detector),
            DetectedCondition(self.chair2, self.detector),
            NothingGrasped(self.robot.gripper),
            ChairsOrientedCondition([self.chair1, self.chair2, self.chair3], error = 0.25)
        ])

    def init_episode(self, index: int) -> List[str]:
        indices = np.random.choice(np.arange(len(colors)), size = 2, replace = False)

        color1, rgb1 = colors[index]
        color2, rgb2 = colors[indices[0]]
        color3, rgb3 = colors[indices[1]] # target chair color

        self.cushions1.set_color(rgb1)
        self.cushions2.set_color(rgb2)
        self.cushions3.set_color(rgb3)

        self.boundary.clear()
        self.boundary.sample(self.chair1, min_distance=0.1,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.chair2, min_distance=0.1,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))
        self.boundary.sample(self.chair3, min_distance=0.1,
                             min_rotation=(0, 0, 0), max_rotation=(0, 0, 0))

        return [f'stack the other chairs on top of the {color1} chair',
                f'put the remaining two chairs on top of the {color1} chair',
                f'pick up and set the chairs down onto the {color1} chair',
                f'create a stack of chairs with the {color1} chair at its base',
                f'keeping the {color1} chair on the table, stack the other two onto it']

    def variation_count(self) -> int:
        return len(colors)
