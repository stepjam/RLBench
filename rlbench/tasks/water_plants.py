from typing import List
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.const import PrimitiveShape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition

WATER_NUM = 5


class WaterPlants(Task):

    def init_task(self) -> None:
        self.drops = []
        self.success_sensor = ProximitySensor('success')
        pour_point = ProximitySensor('pour_point')
        self.waterer = Shape('waterer')
        self.head = Shape('head')
        self.register_graspable_objects([self.waterer])
        self.pour_point_reached = DetectedCondition(
            self.head, pour_point)

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions(
            [DetectedCondition(self.waterer, self.success_sensor)])
        self.reached = False
        self.reachedOnce = False
        return ['water plant',
                'pick up the watering can by its handle and water the plant',
                'pour some water on the plant',
                'the plant needs hydration',
                'pour water from the watering can into the plant pot',
                'water the soil']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self.reached:
            self.reached = self.pour_point_reached.condition_met()[0]
            if self.reached and not self.reachedOnce:
                for i in range(WATER_NUM):
                    drop = Shape.create(PrimitiveShape.CUBOID, mass=0.0001,
                                        size=[0.005, 0.005, 0.005])
                    drop.set_color([0.1, 0.1, 0.9])
                    pos = list(np.random.normal(0, 0.0005, size=(3,)))
                    drop.set_position(pos,
                                      relative_to=self.head)
                    self.drops.append(drop)
                self.register_success_conditions(
                    [DetectedCondition(self.drops[i], self.success_sensor) for i
                     in range(WATER_NUM)])
                self.reachedOnce = True

    def cleanup(self) -> None:
        for d in self.drops:
            d.remove()
        self.drops = []
