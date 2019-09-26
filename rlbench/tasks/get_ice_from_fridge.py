from typing import List, Tuple
import numpy as np
from pyrep.objects.object import Object
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.const import PrimitiveShape
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, JointCondition

ICE_NUM = 5


class GetIceFromFridge(Task):

    def init_task(self) -> None:
        self.drops = []
        self.success_sensor = ProximitySensor('success')
        self.cup = Shape('cup')
        self.register_graspable_objects([self.cup])
        self.tongue_condition = JointCondition(Joint('joint'), np.deg2rad(1))

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions(
            [DetectedCondition(self.robot.gripper, self.success_sensor)])
        self.tongued = False
        self.tonguedOnce = False
        return ['get ice from fridge',
                'use the cup to get ice from the fridge',
                'pick up the cup and fill it with ice from the fridge',
                'push the cup up against the tongue of the ice dispenser to '
                'retrieve ice from the fridge']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self.tongued:
            self.tongued = self.tongue_condition.condition_met()[0]
            if self.tongued and not self.tonguedOnce:
                for i in range(ICE_NUM):
                    drop = Shape.create(PrimitiveShape.CUBOID, mass=0.0001,
                                        size=[0.005, 0.005, 0.005])
                    drop.set_color([0.1, 0.1, 0.9])
                    pos = list(np.random.normal(0, 0.0005, size=(3,)))
                    drop.set_position(pos,
                                      relative_to=self.cup)
                    self.drops.append(drop)
                self.register_success_conditions(
                    [DetectedCondition(self.drops[i], self.success_sensor) for i
                     in range(ICE_NUM)])
                self.tonguedOnce = True

    def cleanup(self) -> None:
        for d in self.drops:
            d.remove()
        self.drops = []

    def boundary_root(self) -> Object:
        return Shape('fridge_root')

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -3.14/2), (0.0, 0.0, 3.14/2)
