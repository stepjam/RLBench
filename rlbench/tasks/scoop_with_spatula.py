from rlbench.backend.task import Task
from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.conditions import GraspedCondition, DetectedCondition


class ScoopWithSpatula(Task):

    def init_task(self) -> None:
        spatula = Shape('scoop_with_spatula_spatula')
        self.register_graspable_objects([spatula])
        self.register_success_conditions([
            DetectedCondition(Shape('Cuboid'), ProximitySensor('success')),
            GraspedCondition(self.robot.gripper, spatula)
        ])

    def init_episode(self, index: int) -> List[str]:
        return ['scoop up the cube and lift it with the spatula',
                'scoop up the block and lift it with the spatula',
                'use the spatula to scoop the cube and lift it',
                'use the spatula to scoop the block and lift it',
                'pick up the cube using the spatula',
                'pick up the block using the spatula']

    def variation_count(self) -> int:
        return 1
