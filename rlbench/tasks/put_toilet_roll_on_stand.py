from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary


class PutToiletRollOnStand(Task):

    def init_task(self) -> None:
        roll = Shape('toilet_roll')
        self.boundary_roll = SpawnBoundary([Shape('boundary_roll')])
        self.boundary_stand = SpawnBoundary([Shape('boundary_stand')])
        self.roll_box = Shape('toilet_roll_box')
        self.stand_base = Shape('stand_base')
        success_sensor = ProximitySensor('success')
        self.register_graspable_objects([roll])
        self.register_success_conditions(
            [DetectedCondition(roll, success_sensor),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        self.boundary_roll.clear()
        self.boundary_stand.clear()
        self.boundary_roll.sample(self.roll_box)
        self.boundary_stand.sample(self.stand_base)
        return ['put toilet roll on stand',
                'place the toilet roll on the stand',
                'slide the roll onto its holder']

    def variation_count(self) -> int:
        return 1
