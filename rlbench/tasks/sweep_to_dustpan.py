from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition

DIRT_NUM = 5


class SweepToDustpan(Task):

    def init_task(self) -> None:
        broom = Shape('broom')
        success_sensor = ProximitySensor('success')
        dirts = [Shape('dirt' + str(i)) for i in range(DIRT_NUM)]
        conditions = [DetectedCondition(dirt, success_sensor) for dirt in dirts]
        self.register_graspable_objects([broom])
        self.register_success_conditions(conditions)

    def init_episode(self, index: int) -> List[str]:
        return ['sweep dirt to dustpan',
                'sweep the dirt up',
                'use the broom to brush the dirt into the dustpan',
                'clean up the dirt',
                'pick up the brush and clean up the table',
                'grasping the broom by its handle, clear way the dirt from the '
                'table',
                'leave the table clean']

    def variation_count(self) -> int:
        return 1
