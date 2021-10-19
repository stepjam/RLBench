from typing import List
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import ConditionSet, DetectedCondition, \
    NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class PutKnifeOnChoppingBoard(Task):

    def init_task(self) -> None:
        knife = Shape('knife')
        self._knife_block = Shape('knife_block')
        self._boundary = SpawnBoundary([Shape('boundary')])
        self.register_graspable_objects([knife])
        self.register_success_conditions([
            ConditionSet([DetectedCondition(knife, ProximitySensor('success')),
                          NothingGrasped(self.robot.gripper)],
                         order_matters=True)])

    def init_episode(self, index: int) -> List[str]:
        self._boundary.clear()
        self._boundary.sample(self._knife_block)
        return ['put the knife on the chopping board',
                'slide the knife out of the knife block and put it down on the '
                'chopping board',
                'place the knife on the chopping board',
                'pick up the knife and leave it on the chopping board',
                'move the knife from the holder to the chopping board']

    def variation_count(self) -> int:
        return 1
