from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import ConditionSet, DetectedCondition, \
    NothingGrasped


class PutKnifeOnChoppingBoard(Task):

    def init_task(self) -> None:
        knife = Shape('knife')
        success = ProximitySensor('success')
        self.knife_block = Shape('knife_block')
        self.chopping_board = Shape('chopping_board')
        self.register_graspable_objects([knife])
        cond_set = ConditionSet([DetectedCondition(knife, success),
                                 NothingGrasped(self.robot.gripper)],
                                order_matters=True)
        self.register_success_conditions([cond_set])
        self.boundary = SpawnBoundary([Shape('boundary')])

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        self.boundary.sample(self.knife_block)
        return ['put the knife on the chopping board',
                'slide the knife out of the knife block and put it down on the '
                'chopping board',
                'place the knife on the chopping board',
                'pick up the knife and leave it on the chopping board',
                'move the knife from the holder to the chopping board']

    def variation_count(self) -> int:
        return 1
