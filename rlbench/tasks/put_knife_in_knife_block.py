from typing import List, Tuple
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import ConditionSet, DetectedCondition, \
    NothingGrasped


class PutKnifeInKnifeBlock(Task):

    def init_task(self) -> None:
        knife = Shape('knife')
        self.knife_base = Dummy('knife_base')
        success = ProximitySensor('success')
        self.knife_block = Shape('knife_block')
        self.chopping_board = Shape('chopping_board')
        self.register_graspable_objects([knife])
        set = ConditionSet([DetectedCondition(knife, success),
                            NothingGrasped(self.robot.gripper)],
                           order_matters=True)
        self.register_success_conditions([set])
        self.boundary = SpawnBoundary([Shape('boundary')])
        self.knife_bound = SpawnBoundary([Shape('knife_boundary')])

    def init_episode(self, index: int) -> List[str]:
        self.knife_bound.clear()
        self.boundary.clear()
        self.boundary.sample(self.knife_block)
        self.boundary.sample(self.chopping_board)
        self.knife_bound.sample(self.knife_base)
        return ['put the knife in the knife block',
                'slide the knife into its slot in the knife block',
                'place the knife in the knife block',
                'pick up the knife and leave it in its holder',
                'move the knife from the chopping board to the holder']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, 0], [0, 0, 0]
