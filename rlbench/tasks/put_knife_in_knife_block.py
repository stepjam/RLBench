from typing import List, Tuple
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import ConditionSet, DetectedCondition, \
    NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class PutKnifeInKnifeBlock(Task):

    def init_task(self) -> None:
        knife = Shape('knife')
        self._knife_base = Dummy('knife_base')
        self._knife_block = Shape('knife_block')
        self._chopping_board = Shape('chopping_board')
        self._boundary = SpawnBoundary([Shape('boundary')])
        self._knife_bound = SpawnBoundary([Shape('knife_boundary')])
        self.register_graspable_objects([knife])
        self.register_success_conditions([
            ConditionSet([
                DetectedCondition(knife, ProximitySensor('success')),
                NothingGrasped(self.robot.gripper)],
                order_matters=True)])

    def init_episode(self, index: int) -> List[str]:
        self._knife_bound.clear()
        self._boundary.clear()
        self._boundary.sample(self._knife_block)
        self._boundary.sample(self._chopping_board)
        self._knife_bound.sample(self._knife_base)
        return ['put the knife in the knife block',
                'slide the knife into its slot in the knife block',
                'place the knife in the knife block',
                'pick up the knife and leave it in its holder',
                'move the knife from the chopping board to the holder']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, 0], [0, 0, 0]
