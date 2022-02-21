from typing import List

from pyrep.objects import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape

from rlbench.backend.conditions import DetectedCondition, ConditionSet, \
    GraspedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class LiftNumberedBlock(Task):

    def init_task(self) -> None:
        self._blocks = [Shape('block%d' % i) for i in range(1, 4)]
        self._anchor = [Dummy('anchor%d' % i) for i in range(1, 4)]
        self.register_graspable_objects(self._blocks)
        self._boundary = SpawnBoundary([Shape('boundary')])
        self._success_detector = ProximitySensor('success')
        self._w1 = Dummy('waypoint1')

    def init_episode(self, index: int) -> List[str]:
        block_num = index + 1
        target_block = self._blocks[index]
        self._boundary.clear()
        for block in self._blocks:
            self._boundary.sample(block, min_distance=0.2)
        self._w1.set_pose(self._anchor[index].get_pose())

        self.register_success_conditions([
            GraspedCondition(self.robot.gripper, target_block),
            DetectedCondition(target_block, self._success_detector)
        ])

        return ['pick up the block with the number %d' % block_num,
                'grasp the %d numbered block and lift' % block_num,
                'lift the %d numbered block' % block_num]

    def variation_count(self) -> int:
        return 3

    def is_static_workspace(self) -> bool:
        return True
