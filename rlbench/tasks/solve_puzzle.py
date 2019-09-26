from typing import List

from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape

from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.task import Task
    

class SolvePuzzle(Task):

    def init_task(self) -> None:
        piece2 = Shape('solve_puzzle_piece2')
        self.register_graspable_objects([piece2])
        self.register_success_conditions([
            NothingGrasped(self.robot.gripper),
            DetectedCondition(piece2, ProximitySensor('success'))
        ])

    def init_episode(self, index: int) -> List[str]:
        return [
            'solve the puzzle',
            'put the puzzle piece into the puzzle',
            'pick up the puzzle piece and place it on the puzzle'
        ]

    def variation_count(self) -> int:
        return 1
