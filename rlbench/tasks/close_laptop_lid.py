from typing import List
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class CloseLaptopLid(Task):

    def init_task(self) -> None:
        self.register_success_conditions([JointCondition(Joint('joint'), 1.79)])

    def init_episode(self, index: int) -> List[str]:
        return ['close laptop lid',
                'close the laptop',
                'shut the laptop lid']

    def variation_count(self) -> int:
        return 1
