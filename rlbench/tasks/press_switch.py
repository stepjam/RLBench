from typing import List
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class PressSwitch(Task):

    def init_task(self) -> None:
        switch_joint = Joint('joint')
        self.register_success_conditions([JointCondition(switch_joint, 1.0)])

    def init_episode(self, index: int) -> List[str]:
        return ['press switch',
                'turn the switch on or off',
                'flick the switch']

    def variation_count(self) -> int:
        return 1
