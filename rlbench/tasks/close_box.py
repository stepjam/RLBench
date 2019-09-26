from typing import List
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class CloseBox(Task):

    def init_task(self) -> None:
        box_joint = Joint('joint')
        self.register_success_conditions([JointCondition(box_joint, 2.6)])

    def init_episode(self, index: int) -> List[str]:
        return ['close box',
                'close the lid on the box',
                'shut the box',
                'shut the box lid']

    def variation_count(self) -> int:
        return 1
