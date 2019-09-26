from typing import List
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class OpenBox(Task):

    def init_task(self):
        box_joint = Joint('joint')
        self.register_success_conditions([JointCondition(box_joint, 1.9)])

    def init_episode(self, index: int) -> List[str]:
        return ['open box',
                'open the box lid',
                'open the box',
                'grasp the lid and open the box']

    def variation_count(self):
        return 1
