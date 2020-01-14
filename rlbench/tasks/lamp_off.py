from typing import List
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint


class LampOff(Task):

    def init_task(self) -> None:
        self.bulb_glass_visual = Shape('bulb')
        self.bulb_glass_visual.set_color([1, 1, 1])
        self.joint = Joint('target_button_joint')
        self.condition = JointCondition(self.joint, 0.003)

    def init_episode(self, index: int) -> List[str]:
        self.bulb_glass_visual.set_color([1, 1, 1])
        self.register_success_conditions([self.condition])
        return ['turn off the light',
                'press the button to turn off the lamp',
                'press the light switch',
                'turn the lamp off',
                'close the gripper and press on the button until the light '
                'turns off']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if self.condition.condition_met() == (True, True):
            self.bulb_glass_visual.set_color([0, 0, 0])
