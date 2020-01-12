from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition,ConditionSet

# button top plate and wrapper will be be red before task completion
# and be changed to cyan upon success of task, so colors list used to randomly vary colors of
# base block will be redefined, excluding red and green
colors = [
    ('maroon', (0.5, 0.0, 0.0)),
    ('green', (0.0, 0.5, 0.0)),
    ('blue', (0.0, 0.0, 1.0)),
    ('navy', (0.0, 0.0, 0.5)),
    ('yellow', (1.0, 1.0, 0.0)),
    ('cyan', (0.0, 1.0, 1.0)),
    ('magenta', (1.0, 0.0, 1.0)),
    ('silver', (0.75, 0.75, 0.75)),
    ('gray', (0.5, 0.5, 0.5)),
    ('orange', (1.0, 0.5, 0.0)),
    ('olive', (0.5, 0.5, 0.0)),
    ('purple', (0.5, 0.0, 0.5)),
    ('teal', (0, 0.5, 0.5)),
    ('azure', (0.0, 0.5, 1.0)),
    ('violet', (0.5, 0.0, 1.0)),
    ('rose', (1.0, 0.0, 0.5)),
    ('black', (0.0, 0.0, 0.0)),
    ('white', (1.0, 1.0, 1.0)),
]


class PushButton(Task):

    def init_task(self):
        self.target_button = Shape('push_button_target')
        self.target_topPlate = Shape('target_button_topPlate')
        self.joint = Joint('target_button_joint')
        self.target_wrap = Shape('target_button_wrap')
        self.goal_condition = JointCondition(self.joint, 0.003)

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        self.target_topPlate.set_color([1.0, 0.0, 0.0])
        self.target_wrap.set_color([1.0, 0.0, 0.0])
        self.variation_index = index
        button_color_name, button_rgb = colors[index]
        self.target_button.set_color(button_rgb)
        self.register_success_conditions(
            [ConditionSet([self.goal_condition], True, False)])
        return ['push the %s button' % button_color_name,
                'push down the %s button' % button_color_name,
                'press the button with the %s base' % button_color_name,
                'press the %s button' % button_color_name]

    def variation_count(self) -> int:
        return len(colors)

    def step(self) -> None:
        if self.goal_condition.condition_met() == (True, True):
            self.target_topPlate.set_color([0.0, 1.0, 0.0])
            self.target_wrap.set_color([0.0, 1.0, 0.0])
