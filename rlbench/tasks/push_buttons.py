from typing import List
import itertools
import math
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import JointCondition, ConditionSet

MAX_TARGET_BUTTONS = 3
MAX_VARIATIONS = 50

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

color_permutations = list(itertools.permutations(colors, 3))


def print_permutations(color_permutations):
    # pretty printing color_permutations for debug
    print('num permutations: ', str(len(color_permutations)))
    print('color_permutations:\n')
    for i in range(len(color_permutations)):
        print(str(color_permutations[i]))
        if ((i + 1) % 16 == 0): print('')


class PushButtons(Task):

    def init_task(self) -> None:
        self.buttons_pushed = 0
        self.color_variation_index = 0
        self.target_buttons = [Shape('push_buttons_target%d' % i)
                               for i in range(3)]
        self.target_topPlates = [Shape('target_button_topPlate%d' % i)
                                 for i in range(3)]
        self.target_joints = [Joint('target_button_joint%d' % i)
                              for i in range(3)]
        self.target_wraps = [Shape('target_button_wrap%d' % i)
                             for i in range(3)]
        self.boundaries = Shape('push_buttons_boundary')
        # goal_conditions merely state joint conditions for push action for
        # each button regardless of whether the task involves pushing it
        self.goal_conditions = [JointCondition(self.target_joints[n], 0.003)
                                for n in range(3)]

        self.register_waypoint_ability_start(0, self._move_above_next_target)
        self.register_waypoints_should_repeat(self._repeat)

    def init_episode(self, index: int) -> List[str]:
        for tp in self.target_topPlates:
            tp.set_color([1.0, 0.0, 0.0])
        for w in self.target_wraps:
            w.set_color([1.0, 0.0, 0.0])
        # For each color permutation, we want to have 1, 2 or 3 buttons pushed
        color_index = int(index / MAX_TARGET_BUTTONS)
        self.buttons_to_push = 1 + index % MAX_TARGET_BUTTONS
        button_colors = color_permutations[color_index]

        self.color_names = []
        self.color_rgbs = []
        self.chosen_colors = []
        i = 0
        for b in self.target_buttons:
            color_name, color_rgb = button_colors[i]
            self.color_names.append(color_name)
            self.color_rgbs.append(color_rgb)
            self.chosen_colors.append((color_name, color_rgb))
            b.set_color(color_rgb)
            i += 1

        # for task success, all button to push must have green color RGB
        self.success_conditions = []
        for i in range(self.buttons_to_push):
            self.success_conditions.append(self.goal_conditions[i])

        self.register_success_conditions(
            [ConditionSet(self.success_conditions, True, False)])

        rtn0 = 'push the %s button' % self.color_names[0]
        rtn1 = 'press the %s button' % self.color_names[0]
        rtn2 = 'push down the button with the %s base' % self.color_names[0]
        for i in range(self.buttons_to_push):
            if i == 0:
                continue
            else:
                rtn0 += ', then push the %s button' % self.color_names[i]
                rtn1 += ', then press the %s button' % self.color_names[i]
                rtn2 += ', then the %s one' % self.color_names[i]

        b = SpawnBoundary([self.boundaries])
        for button in self.target_buttons:
            b.sample(button, min_distance=0.1)

        num_non_targets = 3 - self.buttons_to_push
        spare_colors = list(set(colors)
                            - set(
            [self.chosen_colors[i] for i in range(self.buttons_to_push)]))

        spare_color_rgbs = []
        for i in range(len(spare_colors)):
            _, rgb = spare_colors[i]
            spare_color_rgbs.append(rgb)

        color_choice_indexes = np.random.choice(range(len(spare_colors)),
                                                size=num_non_targets,
                                                replace=False)
        non_target_index = 0
        for i, button in enumerate(self.target_buttons):
            if i in range(self.buttons_to_push):
                pass
            else:
                _, rgb = spare_colors[color_choice_indexes[non_target_index]]
                button.set_color(rgb)
                non_target_index += 1

        return [rtn0, rtn1, rtn2]

    def variation_count(self) -> int:
        return np.minimum(
            len(color_permutations) * MAX_TARGET_BUTTONS, MAX_VARIATIONS)

    def step(self) -> None:
        for i in range(len(self.target_buttons)):
            if self.goal_conditions[i].condition_met() == (True, True):
                self.target_topPlates[i].set_color([0.0, 1.0, 0.0])
                self.target_wraps[i].set_color([0.0, 1.0, 0.0])

    def cleanup(self) -> None:
        self.buttons_pushed = 0

    def _move_above_next_target(self, waypoint):
        if self.buttons_pushed >= self.buttons_to_push:
            print('buttons_pushed:', self.buttons_pushed, 'buttons_to_push:',
                  self.buttons_to_push)
            raise RuntimeError('Should not be here.')
        w0 = Dummy('waypoint0')
        x, y, z = self.target_buttons[self.buttons_pushed].get_position()
        w0.set_position([x, y, z + 0.083])
        w0.set_orientation([math.pi, 0, math.pi])

    def _repeat(self):
        self.buttons_pushed += 1
        return self.buttons_pushed < self.buttons_to_push
