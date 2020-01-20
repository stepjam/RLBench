from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy


class LightBulbIn(Task):

    def init_task(self) -> None:
        self.bulbs_visual = [Shape('light_bulb%d' % i) for i in range(2)]
        self.bulb_glass_visual = [Shape('bulb%d' % i) for i in range(2)]
        self.holders = [Shape('bulb_holder%d' % i) for i in range(2)]
        self.bulbs = [Shape('bulb_phys%d' % i) for i in range(2)]
        self.conditions = [NothingGrasped(self.robot.gripper)]
        self.register_graspable_objects(self.bulbs)
        self.boundary = Shape('spawn_boundary')

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        b = SpawnBoundary([self.boundary])
        for holder in self.holders:
            b.sample(holder, min_distance=0.01)
        self.w1 = Dummy('waypoint1')
        self.w1.set_position([0, 0, +10*10**(-3)],
                             relative_to=self.bulb_glass_visual[index % 2],
                             reset_dynamics=False)
        target_color_name, target_color_rgb = colors[index]
        color_choice = np.random.choice(
            list(range(index)) + list(
                range(index + 1, len(colors))),
            size=1, replace=False)[0]
        _, distractor_color_rgb = colors[color_choice]
        self.holders[index % 2].set_color(target_color_rgb)
        other_index = {0: 1, 1: 0}
        self.holders[other_index[index % 2]].set_color(distractor_color_rgb)
        self.register_success_conditions([DetectedCondition(
                                              self.bulbs[index % 2],
                                              ProximitySensor('success')),
                                          NothingGrasped(self.robot.gripper)])

        return ['screw in the %s light bulb' % target_color_name,
                'screw the light bulb from the %s holder into the lamp'
                % target_color_name,
                'pick up the light bulb from the %s stand, lift it up to just '
                'above the lamp, then screw it down into the lamp in a '
                'clockwise fashion' % target_color_name,
                'put the light bulb from the %s casing into the lamp'
                % target_color_name]

    def variation_count(self) -> int:
        return len(colors)

    def step(self) -> None:
        if DetectedCondition(self.bulbs[self._variation_index % 2],
                                  ProximitySensor('success')).condition_met() \
                == (True, True):
            self.bulb_glass_visual[self._variation_index % 2].set_color(
                [1.0, 1.0, 0.0])

    def cleanup(self) -> None:
        if self.bulb_glass_visual:
            [self.bulb_glass_visual[i].set_color([1.0, 1.0, 1.0])
             for i in range(2)]