from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.dummy import Dummy


class LightBulbOut(Task):

    def init_task(self) -> None:
        self.bulb_visual = Shape('light_bulb')
        self.bulb_glass_visual = Shape('bulb')
        self.holders = [Shape('bulb_holder%d' % i) for i in range(2)]
        self.bulb = Shape('bulb_phys')
        self.conditions = [NothingGrasped(self.robot.gripper)]
        self.boundary = Shape('spawn_boundary')
        self.register_graspable_objects([self.bulb])

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        b = SpawnBoundary([self.boundary])
        for holder in self.holders:
            b.sample(holder, min_distance=0.01)
        ProximitySensor('success').set_position([0, 0, 0],
                                                relative_to=self.holders[
                                                    index % 2],
                                                reset_dynamics=False)
        w1 = Dummy('waypoint1')
        w1.set_orientation([-np.pi, 0, -np.pi], reset_dynamics=False)
        w4 = Dummy('waypoint4')
        w4.set_orientation([-np.pi, 0, +3.735*10**(-1)], reset_dynamics=False)
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
            self.bulb,
            ProximitySensor('success')),
            NothingGrasped(self.robot.gripper)])
        return ['put the bulb in the %s holder' % target_color_name,
                'screw the bulb out and leave it in the %s stand' %
                target_color_name,
                'remove the bulb from the lamp and put it in the %s stand' %
                target_color_name,
                'take the light bulb out of the lamp and place it in the %s '
                'holder' % target_color_name,
                'grasp the light bulb, twist it anti clockwise until it is no '
                'longer attached to the lamp, and drop it into the %s stand'
                % target_color_name]

    def variation_count(self) -> int:
        return len(colors)

    def step(self) -> None:
        if DetectedCondition(self.bulb, ProximitySensor('lamp_detector'),
                             negated=True).condition_met() \
                == (True, True):
            self.bulb_glass_visual.set_color([1.0, 1.0, 1.0])

    def cleanup(self) -> None:
        if self.bulb_glass_visual:
            self.bulb_glass_visual.set_color([1.0, 1.0, 0.0])
