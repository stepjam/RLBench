from typing import List, Tuple

import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import (DetectedSeveralCondition,
                                        NothingGrasped, ConditionSet)
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task
from rlbench.const import colors


class BlockPyramid(Task):

    def init_task(self) -> None:
        self.blocks = [Shape('block_pyramid_block%d' % i) for i in range(6)]
        self.distractors = [Shape(
            'block_pyramid_distractor_block%d' % i) for i in range(6)]
        success_detectors = [ProximitySensor(
            'block_pyramid_success_block%d' % i) for i in range(3)]

        cond_set = ConditionSet([
            DetectedSeveralCondition(self.blocks, success_detectors[0], 3),
            DetectedSeveralCondition(self.blocks, success_detectors[1], 2),
            DetectedSeveralCondition(self.blocks, success_detectors[2], 1),
            NothingGrasped(self.robot.gripper)
        ])
        self.register_success_conditions([cond_set])
        self.register_graspable_objects(self.blocks + self.distractors)
        self.spawn_boundary = SpawnBoundary(
            [Shape('block_pyramid_boundary%d' % i) for i in range(4)])

    def init_episode(self, index: int) -> List[str]:

        color_name, color_rgb = colors[index]
        for obj in self.blocks:
            obj.set_color(color_rgb)

        color_choice = np.random.choice(
            list(range(index)) + list(range(index + 1, len(colors))),
            size=1, replace=False)[0]
        name, rgb = colors[color_choice]
        for obj in self.distractors:
            obj.set_color(rgb)
        self.spawn_boundary.clear()
        for ob in self.blocks + self.distractors:
            self.spawn_boundary.sample(
                ob, min_distance=0.08, min_rotation=(0.0, 0.0, -np.pi / 4),
                max_rotation=(0.0, 0.0, np.pi / 4))

        return ['stack %s blocks in a pyramid' % color_name,
                'create a pyramid with the %s objects' % color_name,
                'make a pyramid out of %s cubes' % color_name,
                'position the %s blocks in the shape of a pyramid' % color_name,
                'use the %s blocks to build a pyramid' % color_name]

    def variation_count(self) -> int:
        return len(colors)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
