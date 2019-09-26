from typing import List, Tuple
import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.const import PrimitiveShape
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, ConditionSet
from rlbench.backend.spawn_boundary import SpawnBoundary

LIQUID_BALLS = 20


class PourFromCupToCup(Task):

    def init_task(self) -> None:
        self.drops = []
        self.cup_target_base = Dummy('cup_target_base')
        self.cup_source = Shape('cup_source')
        self.cup_target = Shape('cup_target')
        self.cup_source_visual = Shape('cup_source_visual')
        self.cup_target_visual = Shape('cup_target_visual')

        self.distractors = [Shape('cup_distractor%d' % i) for i in range(3)]
        self.distractors_vis = [Shape('cup_distractor_visual%d' % i)
                                for i in range(3)]

        self.success_detector = ProximitySensor('success')
        self.register_graspable_objects([self.cup_source])

    def init_episode(self, index: int) -> List[str]:
        target_index = (index + 1) % self.variation_count()
        source_name, source_rgb = colors[index]
        target_name, target_rgb = colors[target_index]

        self.cup_source_visual.set_color(source_rgb)
        self.cup_target_visual.set_color(target_rgb)

        options = (list(range(index)) + list(range(index + 1, len(colors))))
        options.remove(target_index)

        color_choices = np.random.choice(options, size=3, replace=False)
        for obj, color_index in zip(self.distractors_vis, color_choices):
            _, rgb = colors[color_index]
            obj.set_color(rgb)

        b = SpawnBoundary([Shape('boundary')])
        b.sample(self.cup_source, min_distance=0.12)
        b.sample(self.cup_target, min_distance=0.12)
        [b.sample(d, min_distance=0.12) for d in self.distractors]

        # Make the waypoints always be the same orientation
        self.cup_target_base.set_orientation([0.] * 3)

        self.drops = []
        conditions = []
        for i in range(LIQUID_BALLS):
            drop = Shape.create(PrimitiveShape.SPHERE, mass=0.0001,
                                size=[0.005, 0.005, 0.005])
            drop.set_parent(self.cup_source)
            drop.set_color([0.1, 0.1, 0.9])
            drop.set_position(list(np.random.normal(0, 0.0005, size=(3,))),
                              relative_to=self.cup_source)
            self.drops.append(drop)
            conditions.append(DetectedCondition(drop, self.success_detector))
        self.register_success_conditions([ConditionSet(conditions)])

        return [
            'pour liquid from the %s cup to the %s cup' % (
                source_name, target_name),
            'pour liquid from the %s mug to the %s mug' % (
                source_name, target_name),
            'pour the contents of the %s mug into the %s one' % (
                source_name, target_name),
            'pick up the %s cup and pour the liquid into the %s one' % (
                source_name, target_name)]

    def variation_count(self) -> int:
        return len(colors)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, 0], [0, 0, 0]

    def cleanup(self) -> None:
        for d in self.drops:
            d.remove()
        self.drops.clear()
