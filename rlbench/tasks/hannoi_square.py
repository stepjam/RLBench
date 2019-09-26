from typing import List
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.const import colors
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, ConditionSet
from rlbench.backend.spawn_boundary import SpawnBoundary



class HannoiSquare(Task):

    def init_task(self) -> None:
        self.square_ring = Shape('hannoi_square_ring')
        self.success_detector = ProximitySensor(
            'hannoi_square_success_detector')
        self.register_graspable_objects([self.square_ring])
        success_condition1 = DetectedCondition(self.square_ring,
                                               self.success_detector)
        self.register_success_conditions([success_condition1])

    def init_episode(self, index: int) -> List[str]:
        pillar0 = Shape('hannoi_square_pillar0')
        pillar1 = Shape('hannoi_square_pillar1')
        pillar2 = Shape('hannoi_square_pillar2')
        spokes = [pillar0, pillar1, pillar2]

        color_name, color_rgb = colors[index]
        chosen_pillar = np.random.choice(spokes)
        chosen_pillar.set_color(color_rgb)
        _, _, z = self.success_detector.get_position()
        x, y, _ = chosen_pillar.get_position()
        self.success_detector.set_position([x, y, z])

        color_choices = np.random.choice(
            list(range(index)) + list(range(index + 1, len(colors))),
            size=2, replace=False)
        spokes.remove(chosen_pillar)
        for spoke, i in zip(spokes, color_choices):
            name, rgb = colors[i]
            spoke.set_color(rgb)

        boundary1 = Shape('hannoi_square_boundary0')
        square_ring = Shape('hannoi_square_ring')
        b = SpawnBoundary([boundary1])
        b.sample(square_ring)

        return ['put the ring on the %s spoke' % color_name,
                'slide the ring onto the %s colored spoke' % color_name,
                'place the ring onto the %s spoke' %color_name]

    def variation_count(self) -> int:
        return len(colors)
