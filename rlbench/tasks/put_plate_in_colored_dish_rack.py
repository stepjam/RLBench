from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition, \
    NothingGrasped, ConditionSet
from rlbench.const import colors


class PutPlateInColoredDishRack(Task):

    def init_task(self) -> None:
        plate = Shape('plate')
        self.dish_rack = Shape('dish_rack')
        self.plate_stand = Shape('plate_stand')
        self.success_sensor = ProximitySensor('success')
        self.success_poses = [Dummy('success_pos%d' % i)
                              for i in range(3)]
        self.pillars = [Shape('dish_rack_pillar%d' % i)
                        for i in range(6)]

        self.boundary = SpawnBoundary([Shape('boundary')])
        self.register_graspable_objects([plate])
        cond_set = ConditionSet([DetectedCondition(plate, self.success_sensor),
                                 NothingGrasped(self.robot.gripper)],
                                order_matters=True)
        self.register_success_conditions([cond_set])

    def init_episode(self, index: int) -> List[str]:
        color_name, _ = colors[index]

        shuffled_pillar_indexes = list(range(3))
        np.random.shuffle(shuffled_pillar_indexes)

        # Color the other spokes
        color_choices = [index] + list(np.random.choice(
            list(range(index)) + list(range(index + 1, len(colors))),
            size=2, replace=False))
        for pillar_i, color_i in zip(shuffled_pillar_indexes, color_choices):
            _, rgb = colors[color_i]
            self.pillars[pillar_i * 2].set_color(rgb)
            self.pillars[1 + pillar_i * 2].set_color(rgb)

        # Move the success detector to the correct spot
        success_pos = self.success_poses[shuffled_pillar_indexes[0]]
        self.success_sensor.set_position(success_pos.get_position())
        self.success_sensor.set_orientation(success_pos.get_orientation())

        self.boundary.clear()
        self.boundary.sample(
            self.plate_stand, min_rotation=(0, 0, 0.5),
            max_rotation=(0, 0, 0.5))
        self.boundary.sample(self.dish_rack, min_distance=0.2)

        return ['put the plate between the %s pillars of the dish rack'#
                % color_name,
                'place the plate in the the %s section of the drying rack'
                % color_name,
                'pick up the plate and leave it between the %s spokes on the '
                'dish rack' % color_name]


    def variation_count(self) -> int:
        return len(colors)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, 0], [0, 0, 0]
