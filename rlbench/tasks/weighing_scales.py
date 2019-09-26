from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

UNIQUE_PEPPERS_TO_PLACE = 3


class WeighingScales(Task):

    def init_task(self) -> None:
        self.needle = Shape('scales_meter_needle')
        self.needle_pivot = Shape('scales_meter_pivot')
        self.top_plate = Shape('scales_tray_visual')
        self.joint = Joint('scales_joint')

        _, _, starting_z = self.top_plate.get_position()
        self.top_plate_starting_z = starting_z
        self.needle_starting_ori = self.needle.get_orientation(
            relative_to=self.needle_pivot)

        self.peppers = [Shape('pepper%d' % i) for i in range(3)]
        self.register_graspable_objects(self.peppers)

        self.boundary = Shape('peppers_boundary')

        self.success_detector = ProximitySensor('success_detector')
        self.needle_detector = ProximitySensor('needle_sensor')
        self.success_conditions = [NothingGrasped(self.robot.gripper),
                                   DetectedCondition(
                                       self.needle, self.needle_detector)
                                   ]

        self.w0 = Dummy('waypoint0')
        self.w0_rel_pos = [+2.6203 * 10 ** (-3), -1.7881 * 10 ** (-7),
                           +1.5197 * 10 ** (-1)]
        self.w0_rel_ori = [-3.1416, -1.7467 * 10 ** (-2), -3.1416]

    def init_episode(self, index: int) -> List[str]:
        self.target_pepper_index = index
        while len(self.success_conditions) > 2:
            self.success_conditions.pop()
        self.success_conditions.append(DetectedCondition(
            self.peppers[self.target_pepper_index], self.success_detector))
        self.register_success_conditions(self.success_conditions)
        b = SpawnBoundary([self.boundary])
        for p in self.peppers:
            b.sample(p, ignore_collisions=False, min_distance=0.12,
                     min_rotation=(0.00, 0.00, -3.14),
                     max_rotation=(0.00, 0.00, +3.14)
                     )

        self.w0.set_position(self.w0_rel_pos,
                             relative_to=self.peppers[
                                 self.target_pepper_index],
                             reset_dynamics=False)
        self.w0.set_orientation(self.w0_rel_ori,
                                relative_to=self.peppers[
                                    self.target_pepper_index],
                                reset_dynamics=False)
        pepper = {0: 'green', 1: 'red', 2: 'yellow'}

        return ['weigh the %s pepper' % pepper[index],
                'pick up the %s pepper and set it down on the weighing scales'
                % pepper[index],
                'put the %s pepper on the scales' % pepper[index],
                'place the %s pepper onto the scales tray' % pepper[index],
                'lift up the %s pepper onto the weighing scales'
                % pepper[index],
                'grasp the %s pepper from above and lower it onto the tray'
                % pepper[index]]

    def variation_count(self) -> int:
        return UNIQUE_PEPPERS_TO_PLACE

    def step(self):
        _, _, pos_z = self.top_plate.get_position()
        dz = self.top_plate_starting_z - pos_z
        d_alpha = -120 * dz
        new_needle_ori = [self.needle_starting_ori[0] + d_alpha,
                          self.needle_starting_ori[1],
                          self.needle_starting_ori[2]]

        self.needle.set_orientation(new_needle_ori,
                                    relative_to=self.needle_pivot,
                                    reset_dynamics=False)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -0.25 * np.pi], [0.0, 0.0, +0.25 * np.pi]
