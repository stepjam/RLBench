from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

UNIQUE_PEPPERS_TO_TAKE_OFF = 3


class TakeOffWeighingScales(Task):

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
        self.success_conditions = [NothingGrasped(self.robot.gripper)]
        self.w0 = Dummy('waypoint0')
        self.w0_rel_pos = self.w0.get_position(relative_to=self.peppers[0])
        self.w0_rel_ori = self.w0.get_orientation(relative_to=self.peppers[0])

    def init_episode(self, index: int) -> List[str]:
        self._variation_index = index
        self.target_pepper_index = index
        while len(self.success_conditions) > 1:
            self.success_conditions.pop()
        self.success_conditions.append(DetectedCondition(
            self.peppers[self.target_pepper_index], self.success_detector))
        self.register_success_conditions(self.success_conditions)
        b = SpawnBoundary([self.boundary])
        for p in self.peppers:
            b.sample(p, ignore_collisions=False, min_distance=0.09,
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
        index_dic = {0: 'green', 1: 'red', 2: 'yellow'}
        if self.target_pepper_index in range(3):
            return [
                'remove the %s pepper from the weighing scales and place it '
                'on the table' % index_dic[self.target_pepper_index],
                'take the %s pepper off of the scales'
                % index_dic[self.target_pepper_index],
                'lift the %s pepper off of the tray and set it down on the '
                'table' % index_dic[self.target_pepper_index],
                'grasp the %s pepper and move it to the table top'
                % index_dic[self.target_pepper_index],
                'take the %s object off of the scales tray'
                % index_dic[self.target_pepper_index],
                'put the %s item on the item'
                % index_dic[self.target_pepper_index]]
        else:
            raise ValueError('Invalid pepper index, should not be here')

    def variation_count(self) -> int:
        return UNIQUE_PEPPERS_TO_TAKE_OFF

    def step(self) -> None:
        _, _, pos_z = self.top_plate.get_position()
        dz = self.top_plate_starting_z - pos_z
        d_alpha = -130 * dz
        new_needle_ori = [self.needle_starting_ori[0] + d_alpha,
                          self.needle_starting_ori[1],
                          self.needle_starting_ori[2]]

        self.needle.set_orientation(new_needle_ori,
                                    relative_to=self.needle_pivot,
                                    reset_dynamics=False)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -0.25 * np.pi], [0.0, 0.0, +0.25 * np.pi]
