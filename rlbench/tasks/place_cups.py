from typing import List, Tuple
import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

MAX_CUPS_TO_PLACE = 3


class PlaceCups(Task):

    def init_task(self) -> None:
        self.cups_placed = -1
        self.cups = [Shape('mug%d' % i) for i in range(3)]
        self.spokes = [Shape('place_cups_holder_spoke%d' % i) for i in range(3)]
        self.holder = Shape('place_cups_holder_base')
        self.cups_boundary = Shape('mug_boundary')
        self.register_graspable_objects(self.cups)
        self.success_detectors = [ProximitySensor('success_detector%d' % i) for
                                  i in range(3)]
        self.success_conditions = [NothingGrasped(self.robot.gripper)]

        self.w1 = Dummy('waypoint1')
        self.w1_rel_pos = [-1.3411 * 10 ** (-7), -1.6046 * 10 ** (-2),
                           +1.9247 * 10 ** (-2)]
        self.w1_rel_ori = [+3.1416, +1.4807 * 10 ** (-7), +2.6180]

    def init_episode(self, index: int) -> List[str]:
        self.cups_placed = -1
        self.cups_to_place = 1 + index % MAX_CUPS_TO_PLACE
        self.w1.set_position(self.w1_rel_pos,
                             relative_to=self.cups[0],
                             reset_dynamics=False)
        self.w1.set_orientation(self.w1_rel_ori,
                                relative_to=self.cups[0],
                                reset_dynamics=False)
        b = SpawnBoundary([self.cups_boundary])
        for c in self.cups:
            b.sample(c, ignore_collisions=False, min_distance=0.10,
                     min_rotation=(0.00, 0.00, -3.14),
                     max_rotation=(0.00, 0.00, +3.14)
                     )
            if c == self.cups[0]:
                self.w1.set_position(self.w1_rel_pos,
                                     relative_to=c,
                                     reset_dynamics=False)
                self.w1.set_orientation(self.w1_rel_ori,
                                        relative_to=c,
                                        reset_dynamics=False)
        for i in range(self.cups_to_place):
            self.success_conditions.append(
                DetectedCondition(self.cups[i], self.success_detectors[i])
            )
        self.register_success_conditions(self.success_conditions)
        self.register_waypoint_ability_start(
            0, self._move_above_next_target)
        self.register_waypoints_should_repeat(self._repeat)

        if self.cups_to_place == 1:
            return ['place 1 cup on the cup holder'
                    'pick up one cup and put it on the mug tree',
                    'move 1 mug from the table to the cup holder',
                    'pick up one cup and slide its handle onto a spoke on the '
                    'mug holder']
        else:
            return ['place %d cups on the cup holder' % self.cups_to_place,
                    'pick up %d cups and place them on the holder'
                    % self.cups_to_place,
                    'move %d cups from the table to the mug tree'
                    % self.cups_to_place,
                    'pick up %d mugs and slide their handles onto the cup '
                    'holder spokes' % self.cups_to_place]

    def variation_count(self) -> int:
        return MAX_CUPS_TO_PLACE

    def _move_above_next_target(self, waypoint):
        if self.cups_placed > self.cups_to_place:
            raise RuntimeError('Should not be here, all cups should have been '
                               'placed')
        move_index = self.cups_placed if self.cups_placed > -1 else 0
        next_move_index = self.cups_placed + 1 if self.cups_placed > -1 else 0
        if self.cups_placed > -1:
            w4 = Dummy('waypoint4')
            w4_x, w4_y, w4_z = w4.get_position(
                relative_to=self.spokes[move_index]
            )
            w4_alpha, w4_beta, w4_gamma = w4.get_orientation(
                relative_to=self.spokes[move_index]
            )
            self.w1.set_position(self.w1_rel_pos,
                                 relative_to=self.cups[next_move_index],
                                 reset_dynamics=False)
            self.w1.set_orientation(self.w1_rel_ori,
                                    relative_to=self.cups[next_move_index],
                                    reset_dynamics=False)
            w4.set_position([w4_x, w4_y, w4_z],
                            relative_to=self.spokes[next_move_index],
                            reset_dynamics=False)
            w4.set_orientation([w4_alpha, w4_beta, w4_gamma],
                               relative_to=self.spokes[next_move_index],
                               reset_dynamics=False)
        ######
        self.cups_placed += 1
        ######

    def _repeat(self):
        return self.cups_placed < self.cups_to_place - 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -0.5 * np.pi], [0.0, 0.0, +0.5 * np.pi]
