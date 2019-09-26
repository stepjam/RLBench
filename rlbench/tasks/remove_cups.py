from typing import List, Tuple
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped

MAX_CUPS_TO_REMOVE = 2


class RemoveCups(Task):

    def init_task(self) -> None:
        self.cups_removed = -1
        self.cups = [Shape('mug%d' % i) for i in range(3)]
        self.spokes = [Shape('place_cups_holder_spoke%d' % i) for i in range(3)]
        self.holder = Shape('place_cups_holder_base')
        self.holder_boundary = Shape('tree_boundary')
        self.register_graspable_objects(self.cups)
        self.success_detectors = [ProximitySensor('success_detector%d' % i)
                                  for i in range(3)]
        self.w1 = Dummy('waypoint1')
        self.w1_rel_pos = self.w1.get_position(relative_to=self.cups[0])
        self.w1_rel_ori = self.w1.get_orientation(relative_to=self.cups[0])
        self.w2 = Dummy('waypoint2')
        self.w2_rel_pos = self.w2.get_position(relative_to=self.spokes[0])
        self.w2_rel_ori = self.w2.get_orientation(relative_to=self.spokes[0])
        self.w5 = Dummy('waypoint5')
        self.w5_rel_pos = self.w5.get_orientation(
            relative_to=self.success_detectors[0])
        self.w5_new_pos = self.w5.get_position()
        self.w5_new_pos_saved = self.w5_new_pos
        self.success_conditions = [NothingGrasped(self.robot.gripper)]

    def init_episode(self, index: int) -> List[str]:
        self.cups_removed = -1
        self.cups_to_remove = 1 + index % MAX_CUPS_TO_REMOVE
        self.w5_new_pos = self.w5_new_pos_saved
        self.w1.set_position(self.w1_rel_pos,
                             relative_to=self.cups[0],
                             reset_dynamics=False)
        for i in range(self.cups_to_remove):
            self.success_conditions.append(
                DetectedCondition(self.cups[i], self.success_detectors[i])
            )
        self.register_success_conditions(self.success_conditions)
        self.register_waypoint_ability_start(0, self._move_above_next_target)
        self.register_waypoints_should_repeat(self._repeat)

        if self.cups_to_remove == 1:
            return ['remove 1 cup from the cup holder and place it on the '
                    'table',
                    'remove one cup from the mug holder',
                    'pick up 1 cup from the mug tree and place it on the table',
                    'slide 1 mug off of its spoke on the cup holder and leave '
                    'it on the table top']
        else:
            return ['remove %d cups from the cup holder and place it on the '
                    'table' % self.cups_to_remove,
                    'remove %d cups from the cup holder' % self.cups_to_remove,
                    'pick up %d cups from the mug tree and place them on the '
                    'table' % self.cups_to_remove,
                    'slide %d mugs off of their spokes on the cup holder and '
                    'leave them on the table top' % self.cups_to_remove]

    def variation_count(self) -> int:
        return MAX_CUPS_TO_REMOVE

    def _move_above_next_target(self, waypoint):
        if self.cups_removed > self.cups_to_remove:
            raise RuntimeError('Should not be here, all cups should have been '
                               'removed')
        move_index = self.cups_removed if self.cups_removed > -1 else 0
        next_move_index = self.cups_removed + 1 if self.cups_removed > -1 else 0
        if self.cups_removed > -1:
            self.w1.set_position(self.w1_rel_pos,
                                 relative_to=self.cups[next_move_index],
                                 reset_dynamics=False
                                 )
            self.w1.set_orientation(self.w1_rel_ori,
                                    relative_to=self.cups[next_move_index],
                                    reset_dynamics=False
                                    )
            self.w2.set_position(self.w2_rel_pos,
                                 relative_to=self.spokes[next_move_index],
                                 reset_dynamics=False)
            self.w2.set_orientation(self.w2_rel_ori,
                                    relative_to=self.spokes[next_move_index],
                                    reset_dynamics=False
                                    )
            new_x, new_y, _ = self.success_detectors[
                next_move_index].get_position()
            self.w5_new_pos[0] = new_x
            self.w5_new_pos[1] = new_y
            self.w5.set_position(self.w5_new_pos,
                                 reset_dynamics=False)

        ######
        self.cups_removed += 1
        ######

    def cleanup(self) -> None:
        self.cups_removed = -1

    def _repeat(self):
        return self.cups_removed < self.cups_to_remove - 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
