from typing import List
import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped

MAX_CHECKERS_TO_SETUP = 3


class SetupCheckers(Task):

    def init_task(self) -> None:
        self.checkers_placed = -1
        self.checkers = [Shape('checker%d' % i)
                         for i in range(24)]

        self.chess_board = Shape('chess_board_base')

        self.checkers_starting_pos_list = [
            checker.get_position(self.chess_board) for checker in self.checkers]
        self.checkers_starting_orientation = \
            self.checkers[0].get_orientation(self.chess_board)

        self.register_graspable_objects(self.checkers)
        self.success_detectors = [ProximitySensor('detector%d' % i)
                                  for i in range(24)]
        self.success_conditions = [NothingGrasped(self.robot.gripper)]
        for i in range(len(self.checkers)):
            self.success_conditions.append(
                DetectedCondition(self.checkers[i], self.success_detectors[i]))

        self.register_success_conditions(self.success_conditions)
        self.cleanup()

    def init_episode(self, index: int) -> List[str]:
        self.cleanup()

        self.checkers_to_setup = 1 + index % MAX_CHECKERS_TO_SETUP

        ## Use to showcase episode variations while rlbench backend gets fixed:

        self.register_waypoint_ability_start(0, self._move_above_next_target)
        self.register_waypoints_should_repeat(self._repeat)

        target_checkers_unordered = np.random.choice(self.checkers,
                                                     self.checkers_to_setup,
                                                     replace=False)
        self.target_indexes = []
        self.target_checkers = []
        for checker in self.checkers:
            if checker in target_checkers_unordered:
                self.target_checkers.append(checker)

        self.target_pos_list = []

        pos_delta = 0.045
        start_pos_list = [+0.0070816, +0.31868, -0.094908]

        for i, checker in enumerate(self.checkers):
            if checker in self.target_checkers:
                self.target_indexes.append(i)
                self.target_pos_list = checker.get_position()
                checker.set_position(start_pos_list,
                                     relative_to=self.chess_board,
                                     reset_dynamics=False)
                start_pos_list[2] += pos_delta

        if self.checkers_to_setup == 1:
            rtn = ['place the remaining checker '
                   'in its initial position on the board']
        else:
            rtn = ['place the %d remaining checkers in '
                   'their initial positions on the board'
                   % self.checkers_to_setup]

        rtn.extend(['prepare the checkers board',
                    'get the chess board ready for a game of checkers',
                    'setup the checkers board', 'setup checkers',
                    'arrange the checkers ready for a game',
                    'get checkers ready'])

        return rtn

    def variation_count(self) -> int:
        return MAX_CHECKERS_TO_SETUP

    def cleanup(self) -> None:
        self.checkers_placed = -1
        for i, checker in enumerate(self.checkers):
            checker.set_position(self.checkers_starting_pos_list[i],
                                 self.chess_board)
            checker.set_orientation(self.checkers_starting_orientation,
                                    self.chess_board)

    def _move_above_next_target(self, waypoint):
        self.checkers_placed += 1
        self.target_index = self.target_indexes[self.checkers_placed]

        if self.checkers_placed > self.checkers_to_setup:
            raise RuntimeError('Should not be here')

        w1 = Dummy('waypoint1')
        # self.w2.set_parent(target_checkers[self.checkers_placed]

        unplaced_x, unplaced_y, unplaced_z = self.target_checkers[
            self.checkers_placed].get_position()

        z_offset_pickup = 0

        w1.set_position([unplaced_x, unplaced_y, unplaced_z - z_offset_pickup],
                        reset_dynamics=False)

        w4 = Dummy('waypoint4')

        target_x, target_y, target_z = self.checkers_starting_pos_list[
            self.target_index]
        z_offset_placement = 1.00 * 10 ** (-3)

        w4.set_position([target_x - z_offset_placement, target_y, target_z],
                        relative_to=self.chess_board, reset_dynamics=False)

        if self.checkers_to_setup > 1:
            if self.target_index == self.target_indexes[0]:
                self.target_index = self.target_indexes[1]
            if self.target_index == self.target_indexes[1] \
                    and self.checkers_to_setup == 3:
                self.target_index = self.target_indexes[2]

    def _repeat(self):
        return self.checkers_placed + 1 < self.checkers_to_setup
