from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary

NUM_SHELVES_IN_SAFE = 3


class TakeMoneyOutSafe(Task):

    def init_task(self) -> None:
        self.shelf = {0: 'bottom', 1: 'middle', 2: 'top'}
        self.shelf_alt = {0: 'first', 1: 'second', 2: 'third'}
        self.money_list = [Shape('dollar_stack%d' % i) for i in range(3)]
        self.register_graspable_objects(self.money_list)
        self.success_conditions = [NothingGrasped(self.robot.gripper)]

        self.w1 = Dummy('waypoint1')
        self.w1_rel_pos = [-2.7287 * 10 ** (-4), -2.3246 * 10 ** (-6),
                           +4.5627 * 10 ** (-2)]
        self.w1_rel_ori = [-3.1416, 7.2824 * 10 ** (-1), -2.1265 * 10 ** (-2)]
        self.w3 = Dummy('waypoint3')
        self.place_boundary = Shape('placement_boundary')
        self.pick_boundary = Shape('money_boundary')
        self.safe = Shape('safe_body')
        self.money_ori = self.money_list[0].get_orientation()

    def init_episode(self, index: int) -> List[str]:
        self.target_shelf = index
        target_dummy_name = 'dummy_shelf' + str(self.target_shelf)
        target_pos_dummy = Dummy(target_dummy_name)
        target_pos = target_pos_dummy.get_position()
        target_ori = target_pos_dummy.get_orientation()
        self.w1.set_position(target_pos, reset_dynamics=False)
        self.w1.set_orientation(target_ori, reset_dynamics=False)
        b_place = SpawnBoundary([self.place_boundary])
        b_place.sample(self.w3, min_rotation=(0.0, 0.0, -np.pi / 12),
                       max_rotation=(0.0, 0.0, +np.pi / 12), min_distance=0)
        self.success_detector = ProximitySensor('success_detector')
        while len(self.success_conditions) > 1:
            self.success_conditions.pop()
        self.success_conditions.append(
            DetectedCondition(self.money_list[self.target_shelf],
                              self.success_detector)
        )
        self.register_success_conditions(self.success_conditions)
        # self.w1.set_parent(target_pos_dummy, keep_in_place=True)
        rel_target_pos = self.w1.get_position(
            relative_to=self.money_list[index])
        rel_target_ori = self.w1.get_orientation(
            relative_to=self.money_list[index]
        )
        b_pick = SpawnBoundary([self.pick_boundary])
        b_pick.sample(self.money_list[index],
                      min_rotation=(0.0, 0.0, -np.pi / 10),
                      max_rotation=(0.0, 0.0, 0.0), min_distance=0)

        for m in self.money_list:
            b_pick.sample(m,
                          min_rotation=(0.0, 0.0, -np.pi/10),
                          max_rotation=(0.0, 0.0, 0.0), min_distance=0)

        self.w1.set_position(rel_target_pos, relative_to=self.money_list[index],
                             reset_dynamics=False)
        self.w1.set_orientation(rel_target_ori,
                                relative_to=self.money_list[index],
                                reset_dynamics=False)

        """
        if self.target_shelf == 0:
            return ['take the money out of the bottom shelf and place it on '
                    'the table']
        elif self.target_shelf == 1:
            return ['take the money out of the middle shelf and place it on '
                    'the table']
        elif self.target_shelf == 2:
            return ['take the money out of the top shelf and place it on '
                    'the table']
        else:
            raise ValueError('Invalid target_shelf index, should not be here')
            return -1
        """

        return ['take the money out of the %s shelf and place it on the '
                'table' % self.shelf[index],
                'take the stack of money out of the %s shelf and place it on '
                'the table' % self.shelf_alt[index],
                'take one of the stacks of money out of the %s bit of the safe'
                % self.shelf[index],
                'take the money off of the %s shelf of the safe'
                % self.shelf_alt[index],
                'grasp the bank notes in the %s and remove it from the safe'
                % self.shelf[index],
                'get the money from the %s shelf' % self.shelf_alt[index],
                'retrieve the stack of bank notes from the %s of the safe'
                % self.shelf[index],
                'locate the money on the %s shelf, grasp it, remove if from the'
                ' safe, and set it on the table' % self.shelf[index],
                'put the money on the %s shelf down on the table'
                % self.shelf_alt[index]]

    def variation_count(self) -> int:
        return NUM_SHELVES_IN_SAFE

    def cleanup(self) -> None:
        for m in self.money_list:
            m.set_orientation(self.money_ori, reset_dynamics=False)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return (0.0, 0.0, -0.5 * np.pi), (0.0, 0.0, +0.5 * np.pi)
