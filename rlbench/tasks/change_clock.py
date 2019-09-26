from typing import List, Tuple
import numpy as np
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped

NUM_TARGET_TIMES = 3


class ChangeClock(Task):

    def init_task(self) -> None:
        self.needle_hour = Shape('clock_needle_hour')
        self.needle_minute = Shape('clock_needle_minute')
        self.needle_crank = Shape('clock_needle_crank')
        self.needle_pivot = Shape('clock_needle_pivot')

        self.register_graspable_objects([self.needle_crank])

        _, _, self.needle_crank_starting_gamma = \
            self.needle_crank.get_orientation()
        self.hour_starting_ori = \
            self.needle_hour.get_orientation(relative_to=self.needle_pivot)
        self.minute_starting_ori = \
            self.needle_minute.get_orientation(relative_to=self.needle_pivot)

        self.target_rotation_dic = {0: (1 / 24) * np.pi,
                                    1: (2 / 24) * np.pi,
                                    2: (59 / 24) * np.pi}

        self.success_conditions = [NothingGrasped(self.robot.gripper)]

    def init_episode(self, index: int) -> List[str]:
        self.target_time_index = index
        target_times_dic = {0: '12:15', 1: '12:30', 2: '14:45'}
        target_time_str = target_times_dic[self.target_time_index]

        success_detector_lst = [
            ProximitySensor('detector_hour%d' % self.target_time_index),
            ProximitySensor('detector_minute%d' % self.target_time_index)
        ]

        while len(self.success_conditions) > 1:
            self.success_conditions.pop()

        self.success_conditions += \
            [DetectedCondition(self.needle_hour, success_detector_lst[0]),
             DetectedCondition(self.needle_minute, success_detector_lst[1])]

        self.register_success_conditions(self.success_conditions)
        self.w2 = Dummy('waypoint2')
        w2_starting_ori = self.w2.get_orientation(
            relative_to=self.needle_pivot)
        w2_target_ori = w2_starting_ori * 6
        w2_target_ori[2] -= self.target_rotation_dic[self.target_time_index]
        self.w2.set_orientation(w2_target_ori, relative_to=self.needle_pivot,
                                reset_dynamics=False)
        _, _, w2_set_gamma = self.w2.get_orientation(
            relative_to=self.needle_pivot)
        self.w2_gamma_delta = w2_set_gamma - w2_starting_ori[2]

        self.current_15_angle = 0
        self.total_rotation_min = 0
        self.lots_of_15 = 0
        self.lock_updates = 0

        return ['change the clock to show time %s' %target_time_str,
                'adjust the time to %s' %target_time_str,
                'change the clock to %s' %target_time_str,
                'set the clock to %s' %target_time_str,
                'turn the knob on the back of the clock until the time shows %s'
                %target_time_str,
                'rotate the wheel on the clock to make it show %s'
                %target_time_str,
                'make the clock say %s' %target_time_str]

    def variation_count(self) -> int:
        return NUM_TARGET_TIMES

    def step(self) -> None:
        _, _, current_gamma = self.needle_crank.get_orientation()
        crank_gamma_delta = \
            current_gamma - self.needle_crank_starting_gamma

        new_hour_ori = self.hour_starting_ori.copy()
        new_min_ori = self.minute_starting_ori.copy()

        if ((self.target_rotation_dic[self.target_time_index] % (
                2 * np.pi)) - 0.05) \
                <= -crank_gamma_delta <= \
                ((self.target_rotation_dic[self.target_time_index] % (
                        2 * np.pi)) + 0.05):
            new_hour_ori[1] -= \
                self.target_rotation_dic[self.target_time_index]
            new_min_ori[1] -= (12) * \
                              self.target_rotation_dic[self.target_time_index]
        else:
            new_hour_ori[1] += 12 * crank_gamma_delta / 12
            new_min_ori[1] += 12 * crank_gamma_delta

        self.needle_hour.set_orientation(new_hour_ori,
                                         relative_to=self.needle_pivot,
                                         reset_dynamics=False)
        self.needle_minute.set_orientation(new_min_ori,
                                           relative_to=self.needle_pivot,
                                           reset_dynamics=False
                                           )

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0], [0, 0, +0.4 * np.pi]
