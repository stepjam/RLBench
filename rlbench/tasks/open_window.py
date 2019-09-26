from typing import List, Tuple
import numpy as np
from pyrep.objects.object import Object
from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition

OPTIONS = ['left', 'right']


class OpenWindow(Task):

    def init_task(self):
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        self.waypoint3 = Dummy('waypoint3')

        self.waypoint0_ = Dummy('waypoint0_')
        self.waypoint1_ = Dummy('waypoint1_')
        self.waypoint2_ = Dummy('waypoint2_')
        self.waypoint3_ = Dummy('waypoint3_')

        self.right_unlocked_cond = JointCondition(
            Joint('right_handle_joint'), np.deg2rad(60))
        self.left_unlocked_cond = JointCondition(
            Joint('left_handle_joint'), np.deg2rad(60))
        self.right_frame = Shape('right_frame')
        self.left_frame = Shape('left_frame')

    def init_episode(self, index: int) -> List[str]:
        self.right_frame.set_dynamic(False)
        self.left_frame.set_dynamic(False)
        self.left_unlocked = False
        self.right_unlocked = False

        option = OPTIONS[index]

        if option == 'right':
            self.waypoint0.set_position(self.waypoint0_.get_position())
            self.waypoint1.set_position(self.waypoint1_.get_position())
            self.waypoint2.set_position(self.waypoint2_.get_position())
            self.waypoint2.set_orientation(self.waypoint2_.get_orientation())
            self.waypoint3.set_position(self.waypoint3_.get_position())
            self.waypoint3.set_orientation(self.waypoint3_.get_orientation())
            self.register_success_conditions([JointCondition(
                Joint('right_window_joint'), np.deg2rad(50))])
        else:
            self.register_success_conditions([JointCondition(
                Joint('left_window_joint'), np.deg2rad(50))])
        return ['open %s window' % option,
                'rotate the handle to unlock the %s window, then open it' % option,
                'push the %s window open' % option,
                'use the handle to open the %s window' % option]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        if not self.left_unlocked:
            self.left_unlocked = self.left_unlocked_cond.condition_met()[0]
            if self.left_unlocked:
                self.left_frame.set_dynamic(True)

        if not self.right_unlocked:
            self.right_unlocked = self.right_unlocked_cond.condition_met()[0]
            if self.right_unlocked:
                self.right_frame.set_dynamic(True)

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
