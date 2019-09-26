from typing import List, Tuple
from pyrep.objects.joint import Joint
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition, NothingGrasped

OPTIONS = ['right', 'left']


class SlideCabinetOpen(Task):

    def init_task(self):
        self.left_joint = Joint('left_joint')
        self.right_joint = Joint('right_joint')
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        self.left_initial_waypoint = Dummy('waypoint4')
        self.left_close_waypoint = Dummy('waypoint5')
        self.left_far_waypoint = Dummy('waypoint6')

    def init_episode(self, index: int) -> List[str]:
        option = OPTIONS[index]

        if option == 'left':
            self.waypoint0.set_position(
                self.left_initial_waypoint.get_position())
            self.waypoint1.set_position(self.left_close_waypoint.get_position())
            self.waypoint2.set_position(self.left_far_waypoint.get_position())
            self.register_success_conditions(
                [JointCondition(self.left_joint, 0.06),
                 NothingGrasped(self.robot.gripper)])
        else:
            self.register_success_conditions(
                [JointCondition(self.right_joint, 0.06),
                 NothingGrasped(self.robot.gripper)])

        return ['slide %s cabinet open' % option,
                'open the %s door' % option,
                'open the %s half of the cabinet' % option,
                'slide open the %s slide of the cabinet' % option,
                'grip the %s handle and slide the door open' % option,
                'grasp the %s door\'s handle and and drag it towards the middle'
                ' of the cabinet in order to slide that door open' % option]

    def variation_count(self) -> int :
        return 2

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -3.14 / 4.], [0.0, 0.0, 3.14 / 4.]
