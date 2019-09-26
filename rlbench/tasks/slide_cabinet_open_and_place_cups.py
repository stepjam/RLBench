from typing import List, Tuple

from pyrep.objects.joint import Joint
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, JointCondition, \
    NothingGrasped

OPTIONS = ['left', 'right']


class SlideCabinetOpenAndPlaceCups(Task):

    def init_task(self):

        self.cup = Shape('cup')
        self.success_sensor = ProximitySensor('success')
        self.register_graspable_objects([Shape('cup')])

        self.left_joint = Joint('left_joint')
        self.right_joint = Joint('right_joint')
        self.waypoint0 = Dummy('waypoint0')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        self.waypoint6 = Dummy('waypoint6')
        self.left_initial_waypoint = Dummy('left_initial_waypoint')
        self.left_close_waypoint = Dummy('left_close_waypoint')
        self.left_far_waypoint = Dummy('left_far_waypoint')
        self.place_cup_left_waypoint = Dummy('place_cup_left_waypoint')

        # self.register_success_conditions([
        #     DetectedCondition(sel, ProximitySensor('success'))])

    def init_episode(self, index: int) -> List[str]:

        option = OPTIONS[index]

        conditions = [DetectedCondition(self.cup, self.success_sensor),
                      NothingGrasped(self.robot.gripper)]

        if option == 'left':
            self.waypoint0.set_position(
                self.left_initial_waypoint.get_position())
            self.waypoint1.set_position(self.left_close_waypoint.get_position())
            self.waypoint2.set_position(self.left_far_waypoint.get_position())
            self.waypoint6.set_position(self.place_cup_left_waypoint.get_position())
            conditions.append(JointCondition(self.left_joint, 0.06))
        else:
            conditions.append(JointCondition(self.right_joint, 0.06))
        self.register_success_conditions(conditions)

        return ['put cup in %s cabinet' % option,
                'put the mug away in the %s half of the cabinet' % option,
                'open the %s side of the cabinet and put the cup away in it'
                % option,
                'grasping the %s handle, open the cabinet, then pick up the cup'
                ' and set it down inside the cabinet' % option,
                'slide open the %s door on the cabinet and put away the mug'
                % option]

    def variation_count(self) -> int:
        return 2

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -3.14 / 4.], [0.0, 0.0, 3.14 / 4.]
