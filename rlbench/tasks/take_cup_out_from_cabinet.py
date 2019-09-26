from typing import List, Tuple
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped

OPTIONS = ['left', 'right']


class TakeCupOutFromCabinet(Task):

    def init_task(self) -> None:
        self.cup = Shape('cup')
        self.left_placeholder = Dummy('left_cup_placeholder')
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint2 = Dummy('waypoint2')
        self.left_way_placeholder1 = Dummy('left_way_placeholder1')
        self.left_way_placeholder2 = Dummy('left_way_placeholder2')

        self.register_success_conditions(
            [DetectedCondition(
                self.cup, ProximitySensor('success'), negated=True),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        option = OPTIONS[index]

        if option == 'left':
            self.waypoint1.set_position(
                self.left_way_placeholder1.get_position())
            self.waypoint2.set_position(
                self.left_way_placeholder2.get_position())
            self.cup.set_position(self.left_placeholder.get_position())

        return ['take out a cup from the %s half of the cabinet' % option,
                'open the %s side of the cabinet and get the cup'
                % option,
                'grasping the %s handle, open the cabinet, then retrieve the '
                'cup' % option,
                'slide open the %s door on the cabinet and put take the cup out'
                % option,
                'remove the cup from the %s part of the cabinet' % option]

    def variation_count(self) -> int:
        return 2

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -3.14/2], [0.0, 0.0, 3.14/2]
