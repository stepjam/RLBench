from typing import List, Tuple
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition


class OpenDrawer(Task):

    def init_task(self) -> None:
        self.options = ['bottom', 'middle', 'top']
        self.anchors = [Dummy('waypoint_anchor_%s' % opt)
                        for opt in self.options]
        self.joints = [Joint('drawer_joint_%s' % opt)
                       for opt in self.options]
        self.waypoint1 = Dummy('waypoint1')

    def init_episode(self, index: int) -> List[str]:
        option = self.options[index]
        self.waypoint1.set_position(self.anchors[index].get_position())
        self.register_success_conditions(
            [JointCondition(self.joints[index], 0.15)])
        return ['open %s drawer' % option,
                'grip the %s handle and pull the %s drawer open' % (
                    option, option),
                'slide the %s drawer open' % option]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]
