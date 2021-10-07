from typing import List, Tuple
import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from rlbench.backend.conditions import JointCondition
from rlbench.backend.task import Task


class OpenDrawer(Task):

    def init_task(self) -> None:
        self._options = ['bottom', 'middle', 'top']
        self._anchors = [Dummy('waypoint_anchor_%s' % opt)
                         for opt in self._options]
        self._joints = [Joint('drawer_joint_%s' % opt)
                        for opt in self._options]
        self._waypoint1 = Dummy('waypoint1')

    def init_episode(self, index: int) -> List[str]:
        option = self._options[index]
        self._waypoint1.set_position(self._anchors[index].get_position())
        self.register_success_conditions(
            [JointCondition(self._joints[index], 0.15)])
        return ['open %s drawer' % option,
                'grip the %s handle and pull the %s drawer open' % (
                    option, option),
                'slide the %s drawer open' % option]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
