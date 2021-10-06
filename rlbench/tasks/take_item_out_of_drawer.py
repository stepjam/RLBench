from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import DetectedCondition
from rlbench.backend.task import Task


class TakeItemOutOfDrawer(Task):

    def init_task(self) -> None:
        self._options = ['bottom', 'middle', 'top']
        self._anchors = [Dummy('waypoint_anchor_%s' % opt)
                         for opt in self._options]
        self._joints = [Joint('drawer_joint_%s' % opt)
                        for opt in self._options]
        self._waypoint1 = Dummy('waypoint1')
        self._item = Shape('item')
        self.register_graspable_objects([self._item])
        self.register_success_conditions(
            [DetectedCondition(self._item, ProximitySensor('success'))])

    def init_episode(self, index: int) -> List[str]:
        option = self._options[index]
        anchor = self._anchors[index]
        self._waypoint1.set_position(anchor.get_position())
        _, _, z_target = anchor.get_position()
        x, y, z = self._item.get_position()
        self._item.set_position([x, y, z_target])
        return ['take item out of the %s drawer' % option,
                'open the %s drawer and take the cube out' % option,
                'grasp the %s handle on the drawers, pull the drawer open, and'
                ' take out what you find inside' % option,
                'clear out the %s drawer' % option,
                'make sure the %s drawer is empty' % option,
                'lift the block out of the %s drawer' % option,
                'using the handle, open the %s drawer, and remove any items #'
                'inside' % option]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, - np.pi / 8], [0, 0, np.pi / 8]
