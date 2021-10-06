from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class TvOn(Task):

    def init_task(self) -> None:
        self._remote = Shape('tv_remote')
        self.register_graspable_objects([self._remote])
        self.register_success_conditions([
            JointCondition(Joint('target_button_joint0'), 0.003),
            DetectedCondition(Dummy('tv_remote_top'),
                              ProximitySensor('success0')),
            DetectedCondition(Dummy('tv_remote_bottom'),
                              ProximitySensor('success1'))])
        self._spawn_boundary = SpawnBoundary([Shape('spawn_boundary')])

    def init_episode(self, index: int) -> List[str]:
        self._spawn_boundary.clear()
        self._spawn_boundary.sample(self._remote)
        return [
                'turn on the TV',
                'point the remote control at the television and turn on the '
                'television',
                'pick up the remote and rotate it such that the front of the '
                'remote is pointed straight at the television, then set the '
                'remote down and press the power button down in order to switch'
                ' on the TV',
                'find the power button at the top of the remote, ensure the '
                'remote is pointed at the tv, then turn the tv on']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi / 2), (0.0, 0.0, np.pi / 2)
