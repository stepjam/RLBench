from typing import List, Tuple

import numpy as np
from pyrep.objects.dummy import Dummy
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import JointCondition, DetectedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.task import Task


class ChangeChannel(Task):

    def init_task(self) -> None:
        self._remote = Shape('tv_remote')
        self.register_graspable_objects([self._remote])
        self._joint_conditions = [
            JointCondition(Joint('target_button_joint1'), 0.003),
            JointCondition(Joint('target_button_joint2'), 0.003)
        ]
        self._remote_conditions = [
            DetectedCondition(Dummy('tv_remote_top'),
                              ProximitySensor('success0')),
            DetectedCondition(Dummy('tv_remote_bottom'),
                              ProximitySensor('success1'))
        ]
        self._spawn_boundary = SpawnBoundary([Shape('spawn_boundary')])
        self._target_buttons = [
            Shape('target_button_wrap1'),
            Shape('target_button_wrap2')]
        self._w6 = Dummy('waypoint6')
        self._w6z = self._w6.get_position()[2]

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions(
            [self._joint_conditions[index]] + self._remote_conditions)
        x, y, _ = self._target_buttons[index % 3].get_position()
        self._w6.set_position([x, y, self._w6z])
        self._spawn_boundary.clear()
        self._spawn_boundary.sample(self._remote)

        btn = ['plus', 'minus']
        chnl = ['up', 'minus']
        return [
            'turn the channel %s' % chnl[index - 1],
            'change the television channel %s' % chnl[index - 1],
            'point the remote at the tv and press the %s button to turn '
            'the channel %s' % (btn[index - 1], chnl[index - 1]),
            'using the tv remote, ensure it is facing the television and '
            'press the %s button to increment the channel %s by one'
            % (btn[index - 1], chnl[index - 1]),
            'find the %s button on the remote, rotate the remote such that'
            ' it is pointed at the tv, then press the button to change '
            'the channel %s' % (chnl[index - 1], btn[index - 1])]

    def variation_count(self) -> int:
        return 2

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -np.pi / 2), (0.0, 0.0, np.pi / 2)
