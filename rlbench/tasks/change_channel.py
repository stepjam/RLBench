from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition, DetectedCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np


class ChangeChannel(Task):

    def init_task(self) -> None:
        self.tv = Shape('tv_frame')
        self.screen_on = Shape('tv_screen_on')
        self.screen_up = Shape('tv_screen_up')
        self.screen_down = Shape('tv_screen_down')
        self.remote = Shape('tv_remote')
        self.boundary = Shape('spawn_boundary')
        self.joint_conditions = [
            JointCondition(Joint('target_button_joint1'), 0.003),
            JointCondition(Joint('target_button_joint2'), 0.003)
        ]

    def init_episode(self, index: int) -> List[str]:
        detectors = [ProximitySensor('success%d' % i) for i in range(2)]
        self.register_success_conditions([self.joint_conditions[index],
                                          DetectedCondition(self.remote,
                                                            detectors[0]),
                                          DetectedCondition(self.remote,
                                                            detectors[1])
                                          ])
        self.register_graspable_objects([self.remote])
        w6 = Dummy('waypoint6')
        _, _, z = w6.get_position(
            relative_to=self.remote)
        rel_pos_lst = [
            Shape('target_button_wrap1').get_position(relative_to=self.remote),
            Shape('target_button_wrap2').get_position(relative_to=self.remote)
        ]
        rel_pos_lst[index % 2][2] = z
        w6.set_position(rel_pos_lst[index], relative_to=self.remote,
                        reset_dynamics=False)
        b = SpawnBoundary([self.boundary])
        b.sample(self.remote)
        btn = {0: 'plus', 1: 'minus'}
        chnl = {0: 'up', 1: 'minus'}
        return ['turn the channel %s' % chnl[index],
                'change the television channel %s' % chnl[index],
                'point the remote at the tv and press the %s button to turn '
                'the channel %s' % (btn[index], chnl[index]),
                'using the tv remote, ensure it is facing the television and '
                'press the %s button to increment the channel %s by one'
                % (btn[index], chnl[index]),
                'find the %s button on the remote, rotate the remote such that'
                ' it is pointed at the tv, then press the button to change '
                'the channel %s' % (chnl[index], btn[index])]

    def variation_count(self) -> int:
        return 2

    def step(self) -> None:
        if self.joint_conditions[0].condition_met() == (True, True):
            self.screen_on.set_position(
                [9.9959*10**(-3), -2.3971*10**(-2), 5.2193*10**(-4)],
                relative_to=self.tv,
                reset_dynamics=False)
            self.screen_up.set_position(
                [3.8501*10**(-3), -2.3971*10**(-2), +5.226*10**(-4)],
                relative_to=self.tv,
                reset_dynamics=False
            )
        elif self.joint_conditions[1].condition_met() == (True, True):
            self.screen_on.set_position(
                [9.9959*10**(-3), -2.3971*10**(-2), 5.2193*10**(-4)],
                relative_to=self.tv,
                reset_dynamics=False)
            self.screen_down.set_position(
                [3.8501*10**(-3), -2.3971*10**(-2), +5.226*10**(-4)],
                relative_to=self.tv,
                reset_dynamics=False
            )

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -0.5*np.pi), (0.0, 0.0, +0.5 * np.pi)
