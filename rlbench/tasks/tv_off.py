from typing import List, Tuple
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.spawn_boundary import SpawnBoundary
import numpy as np


class TvOff(Task):

    def init_task(self) -> None:
        self.tv = Shape('tv_frame')
        self.screen_off = Shape('tv_screen_off')
        self.remote = Shape('tv_remote')
        self.boundary = Shape('spawn_boundary')
        self.condition = JointCondition(Joint('target_button_joint0'), 0.003)

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions([self.condition])
        self.register_graspable_objects([self.remote])
        b = SpawnBoundary([self.boundary])
        b.sample(self.remote)
        return ['turn off the TV',
                'point the remote control at the television and turn off the '
                'television',
                'pick up the remote and rotate it such that the front of the '
                'remote is pointed straight at the television, then set the '
                'remote down and press the power button down in order to switch'
                ' off the TV',
                'find the power button at the top of the remote, ensure the '
                'remote is pointed at the tv, then turn the tv off']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if self.condition.condition_met() == (True, True):
            self.screen_off.set_position([4*10**(-3), -2.3597*10**(-2), 5.2194*10**(-4)],
                                        relative_to=self.tv,
                                        reset_dynamics=False)

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -0.5*np.pi), (0.0, 0.0, +0.5 * np.pi)
