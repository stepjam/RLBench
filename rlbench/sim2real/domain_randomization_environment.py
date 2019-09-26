from os.path import join
from pyrep import PyRep
from pyrep.objects.dummy import Dummy
from pyrep.objects.shape import Shape
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper

from rlbench.action_modes import ActionMode
from rlbench.backend.const import *
from rlbench.environment import Environment
from rlbench.environment import DIR_PATH
from rlbench.observation_config import ObservationConfig
from rlbench.backend.robot import Robot
from rlbench.sim2real.domain_randomization import RandomizeEvery
from rlbench.sim2real.domain_randomization_scene import DomainRandomizationScene


class DomainRandomizationEnvironment(Environment):
    """Each environment has a scene."""

    def __init__(self,
                 action_mode: ActionMode, dataset_root='',
                 obs_config=ObservationConfig(), headless=False,
                 static_positions: bool=False,
                 randomize_every: RandomizeEvery=RandomizeEvery.EPISODE,
                 frequency: int=1,
                 visual_randomization_config=None,
                 dynamics_randomization_config=None):
        super().__init__(
            action_mode, dataset_root, obs_config, headless, static_positions)
        self._randomize_every = randomize_every
        self._frequency = frequency
        self._visual_rand_config = visual_randomization_config
        self._dynamics_rand_config = dynamics_randomization_config

    def launch(self):
        if self._pyrep is not None:
            raise RuntimeError('Already called launch!')
        self._pyrep = PyRep()
        self._pyrep.launch(join(DIR_PATH, TTT_FILE), headless=self._headless)
        self._pyrep.set_simulation_timestep(0.005)

        self._robot = Robot(Panda(), PandaGripper())
        self._scene = DomainRandomizationScene(
            self._pyrep, self._robot, self._obs_config, self._randomize_every,
            self._frequency,self._visual_rand_config, self._dynamics_rand_config
        )
        self._set_arm_control_action()
        # Raise the domain randomized floor.
        Shape('Floor').set_position(Dummy('FloorAnchor').get_position())
