import unittest
from os import path

import numpy as np
from pyrep.const import RenderMode

from rlbench import Environment
from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.sim2real.domain_randomization import RandomizeEvery, \
    VisualRandomizationConfig
from rlbench.tasks.open_microwave import OpenMicrowave

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets', 'textures')


class TestDomainRandomizaionEnvironment(unittest.TestCase):

    def tearDown(self):
        self.env.shutdown()

    def get_task(self, randomize_every, frequency, visual_config=None,
                 dynamics_config=None):
        cam_config = CameraConfig(render_mode=RenderMode.OPENGL)
        obs_config = ObservationConfig(right_shoulder_camera=cam_config)
        obs_config.set_all(False)
        obs_config.set_all_low_dim(True)
        obs_config.right_shoulder_camera.rgb = True
        mode = MoveArmThenGripper(JointVelocity(), Discrete())
        self.env = Environment(
            mode, ASSET_DIR, obs_config, True, True, 'panda',
            randomize_every, frequency, visual_config, dynamics_config)
        self.env.launch()
        return self.env.get_task(OpenMicrowave)

    def test_visual_randomize_every_2_episodes(self):
        visual_config = VisualRandomizationConfig(ASSET_DIR)
        task = self.get_task(RandomizeEvery.EPISODE, 2, visual_config)
        _, obs1 = task.reset()
        _, obs2 = task.reset()
        _, obs3 = task.reset()
        # obs1 and obs2 should be almost identical
        self.assertLess(
            np.mean(np.abs(obs1.right_shoulder_rgb - obs2.right_shoulder_rgb)),
            5)
        # obs2 and obs3 should be very different
        self.assertGreater(
            np.mean(np.abs(obs2.right_shoulder_rgb - obs3.right_shoulder_rgb)),
            5)

    def test_visual_randomize_every_2_transitions(self):
        visual_config = VisualRandomizationConfig(ASSET_DIR)
        task = self.get_task(RandomizeEvery.TRANSITION, 2, visual_config)
        _, _ = task.reset()
        a = ([0.0] * 7) + [1.0]
        obs1, _, _ = task.step(a)
        obs2, _, _ = task.step(a)
        obs3, _, _ = task.step(a)
        # obs1 and obs2 should be almost identical
        self.assertLess(
            np.mean(np.abs(obs1.right_shoulder_rgb - obs2.right_shoulder_rgb)),
            5)
        # obs2 and obs3 should be very different
        self.assertGreater(
            np.mean(np.abs(obs2.right_shoulder_rgb - obs3.right_shoulder_rgb)),
            5)
