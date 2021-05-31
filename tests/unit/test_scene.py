import unittest
from os import path
from os.path import join
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from rlbench import environment
from rlbench.backend.const import TTT_FILE
from rlbench.backend.scene import Scene
from rlbench.noise_model import GaussianNoise
from rlbench.observation_config import ObservationConfig, CameraConfig
import numpy as np
from rlbench.backend.robot import Robot
from rlbench.tasks.reach_target import ReachTarget

ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets', 'tasks')


class TestScene(unittest.TestCase):
    """Tests the following:
        - Getting observations from the scene
        - Applying noise
        - Applying domain randomization
    """

    def setUp(self):
        self.pyrep = PyRep()
        self.pyrep.launch(join(environment.DIR_PATH, TTT_FILE), headless=True)
        self.pyrep.set_simulation_timestep(0.005)
        self.robot = Robot(Panda(), PandaGripper())

    def tearDown(self):
        self.pyrep.shutdown()

    def test_sensor_noise_images(self):
        cam_config = CameraConfig(rgb_noise=GaussianNoise(0.05, (.0, 1.)))
        obs_config = ObservationConfig(
            left_shoulder_camera=cam_config,
            joint_forces=False,
            task_low_dim_state=False)
        scene = Scene(self.pyrep, self.robot, obs_config)
        scene.load(ReachTarget(self.pyrep, self.robot))
        obs1 = scene.get_observation()
        obs2 = scene.get_observation()
        self.assertTrue(
            np.array_equal(obs1.right_shoulder_rgb, obs2.right_shoulder_rgb))
        self.assertFalse(
            np.array_equal(obs1.left_shoulder_rgb, obs2.left_shoulder_rgb))
        self.assertTrue(obs1.left_shoulder_rgb.max() <= 255)
        self.assertTrue(obs1.left_shoulder_rgb.min() >= 0)

    def test_sensor_noise_robot(self):
        obs_config = ObservationConfig(
            joint_velocities_noise=GaussianNoise(0.01),
            joint_forces=False,
            task_low_dim_state=False)
        scene = Scene(self.pyrep, self.robot, obs_config)
        scene.load(ReachTarget(self.pyrep, self.robot))
        obs1 = scene.get_observation()
        obs2 = scene.get_observation()
        self.assertTrue(
            np.array_equal(obs1.joint_positions, obs2.joint_positions))
        self.assertFalse(
            np.array_equal(obs1.joint_velocities, obs2.joint_velocities))

