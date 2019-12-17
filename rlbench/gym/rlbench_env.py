import gym
from gym import spaces
from pyrep.const import RenderMode
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor
from rlbench.environment import Environment
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
import numpy as np


class RLBenchEnv(gym.Env):
    """An gym wrapper for RLBench."""

    metadata = {'render.modes': ['human']}

    def __init__(self, task_class, observation_mode='state'):
        self._observation_mode = observation_mode
        obs_config = ObservationConfig()
        if observation_mode == 'state':
            obs_config.set_all_high_dim(False)
            obs_config.set_all_low_dim(True)
        elif observation_mode == 'vision':
            obs_config.set_all(True)
        else:
            raise ValueError(
                'Unrecognised observation_mode: %s.' % observation_mode)
        action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)
        self.env = Environment(
            action_mode, obs_config=obs_config, headless=True)
        self.env.launch()
        self.task = self.env.get_task(task_class)

        _, obs = self.task.reset()

        self.action_space = spaces.Box(
            low=-1.0, high=1.0, shape=(action_mode.action_size,),
            dtype=np.float32)

        if observation_mode == 'state':
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=obs.get_low_dim_data().shape)
        elif observation_mode == 'vision':
            self.observation_space = spaces.Dict({
                "state": spaces.Box(
                    low=-np.inf, high=np.inf,
                    shape=obs.get_low_dim_data().shape),
                "left_shoulder_rgb": spaces.Box(
                    low=0, high=1, shape=obs.left_shoulder_rgb.shape),
                "right_shoulder_rgb": spaces.Box(
                    low=0, high=1, shape=obs.right_shoulder_rgb.shape),
                "wrist_rgb": spaces.Box(
                    low=0, high=1, shape=obs.wrist_rgb.shape),
                })

        self._gym_cam = None

    def _extract_obs(self, obs):
        if self._observation_mode == 'state':
            return obs.get_low_dim_data()
        elif self._observation_mode == 'vision':
            return {
                "state": obs.get_low_dim_data(),
                "left_shoulder_rgb": obs.left_shoulder_rgb,
                "right_shoulder_rgb": obs.right_shoulder_rgb,
                "wrist_rgb": obs.wrist_rgb,
            }

    def render(self, mode='human'):
        if self._gym_cam is None:
            # Add the camera to the scene
            cam_placeholder = Dummy('cam_cinematic_placeholder')
            self._gym_cam = VisionSensor.create([640, 360])
            self._gym_cam.set_pose(cam_placeholder.get_pose())
            self._gym_cam.set_render_mode(RenderMode.OPENGL3_WINDOWED)

    def reset(self):
        descriptions, obs = self.task.reset()
        del descriptions  # Not used.
        return self._extract_obs(obs)

    def step(self, action):
        obs, reward, terminate = self.task.step(action)
        return self._extract_obs(obs), reward, terminate, None

    def close(self):
        self.env.shutdown()
