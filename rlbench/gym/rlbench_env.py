from typing import Union, Dict, Tuple

import gymnasium as gym
import numpy as np
from gymnasium import spaces
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor
from pyrep.const import RenderMode


from rlbench.action_modes.action_mode import JointPositionActionMode
from rlbench.environment import Environment
from rlbench.observation_config import ObservationConfig

class RLBenchEnv(gym.Env):
    """An gym wrapper for RLBench."""
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 4}

    def __init__(self, task_class, observation_mode='state',
                 render_mode: Union[None, str] = None, action_mode=None):
        self.task_class = task_class
        self._observation_mode = observation_mode
        self._render_mode = render_mode
        obs_config = ObservationConfig()
        if observation_mode == 'state':
            obs_config.set_all_high_dim(False)
            obs_config.set_all_low_dim(True)
        elif observation_mode == 'vision':
            obs_config.set_all(True)
        else:
            raise ValueError(
                'Unrecognised observation_mode: %s.' % observation_mode)
        self.obs_config = obs_config
        if action_mode is None:
            action_mode = JointPositionActionMode()
        self.action_mode = action_mode

        self.rlbench_env = Environment(
            action_mode=self.action_mode,
            obs_config=self.obs_config,
            headless=True,
        )
        self.rlbench_env.launch()
        self.rlbench_task_env = self.rlbench_env.get_task(self.task_class)
        if render_mode is not None:
            cam_placeholder = Dummy("cam_cinematic_placeholder")
            self.gym_cam = VisionSensor.create([640, 360])
            self.gym_cam.set_pose(cam_placeholder.get_pose())
            if render_mode == "human":
                self.gym_cam.set_render_mode(RenderMode.OPENGL3_WINDOWED)
            else:
                self.gym_cam.set_render_mode(RenderMode.OPENGL3)
        _, obs = self.rlbench_task_env.reset()

        self.observation_space = {
            "state": spaces.Box(
                low=-np.inf, high=np.inf, shape=obs.get_low_dim_data().shape),
        }
        if observation_mode == 'vision':
            self.observation_space.update({
                "left_shoulder_rgb": spaces.Box(
                    low=0, high=255, shape=obs.left_shoulder_rgb.shape, dtype=np.uint8),
                "right_shoulder_rgb": spaces.Box(
                    low=0, high=255, shape=obs.right_shoulder_rgb.shape, dtype=np.uint8),
                "wrist_rgb": spaces.Box(
                    low=0, high=255, shape=obs.wrist_rgb.shape, dtype=np.uint8),
                "front_rgb": spaces.Box(
                    low=0, high=255, shape=obs.front_rgb.shape, dtype=np.uint8),
            })
        self.observation_space = spaces.Dict(self.observation_space)
        
        action_low, action_high = action_mode.action_bounds()
        self.action_space = spaces.Box(
            low=action_low, high=action_high, shape=self.rlbench_env.action_shape)

    def _extract_obs(self, rlbench_obs):
        gym_obs = {} 
        gym_obs["state"] = np.float32(rlbench_obs.get_low_dim_data())
        if self._observation_mode == 'vision':
            gym_obs.update({
                "left_shoulder_rgb": rlbench_obs.left_shoulder_rgb,
                "right_shoulder_rgb": rlbench_obs.right_shoulder_rgb,
                "wrist_rgb": rlbench_obs.wrist_rgb,
                "front_rgb": rlbench_obs.front_rgb,
            })
        return gym_obs

    def render(self):
        if self.render_mode == 'rgb_array':
            frame = self.gym_cam.capture_rgb()
            frame = np.clip((frame * 255.).astype(np.uint8), 0, 255)
            return frame

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        descriptions, obs = self.rlbench_task_env.reset()
        return self._extract_obs(obs), {"text_descriptions": descriptions}

    def step(self, action):
        obs, reward, success, _terminate = self.rlbench_task_env.step(action)
        terminated = success
        truncated = _terminate and not success
        return self._extract_obs(obs), reward, terminated, truncated, {"success": success}

    def close(self) -> None:
        self.rlbench_env.shutdown()
        

