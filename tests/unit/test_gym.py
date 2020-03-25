import unittest
import numpy as np
import gym
import rlbench.gym


class TestGym(unittest.TestCase):

    def test_state_env(self):
        env = gym.make('reach_target-state-v0')
        env.reset()
        obs, _, _, _ = env.step(env.action_space.sample())
        self.assertEqual(env.observation_space.shape,
                         obs.shape)
        env.close()

    def test_vision_env(self):
        env = gym.make('reach_target-vision-v0')
        env.reset()
        obs, _, _, _ = env.step(env.action_space.sample())
        self.assertEqual(env.observation_space['state'].shape,
                         obs['state'].shape)
        self.assertEqual(env.observation_space['left_shoulder_rgb'].shape,
                         obs['left_shoulder_rgb'].shape)
        self.assertEqual(env.observation_space['right_shoulder_rgb'].shape,
                         obs['right_shoulder_rgb'].shape)
        self.assertEqual(env.observation_space['wrist_rgb'].shape,
                         obs['wrist_rgb'].shape)
        self.assertEqual(env.observation_space['front_rgb'].shape,
                         obs['front_rgb'].shape)
        env.close()

    def test_env_render(self):
        env = gym.make('reach_target-vision-v0', render_mode='rgb_array')
        env.reset()
        obs, _, _, _ = env.step(env.action_space.sample())
        img = env.render('rgb_array')
        self.assertGreater(np.mean(img), 0)
        env.close()

