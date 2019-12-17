import unittest
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
        env.close()
