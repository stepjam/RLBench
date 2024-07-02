import gymnasium as gym
from gymnasium.utils.performance import benchmark_step
import rlbench

env = gym.make('rlbench/reach_target-vision-v0', render_mode="rgb_array")


training_steps = 120
episode_length = 40
for i in range(training_steps):
    if i % episode_length == 0:
        print('Reset Episode')
        obs = env.reset()
    obs, reward, terminate, _, _ = env.step(env.action_space.sample())
    env.render()  # Note: rendering increases step time.

print('Done')

fps = benchmark_step(env, target_duration=10)
print(f"FPS: {fps:.2f}")
env.close()
