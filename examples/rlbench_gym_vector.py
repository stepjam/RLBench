import gymnasium as gym
import rlbench.gym

if __name__ == "__main__":
    # Only works with spawn (multiprocessing) context
    env = gym.make_vec('reach_target-vision-v0', num_envs=2, vectorization_mode="async", vector_kwargs={"context": "spawn"})

    training_steps = 120
    episode_length = 40
    for i in range(training_steps):
        if i % episode_length == 0:
            print('Reset Episode')
            obs = env.reset()
        obs, reward, terminate, _, _ = env.step(env.action_space.sample())
        env.render()  # Note: rendering increases step time.

    print('Done')
    env.close()
