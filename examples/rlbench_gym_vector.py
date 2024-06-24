import time
import gymnasium as gym
import rlbench


def benchmark_vector_step(env, target_duration: int = 5, seed=None) -> float:
    steps = 0
    end = 0.0
    env.reset(seed=seed)
    env.action_space.sample()
    start = time.monotonic()

    while True:
        action = env.action_space.sample()
        _, _, terminal, truncated, _ = env.step(action)
        steps += terminal.shape[0]

        # if terminal or truncated:
        #     env.reset()

        if time.monotonic() - start > target_duration:
            end = time.monotonic()
            break

    length = end - start

    steps_per_time = steps / length
    return steps_per_time

if __name__ == "__main__":
    # Only works with spawn (multiprocessing) context
    env = gym.make_vec('rlbench/reach_target-vision-v0', num_envs=2, vectorization_mode="async", vector_kwargs={"context": "spawn"})

    training_steps = 120
    episode_length = 40
    for i in range(training_steps):
        if i % episode_length == 0:
            print('Reset Episode')
            obs = env.reset()
        obs, reward, terminate, _, _ = env.step(env.action_space.sample())
        env.render()  # Note: rendering increases step time.

    print('Done')

    fps = benchmark_vector_step(env, target_duration=10)
    print(f"FPS: {fps:.2f}")

    env.close()
