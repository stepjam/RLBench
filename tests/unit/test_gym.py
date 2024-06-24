import gymnasium as gym
import numpy as np
import pytest
from gymnasium.utils.env_checker import check_env

import rlbench


@pytest.mark.parametrize("env_id", ['rlbench/reach_target-state-v0', ])
def test_env(env_id):
    env = gym.make(env_id, render_mode="rgb_array")
    check_env(env, skip_render_check=True, skip_close_check=True)
    img = env.render()
    assert np.sum(img) > 0 
    env.close()

    env = gym.make(env_id, render_mode="rgb_array")
    for i in range(10):
        obs, _ = env.reset(seed=i)
        obs2, _ = env.reset(seed=i)
        assert np.allclose(obs["joint_positions"], obs2["joint_positions"], atol=1e-3)
        assert np.allclose(obs["task_low_dim_state"], obs2["task_low_dim_state"])

    env.close()
