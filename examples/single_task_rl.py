from rlbench.environment import Environment
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import ReachTarget
import numpy as np


class Agent(object):

    def __init__(self, action_size):
        self.action_size = action_size

    def act(self, obs):
        arm = np.random.normal(0.0, 0.1, size=(self.action_size - 1,))
        gripper = [1.0]  # Always open
        return np.concatenate([arm, gripper], axis=-1)


obs_config = ObservationConfig()
obs_config.set_all(True)

action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)
env = Environment(
    action_mode, obs_config=obs_config, headless=False)
env.launch()

task = env.get_task(ReachTarget)

agent = Agent(env.action_size)

training_steps = 120
episode_length = 40
obs = None
for i in range(training_steps):
    if i % episode_length == 0:
        print('Reset Episode')
        descriptions, obs = task.reset()
        print(descriptions)
    action = agent.act(obs)
    print(action)
    obs, reward, terminate = task.step(action)

print('Done')
env.shutdown()
