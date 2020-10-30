"""
This is a simple starting point for the rearrangement challenge mentioned
in .....

"""

from rlbench.environment import Environment
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import PutAllGroceriesInCupboard
import numpy as np


class Agent(object):
    """A simple random-action agent. """

    def __init__(self, action_size):
        self.action_size = action_size

    def act(self, obs):
        arm = np.random.normal(0.0, 0.1, size=(self.action_size - 1,))
        gripper = [1.0]  # Always open
        return np.concatenate([arm, gripper], axis=-1)


# Define the observations that we want to get at each timestep.
obs_config = ObservationConfig()
obs_config.set_all(True)

# Define the action mode of the arm. There are many to choose from.
action_mode = ActionMode(ArmActionMode.ABS_JOINT_VELOCITY)

# Create and launch the RLBench environment.
env = Environment(
    action_mode, obs_config=obs_config, headless=False)
env.launch()

# Get the task that we want to interface with. There are >100 tasks to choose!
# For the rearrangement challenge, we want 'PutAllGroceriesInCupboard'.
task = env.get_task(PutAllGroceriesInCupboard)

# Uncomment line below to get 'live' demonstrations of this task!
# demos = task.get_demos(1)

# Create our simple agent
agent = Agent(env.action_size)

training_steps = 120
episode_length = 40
for i in range(training_steps):
    if i % episode_length == 0:
        print('Reset Episode')
        # When we reset the task, we get given a list of strings that describe
        # the task and an initial observation
        descriptions, obs = task.reset()
        print(descriptions)
    # Using the current observation, use an agent to decide on the next action
    action = agent.act(obs)
    print(action)
    # Step the task and obtain a new observation, reward and a terminate flag.
    obs, reward, terminate = task.step(action)

print('Done')
env.shutdown()
