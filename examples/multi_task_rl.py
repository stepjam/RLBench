import numpy as np

from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.environment import Environment
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import MT30_V1


class Agent(object):

    def __init__(self, action_shape):
        self.action_shape = action_shape

    def act(self, obs):
        arm = np.random.normal(0.0, 0.1, size=(self.action_shape[0] - 1,))
        gripper = [1.0]  # Always open
        return np.concatenate([arm, gripper], axis=-1)


obs_config = ObservationConfig()
obs_config.set_all(True)

env = Environment(
    action_mode=MoveArmThenGripper(
        arm_action_mode=JointVelocity(), gripper_action_mode=Discrete()),
    obs_config=ObservationConfig(),
    headless=False)
env.launch()

agent = Agent(env.action_shape)

train_tasks = MT30_V1['train']

training_cycles_per_task = 3
training_steps_per_task = 80
episode_length = 40

for _ in range(training_cycles_per_task):

    task_to_train = np.random.choice(train_tasks, 1)[0]
    task = env.get_task(task_to_train)
    task.sample_variation()  # random variation

    for i in range(training_steps_per_task):
        if i % episode_length == 0:
            print('Reset Episode')
            descriptions, obs = task.reset()
            print(descriptions)
        action = agent.act(obs)
        obs, reward, terminate = task.step(action)

print('Done')
env.shutdown()
