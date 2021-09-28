import numpy as np

from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.backend.scene import Scene
from rlbench.environment import Environment
from rlbench.observation_config import ObservationConfig
from rlbench.tasks import ReachTarget


class CustomAbsoluteJointVelocity(JointVelocity):

    def action(self, scene: Scene, action: np.ndarray):
        if np.random.random() > 0.5:
            # Example of custom behaviour.
            # Here we randomly ignore 50% of actions
            print('Skip action!')
            return
        super(CustomAbsoluteJointVelocity, self).action(scene, action)


class Agent(object):

    def __init__(self, action_shape):
        self.action_shape = action_shape

    def act(self, obs):
        arm = np.random.normal(0.0, 0.1, size=(self.action_shape[0] - 1,))
        gripper = [1.0]  # Always open
        return np.concatenate([arm, gripper], axis=-1)


action_mode = MoveArmThenGripper(
    arm_action_mode=CustomAbsoluteJointVelocity(),
    gripper_action_mode=Discrete())

env = Environment(
    action_mode,
    obs_config=ObservationConfig(),
    headless=False)
env.launch()

task = env.get_task(ReachTarget)

agent = Agent(env.action_shape)

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
