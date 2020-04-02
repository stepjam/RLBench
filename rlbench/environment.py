from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.arms.jaco import Jaco
from pyrep.robots.arms.mico import Mico
from pyrep.robots.arms.sawyer import Sawyer
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from pyrep.robots.end_effectors.jaco_gripper import JacoGripper
from pyrep.robots.end_effectors.mico_gripper import MicoGripper
from pyrep.robots.end_effectors.baxter_gripper import BaxterGripper
from rlbench.sim2real.domain_randomization import RandomizeEvery, \
    VisualRandomizationConfig, DynamicsRandomizationConfig

from rlbench.sim2real.domain_randomization_scene import DomainRandomizationScene

from rlbench.backend.scene import Scene
from rlbench.backend.task import Task
from rlbench.backend.const import *
from rlbench.backend.robot import Robot
from os.path import exists, dirname, abspath, join
import importlib
from typing import Type
from rlbench.observation_config import ObservationConfig
from rlbench.task_environment import TaskEnvironment
from rlbench.action_modes import ActionMode, ArmActionMode


DIR_PATH = dirname(abspath(__file__))

# Arms from PyRep need to be modified to include a wrist camera.
# Currently, only the arms/grippers below are supported.
SUPPORTED_ROBOTS = {
    'panda': (Panda, PandaGripper, 7),
    'jaco': (Jaco, JacoGripper, 6),
    'mico': (Mico, MicoGripper, 6),
    'sawyer': (Sawyer, BaxterGripper, 7),
}


class Environment(object):
    """Each environment has a scene."""

    def __init__(self, action_mode: ActionMode, dataset_root: str='',
                 obs_config=ObservationConfig(), headless=False,
                 static_positions: bool=False,
                 robot_configuration='panda',
                 randomize_every: RandomizeEvery=None,
                 frequency: int=1,
                 visual_randomization_config: VisualRandomizationConfig=None,
                 dynamics_randomization_config: DynamicsRandomizationConfig=None,
                 ):

        self._dataset_root = dataset_root
        self._action_mode = action_mode
        self._obs_config = obs_config
        self._headless = headless
        self._static_positions = static_positions
        self._robot_configuration = robot_configuration

        self._randomize_every = randomize_every
        self._frequency = frequency
        self._visual_randomization_config = visual_randomization_config
        self._dynamics_randomization_config = dynamics_randomization_config

        if robot_configuration not in SUPPORTED_ROBOTS.keys():
            raise ValueError('robot_configuration must be one of %s' %
                             str(SUPPORTED_ROBOTS.keys()))

        if (randomize_every is not None and
                    visual_randomization_config is None and
                    dynamics_randomization_config is None):
            raise ValueError(
                'If domain randomization is enabled, must supply either '
                'visual_randomization_config or dynamics_randomization_config')

        self._check_dataset_structure()

        self._pyrep = None
        self._robot = None
        self._scene = None
        self._prev_task = None

    def _set_arm_control_action(self):
        self._robot.arm.set_control_loop_enabled(True)
        if (self._action_mode.arm == ArmActionMode.ABS_JOINT_VELOCITY or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_VELOCITY):
            self._robot.arm.set_control_loop_enabled(False)
            self._robot.arm.set_motor_locked_at_zero_velocity(True)
        elif (self._action_mode.arm == ArmActionMode.ABS_JOINT_POSITION or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_POSITION or
                self._action_mode.arm == ArmActionMode.ABS_EE_POSE or
                self._action_mode.arm == ArmActionMode.DELTA_EE_POSE or
                self._action_mode.arm == ArmActionMode.ABS_EE_VELOCITY or
                self._action_mode.arm == ArmActionMode.DELTA_EE_VELOCITY or
                self._action_mode.arm == ArmActionMode.ABS_EE_POSE_PLAN or
                self._action_mode.arm == ArmActionMode.DELTA_EE_POSE_PLAN):
            self._robot.arm.set_control_loop_enabled(True)
        elif (self._action_mode.arm == ArmActionMode.ABS_JOINT_TORQUE or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_TORQUE):
            self._robot.arm.set_control_loop_enabled(False)
        else:
            raise RuntimeError('Unrecognised action mode.')

    def _check_dataset_structure(self):
        if len(self._dataset_root) > 0 and not exists(self._dataset_root):
            raise RuntimeError(
                'Data set root does not exists: %s' % self._dataset_root)

    def _string_to_task(self, task_name: str):
        task_name = task_name.replace('.py', '')
        try:
            class_name = ''.join(
                [w[0].upper() + w[1:] for w in task_name.split('_')])
            mod = importlib.import_module("rlbench.tasks.%s" % task_name)
        except Exception as e:
            raise RuntimeError(
                'Tried to interpret %s as a task, but failed. Only valid tasks '
                'should belong in the tasks/ folder' % task_name) from e
        return getattr(mod, class_name)

    def launch(self):
        if self._pyrep is not None:
            raise RuntimeError('Already called launch!')
        self._pyrep = PyRep()
        self._pyrep.launch(join(DIR_PATH, TTT_FILE), headless=self._headless)

        arm_class, gripper_class, _ = SUPPORTED_ROBOTS[
            self._robot_configuration]

        # We assume the panda is already loaded in the scene.
        if self._robot_configuration is not 'panda':
            # Remove the panda from the scene
            panda_arm = Panda()
            panda_pos = panda_arm.get_position()
            panda_arm.remove()
            arm_path = join(DIR_PATH,
                            'robot_ttms', self._robot_configuration + '.ttm')
            self._pyrep.import_model(arm_path)
            arm, gripper = arm_class(), gripper_class()
            arm.set_position(panda_pos)
        else:
            arm, gripper = arm_class(), gripper_class()

        self._robot = Robot(arm, gripper)
        if self._randomize_every is None:
            self._scene = Scene(self._pyrep, self._robot, self._obs_config)
        else:
            self._scene = DomainRandomizationScene(
                self._pyrep, self._robot, self._obs_config,
                self._randomize_every, self._frequency,
                self._visual_randomization_config,
                self._dynamics_randomization_config)

        self._set_arm_control_action()

    def shutdown(self):
        if self._pyrep is not None:
            self._pyrep.shutdown()
        self._pyrep = None

    def get_task(self, task_class: Type[Task]) -> TaskEnvironment:

        # If user hasn't called launch, implicitly call it.
        if self._pyrep is None:
            self.launch()

        self._scene.unload()
        task = task_class(self._pyrep, self._robot)
        self._prev_task = task
        return TaskEnvironment(
            self._pyrep, self._robot, self._scene, task,
            self._action_mode, self._dataset_root, self._obs_config,
            self._static_positions)

    @property
    def action_size(self):
        arm_action_size = 0
        gripper_action_size = 1  # Only one gripper style atm
        if (self._action_mode.arm == ArmActionMode.ABS_JOINT_VELOCITY or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_VELOCITY or
                self._action_mode.arm == ArmActionMode.ABS_JOINT_POSITION or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_POSITION or
                self._action_mode.arm == ArmActionMode.ABS_JOINT_TORQUE or
                self._action_mode.arm == ArmActionMode.DELTA_JOINT_TORQUE or
                self._action_mode.arm == ArmActionMode.ABS_EE_VELOCITY or
                self._action_mode.arm == ArmActionMode.DELTA_EE_VELOCITY):
            arm_action_size = SUPPORTED_ROBOTS[self._robot_configuration][2]
        elif (self._action_mode.arm == ArmActionMode.ABS_EE_POSE or
              self._action_mode.arm == ArmActionMode.DELTA_EE_POSE or
              self._action_mode.arm == ArmActionMode.ABS_EE_POSE_PLAN or
              self._action_mode.arm == ArmActionMode.DELTA_EE_POSE_PLAN):
            arm_action_size = 7  # pose is always 7
        return arm_action_size + gripper_action_size


