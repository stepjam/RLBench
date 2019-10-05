from os import listdir
from os.path import join, exists
import pickle
import numpy as np
from PIL import Image
from pyrep import PyRep
from pyrep.errors import IKError
from rlbench.backend.exceptions import BoundaryError, WaypointError
from rlbench.backend.scene import Scene
from rlbench.backend.task import Task
from rlbench.backend.const import *
from rlbench.backend.utils import image_to_float_array
from rlbench.backend.robot import Robot
import logging
from typing import List
from rlbench.backend.observation import Observation
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
from pyquaternion import Quaternion

_TORQUE_MAX_VEL = 9999
_DT = 0.05
_MAX_RESET_ATTEMPTS = 40
_MAX_DEMO_ATTEMPTS = 10


class InvalidActionError(Exception):
    pass


class TaskEnvironmentError(Exception):
    pass


class TaskEnvironment(object):

    def __init__(self, pyrep: PyRep, robot: Robot, scene: Scene, task: Task,
                 action_mode: ActionMode, dataset_root: str,
                 obs_config: ObservationConfig,
                 static_positions: bool = False):
        self._pyrep = pyrep
        self._robot = robot
        self._scene = scene
        self._task = task
        self._variation_number = 0
        self._action_mode = action_mode
        self._dataset_root = dataset_root
        self._obs_config = obs_config
        self._static_positions = static_positions
        self._reset_called = False
        self._prev_ee_velocity = None

        self._scene.load(self._task)
        self._pyrep.start()

    def get_name(self) -> str:
        return self._task.get_name()

    def sample_variation(self) -> int:
        self._variation_number = np.random.randint(
            0, self._task.variation_count())
        return self._variation_number

    def reset(self) -> (List[str], Observation):
        logging.info('Resetting task: %s' % self._task.get_name())

        self._scene.reset()
        try:
            desc = self._scene.init_episode(
                self._variation_number, max_attempts=_MAX_RESET_ATTEMPTS,
                randomly_place=not self._static_positions)
        except (BoundaryError, WaypointError) as e:
            raise TaskEnvironmentError(
                'Could not place the task %s in the scene. This should not '
                'happen, please raise an issues on this task.'
                % self._task.get_name()) from e

        ctr_loop = self._robot.arm.joints[0].is_control_loop_enabled()
        locked = self._robot.arm.joints[0].is_motor_locked_at_zero_velocity()
        self._robot.arm.set_control_loop_enabled(False)
        self._robot.arm.set_motor_locked_at_zero_velocity(True)

        self._reset_called = True

        self._robot.arm.set_control_loop_enabled(ctr_loop)
        self._robot.arm.set_motor_locked_at_zero_velocity(locked)

        # Returns a list of descriptions and the first observation
        return desc, self._scene.get_observation()

    def _assert_action_space(self, action, expected_shape):
        if np.shape(action) != expected_shape:
            raise RuntimeError(
                'Expected the action shape to be: %s, but was shape: %s' % (
                    str(expected_shape), str(np.shape(action))))

    def _torque_action(self, action):
        self._robot.arm.set_joint_target_velocities(
            [(_TORQUE_MAX_VEL if t < 0 else -_TORQUE_MAX_VEL)
             for t in action])
        self._robot.arm.set_joint_forces(np.abs(action))

    def _ee_action(self, action):
        try:
            joint_positions = self._robot.arm.solve_ik(
                action[:3], quaternion=action[3:])
            self._robot.arm.set_joint_target_positions(joint_positions)
        except IKError as e:
            raise InvalidActionError('Could not find a path.') from e
        self._pyrep.step()

    def step(self, action) -> (Observation, int, bool):
        # returns observation, reward, done, info
        if not self._reset_called:
            raise RuntimeError(
                "Call 'reset' before calling 'step' on a task.")

        # action should contain 1 extra value for gripper open close state
        arm_action = np.array(action[:-1])

        ee_action = action[-1]
        current_ee = (1.0 if self._robot.gripper.get_open_amount()[0] > 0.9
                      else 0.0)

        if ee_action > 0.0:
            ee_action = 1.0
        elif ee_action < -0.0:
            ee_action = 0.0

        if self._action_mode.arm == ArmActionMode.ABS_JOINT_VELOCITY:

            self._assert_action_space(arm_action,
                                      (len(self._robot.arm.joints),))
            self._robot.arm.set_joint_target_velocities(arm_action)

        elif self._action_mode.arm == ArmActionMode.DELTA_JOINT_VELOCITY:

            self._assert_action_space(arm_action,
                                      (len(self._robot.arm.joints),))
            cur = np.array(self._robot.arm.get_joint_velocities())
            self._robot.arm.set_joint_target_velocities(cur + arm_action)

        elif self._action_mode.arm == ArmActionMode.ABS_JOINT_POSITION:

            self._assert_action_space(arm_action,
                                      (len(self._robot.arm.joints),))
            self._robot.arm.set_joint_target_positions(arm_action)

        elif self._action_mode.arm == ArmActionMode.DELTA_JOINT_POSITION:

            self._assert_action_space(arm_action,
                                      (len(self._robot.arm.joints),))
            cur = np.array(self._robot.arm.get_joint_positions())
            self._robot.arm.set_joint_target_positions(cur + arm_action)

        elif self._action_mode.arm == ArmActionMode.ABS_EE_POSE:

            self._assert_action_space(arm_action, (7,))
            self._ee_action(list(arm_action))

        elif self._action_mode.arm == ArmActionMode.DELTA_EE_POSE:

            self._assert_action_space(arm_action, (7,))
            a_x, a_y, a_z, a_qx, a_qy, a_qz, a_qw = arm_action
            x, y, z, qx, qy, qz, qw = self._robot.arm.get_tip().get_pose()
            new_rot = Quaternion(a_qw, a_qx, a_qy, a_qz) * Quaternion(qw, qx, qy, qz)
            qw, qx, qy, qz = list(new_rot)
            new_pose = [a_x + x, a_y + y, a_z + z] + [qx, qy, qz, qw]
            self._ee_action(list(new_pose))

        elif self._action_mode.arm == ArmActionMode.ABS_EE_VELOCITY:

            self._assert_action_space(arm_action, (7,))
            pose = self._robot.arm.get_tip().get_pose()
            new_pos = np.array(pose) + (arm_action * _DT)
            self._ee_action(list(new_pos))

        elif self._action_mode.arm == ArmActionMode.DELTA_EE_VELOCITY:

            self._assert_action_space(arm_action, (7,))
            if self._prev_ee_velocity is None:
                self._prev_ee_velocity = np.zeros((7,))
            self._prev_ee_velocity += arm_action
            pose = self._robot.arm.get_tip().get_pose()
            pose = np.array(pose)
            new_pose = pose + (self._prev_ee_velocity * _DT)
            self._ee_action(list(new_pose))

        elif self._action_mode.arm == ArmActionMode.ABS_JOINT_TORQUE:

            self._assert_action_space(arm_action, (len(self._robot.arm.joints),))
            self._torque_action(arm_action)

        elif self._action_mode.arm == ArmActionMode.DELTA_JOINT_TORQUE:

            cur = np.array(self._robot.arm.get_joint_forces())
            new_action = cur + arm_action
            self._torque_action(new_action)

        else:
            raise RuntimeError('Unrecognised action mode.')

        if current_ee != ee_action:
            done = False
            while not done:
                done = self._robot.gripper.actuate(ee_action, velocity=0.04)
                self._pyrep.step()
                self._task.step()
            if ee_action == 0.0:
                # If gripper close action, the check for grasp.
                for g_obj in self._task.get_graspable_objects():
                    self._robot.gripper.grasp(g_obj)
            else:
                # If gripper opem action, the check for ungrasp.
                self._robot.gripper.release()

        self._scene.step()

        success, terminate = self._task.success()
        return self._scene.get_observation(), int(success), terminate

    def get_demos(self, amount: int, live_demos=False,
                  image_paths=True) -> List[List[Observation]]:
        """Negative means all demos"""

        if not live_demos and (self._dataset_root is None
                       or len(self._dataset_root) == 0):
            raise RuntimeError(
                "Can't ask for a stored demo when no dataset root provided.")

        if not live_demos:
            if self._dataset_root is None or len(self._dataset_root) == 0:
                raise RuntimeError(
                    "Can't ask for stored demo when no dataset root provided.")
            demos = self._get_stored_demos(amount, image_paths)
        else:
            ctr_loop = self._robot.arm.joints[0].is_control_loop_enabled()
            self._robot.arm.joints[0].set_control_loop_enabled(True)
            demos = self._get_live_demos(amount)
            self._robot.arm.joints[0].set_control_loop_enabled(ctr_loop)
        return demos

    def _get_live_demos(self, amount):
        demos = []
        for i in range(amount):
            attempts = _MAX_DEMO_ATTEMPTS
            while attempts > 0:
                self.reset()
                logging.info('Collecting demo %d' % i)
                try:
                    obs = self._scene.get_demo()
                    demos.append(obs)
                    break
                except Exception as e:
                    attempts -= 1
                    logging.info('Bad demo. ' + str(e))
            if attempts <= 0:
                raise RuntimeError(
                    'Could not collect demos. Maybe a problem with the task?')
        return demos

    def _resize_if_needed(self, image, size):
        if image.size[0] != size[0] or image.size[1] != size[1]:
            image = image.resize(size)
        return image

    def _get_stored_demos(self, amount: int, image_paths: bool):

        task_root = join(self._dataset_root, self._task.get_name())
        if not exists(task_root):
            raise RuntimeError("Can't find the demos for %s at: %s" % (
                self._task.get_name(), task_root))

        # Sample an amount of examples for the variation of this task
        examples_path = join(
            task_root, VARIATIONS_FOLDER % self._variation_number,
            EPISODES_FOLDER)
        examples = listdir(examples_path)
        if amount == -1:
            amount = len(examples)
        if amount > len(examples):
            raise RuntimeError(
                'You asked for %d examples, but only %d were available.' % (
                    amount, len(examples)))
        selected_examples = np.random.choice(examples, amount, replace=False)

        obs_config = self._obs_config

        # Process these examples (e.g. loading observations)
        demos = []
        for example in selected_examples:
            example_path = join(examples_path, example)
            with open(join(example_path, LOW_DIM_PICKLE), 'rb') as f:
                obs = pickle.load(f)

            l_sh_rgb_f = join(example_path, LEFT_SHOULDER_RGB_FOLDER)
            l_sh_depth_f = join(example_path, LEFT_SHOULDER_DEPTH_FOLDER)
            l_sh_mask_f = join(example_path, LEFT_SHOULDER_MASK_FOLDER)
            r_sh_rgb_f = join(example_path, RIGHT_SHOULDER_RGB_FOLDER)
            r_sh_depth_f = join(example_path, RIGHT_SHOULDER_DEPTH_FOLDER)
            r_sh_mask_f = join(example_path, RIGHT_SHOULDER_MASK_FOLDER)
            wrist_rgb_f = join(example_path, WRIST_RGB_FOLDER)
            wrist_depth_f = join(example_path, WRIST_DEPTH_FOLDER)
            wrist_mask_f = join(example_path, WRIST_MASK_FOLDER)

            num_steps = len(obs)

            if not (num_steps == len(listdir(l_sh_rgb_f)) == len(
                    listdir(l_sh_depth_f)) == len(listdir(r_sh_rgb_f)) == len(
                    listdir(r_sh_depth_f)) == len(listdir(wrist_rgb_f)) == len(
                    listdir(wrist_depth_f))):
                raise RuntimeError('Broken dataset assumption')

            for i in range(num_steps):
                si = IMAGE_FORMAT % i
                if obs_config.left_shoulder_camera.rgb:
                    obs[i].left_shoulder_rgb = join(l_sh_rgb_f, si)
                if obs_config.left_shoulder_camera.depth:
                    obs[i].left_shoulder_depth = join(l_sh_depth_f, si)
                if obs_config.left_shoulder_camera.mask:
                    obs[i].left_shoulder_mask = join(l_sh_mask_f, si)
                if obs_config.right_shoulder_camera.rgb:
                    obs[i].right_shoulder_rgb = join(r_sh_rgb_f, si)
                if obs_config.right_shoulder_camera.depth:
                    obs[i].right_shoulder_depth = join(r_sh_depth_f, si)
                if obs_config.right_shoulder_camera.depth:
                    obs[i].right_shoulder_mask = join(r_sh_mask_f, si)
                if obs_config.wrist_camera.rgb:
                    obs[i].wrist_rgb = join(wrist_rgb_f, si)
                if obs_config.wrist_camera.depth:
                    obs[i].wrist_depth = join(wrist_depth_f, si)
                if obs_config.wrist_camera.depth:
                    obs[i].wrist_mask = join(wrist_mask_f, si)
            if not image_paths:
                for i in range(num_steps):
                    if obs_config.left_shoulder_camera.rgb:
                        obs[i].left_shoulder_rgb = np.array(
                            self._resize_if_needed(
                                Image.open(obs[i].left_shoulder_rgb),
                                obs_config.left_shoulder_camera.image_size))
                    if obs_config.left_shoulder_camera.depth:
                        obs[i].left_shoulder_depth = image_to_float_array(
                            self._resize_if_needed(
                                Image.open(obs[i].left_shoulder_depth),
                                obs_config.left_shoulder_camera.image_size),
                            DEPTH_SCALE)
                    if obs_config.left_shoulder_camera.mask:
                        obs[i].left_shoulder_mask = np.array(
                            self._resize_if_needed(Image.open(
                                obs[i].left_shoulder_mask),
                                obs_config.left_shoulder_camera.image_size))
                    if obs_config.right_shoulder_camera.rgb:
                        obs[i].right_shoulder_rgb = np.array(
                            self._resize_if_needed(Image.open(
                            obs[i].right_shoulder_rgb),
                                obs_config.right_shoulder_camera.image_size))
                    if obs_config.right_shoulder_camera.depth:
                        obs[i].right_shoulder_depth = image_to_float_array(
                            self._resize_if_needed(
                                Image.open(obs[i].right_shoulder_depth),
                                obs_config.right_shoulder_camera.image_size),
                            DEPTH_SCALE)
                    if obs_config.right_shoulder_camera.mask:
                        obs[i].right_shoulder_mask = np.array(
                            self._resize_if_needed(Image.open(
                                obs[i].right_shoulder_mask),
                                obs_config.right_shoulder_camera.image_size))
                    if obs_config.wrist_camera.rgb:
                        obs[i].wrist_rgb = np.array(
                            self._resize_if_needed(
                                Image.open(obs[i].wrist_rgb),
                                obs_config.wrist_camera.image_size))
                    if obs_config.wrist_camera.depth:
                        obs[i].wrist_depth = image_to_float_array(
                            self._resize_if_needed(
                                Image.open(obs[i].wrist_depth),
                                obs_config.wrist_camera.image_size), DEPTH_SCALE)
                    if obs_config.wrist_camera.mask:
                        obs[i].wrist_mask = np.array(
                            self._resize_if_needed(Image.open(
                                obs[i].wrist_mask),
                                obs_config.wrist_camera.image_size))
            demos.append(obs)
        return demos
