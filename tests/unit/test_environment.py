import unittest
from os import path

from pyrep.objects import Dummy

from rlbench import environment
from rlbench.task_environment import TaskEnvironment
from rlbench.tasks import TakeLidOffSaucepan, ReachTarget
from rlbench.action_modes import ArmActionMode, ActionMode
from rlbench.observation_config import ObservationConfig
import numpy as np
ASSET_DIR = path.join(path.dirname(path.abspath(__file__)), 'assets', 'tasks')


class TestEnvironment(unittest.TestCase):

    def tearDown(self):
        self.env.shutdown()

    def get_task(self, task_class, arm_action_mode, obs_config=None):
        if obs_config is None:
            obs_config = ObservationConfig()
            obs_config.set_all(False)
            obs_config.set_all_low_dim(True)
            obs_config.right_shoulder_camera.rgb = True
        action_mode = ActionMode(arm_action_mode)
        self.env = environment.Environment(
            action_mode, ASSET_DIR, obs_config, headless=True)
        self.env.launch()
        return self.env.get_task(task_class)

    def test_get_task(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        self.assertIsInstance(task, TaskEnvironment)
        self.assertEqual(task.get_name(), 'reach_target')

    def test_reset(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        desc, obs = task.reset()
        self.assertIsNotNone(obs.right_shoulder_rgb)
        self.assertIsNone(obs.left_shoulder_rgb)
        self.assertIsInstance(desc, list)

    def test_get_all_camera_observations(self):
        obs_config = ObservationConfig()
        obs_config.left_shoulder_camera.rgb = True
        obs_config.right_shoulder_camera.rgb = True
        obs_config.front_camera.rgb = True
        obs_config.wrist_camera.rgb = True
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY, obs_config)
        desc, obs = task.reset()
        self.assertIsNotNone(obs.left_shoulder_rgb)
        self.assertIsNotNone(obs.right_shoulder_rgb)
        self.assertIsNotNone(obs.front_rgb)
        self.assertIsNotNone(obs.wrist_rgb)

    def test_step(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        task.reset()
        obs, reward, term = task.step(np.random.uniform(size=8))
        self.assertIsNotNone(obs.right_shoulder_rgb)
        self.assertIsNone(obs.left_shoulder_rgb)
        self.assertEqual(reward, 0)
        self.assertFalse(term)

    def test_get_invalid_number_of_demos(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        with self.assertRaises(RuntimeError):
            task.get_demos(10, live_demos=False, image_paths=True)

    def test_get_stored_demos_paths(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        demos = task.get_demos(2, live_demos=False, image_paths=True)
        self.assertEqual(len(demos), 2)
        self.assertGreater(len(demos[0]), 0)
        self.assertIsInstance(demos[0][0].right_shoulder_rgb, str)
        self.assertIsNone(demos[0][0].left_shoulder_rgb)

    def test_get_stored_demos_images(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        demos = task.get_demos(2, live_demos=False, image_paths=False)
        self.assertEqual(len(demos), 2)
        self.assertGreater(len(demos[0]), 0)
        self.assertIsInstance(demos[0][0].right_shoulder_rgb, np.ndarray)
        self.assertIsNone(demos[0][0].left_shoulder_rgb)

    def test_get_stored_demos_images_without_init_sim(self):
        obs_config = ObservationConfig()
        obs_config.set_all(False)
        obs_config.set_all_low_dim(True)
        obs_config.right_shoulder_camera.rgb = True
        action_mode = ActionMode()
        self.env = environment.Environment(
            action_mode, ASSET_DIR, obs_config, headless=True)
        demos = self.env.get_demos('reach_target', 2)
        self.assertEqual(len(demos), 2)
        self.assertGreater(len(demos[0]), 0)
        self.assertIsInstance(demos[0][0].right_shoulder_rgb, np.ndarray)
        self.assertIsNone(demos[0][0].left_shoulder_rgb)

    def test_get_live_demos(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        demos = task.get_demos(2, live_demos=True)
        self.assertEqual(len(demos), 2)
        self.assertGreater(len(demos[0]), 0)
        self.assertIsInstance(demos[0][0].right_shoulder_rgb, np.ndarray)

    def test_observation_shape_constant_across_demo(self):
        task = self.get_task(
            TakeLidOffSaucepan, ArmActionMode.ABS_JOINT_VELOCITY)
        demos = task.get_demos(1, live_demos=True)
        self.assertEqual(len(demos), 1)
        self.assertGreater(len(demos[0]), 0)
        self.assertGreater(demos[0][0].task_low_dim_state.size, 0)
        shapes = [step.task_low_dim_state.shape for step in demos[0]]
        first_shape = shapes[0]
        self.assertListEqual(shapes, [first_shape] * len(demos[0]))

    def test_reset_to_demos(self):
        task = self.get_task(
            TakeLidOffSaucepan, ArmActionMode.ABS_JOINT_VELOCITY)
        demo = task.get_demos(1, live_demos=True)[0]
        obs = demo[0]
        task.reset_to_demo(demo)
        reset_obs = task.get_observation()

        # Now check that the state has been properly restored
        np.testing.assert_allclose(
            reset_obs.joint_positions, obs.joint_positions, atol=1e-1)
        np.testing.assert_allclose(
            reset_obs.task_low_dim_state, obs.task_low_dim_state, atol=1e-1)
        np.testing.assert_allclose(
            reset_obs.gripper_open, obs.gripper_open, atol=1e-1)

    def test_action_mode_abs_joint_velocity(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_VELOCITY)
        task.reset()
        action = [0.1] * 7 + [1]
        obs, reward, term = task.step(action)
        [self.assertAlmostEqual(0.1, a, delta=0.05)
         for a in obs.joint_velocities]

    def test_action_mode_delta_joint_velocity(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.DELTA_JOINT_VELOCITY)
        task.reset()
        action = [-0.1] * 7 + [1]
        [task.step(action) for _ in range(2)]
        obs, reward, term = task.step(action)
        [self.assertAlmostEqual(-0.3, a, delta=0.1)
         for a in obs.joint_velocities]

    def test_action_mode_abs_joint_position(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_POSITION)
        _, obs = task.reset()
        init_angles = np.append(obs.joint_positions, 1.)  # for gripper
        target_angles = np.array(init_angles) + 0.05
        [task.step(target_angles) for _ in range(5)]
        obs, reward, term = task.step(target_angles)
        [self.assertAlmostEqual(a, actual, delta=0.05)
         for a, actual in zip(target_angles, obs.joint_positions)]

    def test_action_mode_delta_joint_position(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.DELTA_JOINT_POSITION)
        _, obs = task.reset()
        init_angles = obs.joint_positions
        target_angles = np.array(init_angles) + 0.06
        [task.step([0.01] * 7 + [1]) for _ in range(5)]
        obs, reward, term = task.step([0.01] * 8)
        [self.assertAlmostEqual(a, actual, delta=0.05)
         for a, actual in zip(target_angles, obs.joint_positions)]

    def test_action_mode_abs_ee_position(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_EE_POSE_WORLD_FRAME)
        _, obs = task.reset()
        init_pose = obs.gripper_pose
        new_pose = np.append(init_pose, 1.0)  # for gripper
        new_pose[2] -= 0.1  # 10cm down
        obs, reward, term = task.step(new_pose)
        [self.assertAlmostEqual(a, p, delta=0.01)
         for a, p in zip(new_pose, obs.gripper_pose)]

    def test_action_mode_delta_ee_position(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.DELTA_EE_POSE_WORLD_FRAME)
        _, obs = task.reset()
        init_pose = obs.gripper_pose
        new_pose = [0, 0, -0.1, 0, 0, 0, 1.0, 1.0]  # 10cm down
        expected_pose = list(init_pose)
        expected_pose[2] -= 0.1
        obs, reward, term = task.step(new_pose)
        [self.assertAlmostEqual(a, p, delta=0.01)
         for a, p in zip(expected_pose, obs.gripper_pose)]

    def test_action_mode_abs_ee_position_plan(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_EE_POSE_PLAN_WORLD_FRAME)
        _, obs = task.reset()
        init_pose = obs.gripper_pose
        new_pose = np.append(init_pose, 1.0)  # for gripper
        new_pose[2] -= 0.1  # 10cm down
        obs, reward, term = task.step(new_pose)
        [self.assertAlmostEqual(a, p, delta=0.001)
         for a, p in zip(new_pose, obs.gripper_pose)]

    def test_action_mode_delta_ee_position_plan(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.DELTA_EE_POSE_PLAN_WORLD_FRAME)
        _, obs = task.reset()
        init_pose = obs.gripper_pose
        new_pose = [0, 0, -0.1, 0, 0, 0, 1.0, 1.0]  # 10cm down
        expected_pose = list(init_pose)
        expected_pose[2] -= 0.1
        obs, reward, term = task.step(new_pose)
        [self.assertAlmostEqual(a, p, delta=0.001)
         for a, p in zip(expected_pose, obs.gripper_pose)]

    def test_action_mode_ee_position_ee_frame(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.EE_POSE_EE_FRAME)
        _, obs = task.reset()
        dummy = Dummy.create()
        dummy.set_position([0, 0, 0.05], relative_to=task._robot.arm.get_tip())
        action = [0, 0, 0.05, 0, 0, 0, 1, 1]
        obs, reward, term = task.step(action)
        [self.assertAlmostEqual(a, p, delta=0.01)
         for a, p in zip(dummy.get_position(), obs.gripper_pose[:3])]

    def test_action_mode_ee_position_plan_ee_frame(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.EE_POSE_PLAN_EE_FRAME)
        _, obs = task.reset()
        dummy = Dummy.create()
        dummy.set_position([0, 0, 0.05], relative_to=task._robot.arm.get_tip())
        action = [0, 0, 0.05, 0, 0, 0, 1, 1]
        obs, reward, term = task.step(action)
        [self.assertAlmostEqual(a, p, delta=0.01)
         for a, p in zip(dummy.get_position(), obs.gripper_pose[:3])]

    def test_action_mode_abs_joint_torque(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.ABS_JOINT_TORQUE)
        task.reset()
        action = [0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, 1.0]
        obs, reward, term = task.step(action)
        # Difficult to test given gravity, so just check for exceptions.

    def test_action_mode_delta_joint_torque(self):
        task = self.get_task(
            ReachTarget, ArmActionMode.DELTA_JOINT_TORQUE)
        _, obs = task.reset()
        init_forces = np.array(obs.joint_forces)
        action = [0.1, -0.1, 0.1, -0.1, 0.1, -0.1, 0.1, 1]
        expected = np.array([0.3, -0.3, 0.3, -0.3, 0.3, -0.3, 0.3])
        expected += init_forces
        [task.step(action) for _ in range(2)]
        obs, reward, term = task.step(action)
        # Difficult to test given gravity, so just check for exceptions.

    def test_swap_arm(self):
        # Checks if the environment can be setup with each arm
        action_mode = ActionMode(ArmActionMode.DELTA_JOINT_VELOCITY)
        for robot_config in environment.SUPPORTED_ROBOTS.keys():
            with self.subTest(robot_config=robot_config):
                self.env = environment.Environment(
                    action_mode, headless=True,
                    robot_configuration=robot_config)
                self.env.launch()
                self.env.shutdown()
