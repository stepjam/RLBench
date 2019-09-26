from typing import List
from pyrep import PyRep
from pyrep.const import RenderMode
from pyrep.errors import ConfigurationPathError
from pyrep.objects.shape import Shape
from pyrep.objects.vision_sensor import VisionSensor
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.observation import Observation
from rlbench.backend.exceptions import (
    WaypointError, BoundaryError, NoWaypointsError, DemoError)
from rlbench.observation_config import ObservationConfig, CameraConfig
from rlbench.backend.task import Task
from rlbench.backend.robot import Robot
import numpy as np

STEPS_BEFORE_EPISODE_START = 10


class Scene(object):
    """Controls what is currently in the vrep scene. This is used for making
    sure that the tasks are easily reachable. This may be just replaced by
    environment. Responsible for moving all the objects. """

    def __init__(self, pyrep: PyRep, robot: Robot,
                 obs_config=ObservationConfig()):
        self._pyrep = pyrep
        self._robot = robot
        self._obs_config = obs_config
        self._active_task = None
        self._inital_task_state = None
        self._start_arm_joint_pos = robot.arm.get_joint_positions()
        self._starting_gripper_joint_pos = robot.gripper.get_joint_positions()
        self._workspace = Shape('workspace')
        self._workspace_boundary = SpawnBoundary([self._workspace])
        self._cam_over_shoulder_left = VisionSensor('cam_over_shoulder_left')
        self._cam_over_shoulder_right = VisionSensor('cam_over_shoulder_right')
        self._cam_wrist = VisionSensor('cam_wrist')
        self._has_init_task = self._has_init_episode = False
        self._variation_index = 0

        self._initial_robot_state = (robot.arm.get_configuration_tree(),
                                     robot.gripper.get_configuration_tree())

        # Set camera properties from observation config
        self._cam_shoulder_left_mask = None
        self._cam_shoulder_right_mask = None
        self._cam_wrist_mask = None
        self._set_camera_properties()

    def load(self, task: Task) -> None:
        """Loads the task and positions at the centre of the workspace.

        :param task: The task to load in the scene.
        """
        task.load()  # Load the task in to the scene

        # Set at the centre of the workspace
        task.get_base().set_position(self._workspace.get_position())

        self._inital_task_state = task.get_state()
        self._active_task = task
        self._initial_task_pose = task.boundary_root().get_orientation()
        self._has_init_task = self._has_init_episode = False
        self._variation_index = 0

    def unload(self) -> None:
        """Clears the scene. i.e. removes all tasks. """
        if self._active_task is not None:
            self._robot.gripper.release()
            self._active_task.unload()
        self._active_task = None
        self._variation_index = 0

    def init_task(self) -> None:
        self._active_task.init_task()
        self._has_init_task = True
        self._variation_index = 0

    def init_episode(self, index: int, randomly_place: bool=True,
                     max_attempts: int = 5) -> List[str]:
        """Calls the task init_episode and puts randomly in the workspace.
        """

        self._variation_index = index

        if not self._has_init_task:
            self.init_task()

        # Try a few times to init and place in the workspace
        attempts = 0
        descriptions = None
        while attempts < max_attempts:
            descriptions = self._active_task.init_episode(index)
            try:
                if (randomly_place and
                        not self._active_task.is_static_workspace()):
                    self._place_task()
                self._active_task.validate()
                break
            except (BoundaryError, WaypointError) as e:
                self._active_task.cleanup_()
                attempts += 1
                if attempts >= max_attempts:
                    raise e

        # Let objects come to rest
        [self._pyrep.step() for _ in range(STEPS_BEFORE_EPISODE_START)]
        self._has_init_episode = True
        return descriptions

    def reset(self) -> None:
        """Resets the joint angles. """
        self._robot.gripper.release()

        arm, gripper = self._initial_robot_state
        self._pyrep.set_configuration_tree(arm)
        self._pyrep.set_configuration_tree(gripper)
        self._robot.arm.set_joint_positions(self._start_arm_joint_pos)
        self._robot.arm.set_joint_target_velocities(
            [0] * len(self._robot.arm.joints))
        self._robot.gripper.set_joint_positions(
            self._starting_gripper_joint_pos)
        self._robot.gripper.set_joint_target_velocities(
            [0] * len(self._robot.gripper.joints))

        if self._active_task is not None and self._has_init_task:
            self._active_task.cleanup_()
            self._active_task.restore_state(self._inital_task_state)
        [self._pyrep.step_ui() for _ in range(20)]

    def get_observation(self) -> Observation:
        tip = self._robot.arm.get_tip()

        joint_forces = None
        if self._obs_config.joint_forces:
            fs = self._robot.arm.get_joint_forces()
            vels = self._robot.arm.get_joint_target_velocities()
            joint_forces = self._obs_config.joint_forces_noise.apply(
                np.array([-f if v < 0 else f for f, v in zip(fs, vels)]))

        ee_forces_flat = None
        if self._obs_config.gripper_touch_forces:
            ee_forces = self._robot.gripper.get_touch_sensor_forces()
            ee_forces_flat = []
            for eef in ee_forces:
                ee_forces_flat.extend(eef)
            ee_forces_flat = np.array(ee_forces_flat)

        lsc_ob = self._obs_config.left_shoulder_camera
        rsc_ob = self._obs_config.right_shoulder_camera
        wc_ob = self._obs_config.wrist_camera

        obs = Observation(
            left_shoulder_rgb=(
                lsc_ob.rgb_noise.apply(
                    self._cam_over_shoulder_left.capture_rgb())
                if lsc_ob.rgb else None),
            left_shoulder_depth=(
                lsc_ob.depth_noise.apply(
                    self._cam_over_shoulder_left.capture_depth())
                if lsc_ob.depth else None),
            right_shoulder_rgb=(
                rsc_ob.rgb_noise.apply(
                    self._cam_over_shoulder_right.capture_rgb())
                if rsc_ob.rgb else None),
            right_shoulder_depth=(
                rsc_ob.depth_noise.apply(
                    self._cam_over_shoulder_right.capture_depth())
                if rsc_ob.depth else None),
            wrist_rgb=(
                wc_ob.rgb_noise.apply(self._cam_wrist.capture_rgb())
                if wc_ob.rgb else None),
            wrist_depth=(
                wc_ob.depth_noise.apply(self._cam_wrist.capture_depth())
                if wc_ob.depth else None),

            left_shoulder_mask=(
                self._cam_shoulder_left_mask.capture_rgb()
                if lsc_ob.mask else None),
            right_shoulder_mask=(
                self._cam_shoulder_right_mask.capture_rgb()
                if rsc_ob.mask else None),
            wrist_mask=(
                self._cam_wrist_mask.capture_rgb()
                if wc_ob.mask else None),

            joint_velocities=(
                self._obs_config.joint_velocities_noise.apply(
                    np.array(self._robot.arm.get_joint_velocities()))
                if self._obs_config.joint_velocities else None),
            joint_positions=(
                self._obs_config.joint_positions_noise.apply(
                    np.array(self._robot.arm.get_joint_positions()))
                if self._obs_config.joint_positions else None),
            joint_forces=joint_forces,
            gripper_open_amount=(
                1.0 if self._robot.gripper.get_open_amount()[0] > 0.9 else 0.0),
            gripper_pose=(
                np.array(tip.get_pose())
                if self._obs_config.gripper_pose else None),
            gripper_touch_forces=ee_forces_flat,
            gripper_joint_positions=np.array(
                self._robot.gripper.get_joint_positions()),
            task_low_dim_state=(
                self._active_task.get_low_dim_state() if
                self._obs_config.task_low_dim_state else None))
        obs = self._active_task.decorate_observation(obs)
        return obs

    def step(self):
        self._pyrep.step()
        self._active_task.step()

    def get_demo(self, record: bool=True, func=None,
                 randomly_place: bool=True) -> List[Observation]:
        """Returns a demo (list of observations)"""

        if not self._has_init_task:
            self.init_task()
        if not self._has_init_episode:
            self.init_episode(self._variation_index,
                              randomly_place=randomly_place)
        self._has_init_episode = False

        waypoints = self._active_task.get_waypoints()
        if len(waypoints) == 0:
            raise NoWaypointsError(
                'No waypoints were found.', self._active_task)

        demo = []
        if record:
            self._pyrep.step()  # Need this here or get_force doesn't work...
            demo.append(self.get_observation())
        while True:
            success = False
            for i, point in enumerate(waypoints):

                point.start_of_path()
                try:
                    path = point.get_path()
                except ConfigurationPathError as e:
                    raise DemoError(
                        'Could not get a path for waypoint %d.' % i,
                        self._active_task) from e
                ext = point.get_ext()
                path.visualize()

                done = False
                success = False
                while not done:
                    done = path.step()
                    self.step()
                    self._demo_record_step(demo, record, func)
                    success, term = self._active_task.success()
                    if success:
                        break

                point.end_of_path()

                path.clear_visualization()

                if success:
                    # We can quit early because we have finished the task
                    break

                # TODO: need to decide how I do the gripper actions
                if len(ext) > 0:
                    contains_param = False
                    start_of_bracket = -1
                    gripper = self._robot.gripper
                    if 'open_gripper(' in ext:
                        gripper.release()
                        start_of_bracket = ext.index('open_gripper(') + 13
                        contains_param = ext[start_of_bracket] != ')'
                        if not contains_param:
                            done = False
                            while not done:
                                done = gripper.actuate(1.0, 0.04)
                                self._pyrep.step()
                                self._active_task.step()
                                if self._obs_config.record_gripper_closing:
                                    self._demo_record_step(demo, record, func)
                    elif 'close_gripper(' in ext:
                        start_of_bracket = ext.index('close_gripper(') + 14
                        contains_param = ext[start_of_bracket] != ')'
                        if not contains_param:
                            done = False
                            while not done:
                                done = gripper.actuate(0.0, 0.04)
                                self._pyrep.step()
                                self._active_task.step()
                                if self._obs_config.record_gripper_closing:
                                    self._demo_record_step(demo, record, func)

                    if contains_param:
                        rest = ext[start_of_bracket:]
                        num = float(rest[:rest.index(')')])
                        done = False
                        while not done:
                            done = gripper.actuate(num, 0.04)
                            self._pyrep.step()
                            self._active_task.step()
                            if self._obs_config.record_gripper_closing:
                                self._demo_record_step(demo, record, func)

                    if 'close_gripper(' in ext:
                        for g_obj in self._active_task.get_graspable_objects():
                            gripper.grasp(g_obj)

                    self._demo_record_step(demo, record, func)

            if not self._active_task.should_repeat_waypoints() or success:
                break

        # Some tasks may need additional physics steps
        # (e.g. ball rowling to goal)
        if not success:
            for _ in range(10):
                self._pyrep.step()
                self._active_task.step()
                self._demo_record_step(demo, record, func)
                success, term = self._active_task.success()
                if success:
                    break

        success, term = self._active_task.success()
        if not success:
            raise DemoError('Demo was completed, but was not successful.',
                            self._active_task)

        return demo

    def get_observation_config(self) -> ObservationConfig:
        return self._obs_config

    def _demo_record_step(self, demo_list, record, func):
        if record:
            demo_list.append(self.get_observation())
        if func is not None:
            func()

    def _set_camera_properties(self) -> None:
        def _set_props(cam: VisionSensor, rgb: bool, depth: bool,
                       conf: CameraConfig):
            if not (rgb or depth):
                cam.remove()
            else:
                cam.set_resolution(conf.image_size)
                cam.set_render_mode(conf.render_mode)
        _set_props(
            self._cam_over_shoulder_left,
            self._obs_config.left_shoulder_camera.rgb,
            self._obs_config.left_shoulder_camera.depth,
            self._obs_config.left_shoulder_camera)
        _set_props(
            self._cam_over_shoulder_right,
            self._obs_config.right_shoulder_camera.rgb,
            self._obs_config.right_shoulder_camera.depth,
            self._obs_config.right_shoulder_camera)
        _set_props(
            self._cam_wrist, self._obs_config.wrist_camera.rgb,
            self._obs_config.wrist_camera.depth,
            self._obs_config.wrist_camera)

        if self._obs_config.left_shoulder_camera.mask:
            self._cam_shoulder_left_mask = self._cam_over_shoulder_left.copy()
            self._cam_shoulder_left_mask.set_render_mode(
                RenderMode.OPENGL_COLOR_CODED)
        if self._obs_config.right_shoulder_camera.mask:
            self._cam_shoulder_right_mask = self._cam_over_shoulder_right.copy()
            self._cam_shoulder_right_mask.set_render_mode(
                RenderMode.OPENGL_COLOR_CODED)
        if self._obs_config.wrist_camera.mask:
            self._cam_wrist_mask = self._cam_wrist.copy()
            self._cam_wrist_mask.set_render_mode(
                RenderMode.OPENGL_COLOR_CODED)

    def _place_task(self) -> None:
        self._workspace_boundary.clear()
        # Find a place in the robot workspace for task
        self._active_task.boundary_root().set_orientation(
            self._initial_task_pose)
        min_rot, max_rot = self._active_task.base_rotation_bounds()
        self._workspace_boundary.sample(
            self._active_task.boundary_root(),
            min_rotation=min_rot, max_rotation=max_rot)
