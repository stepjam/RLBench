import os
from typing import Type
import numpy as np
from absl import app
from absl import flags
from pyrep.objects.dummy import Dummy
from pyrep.objects.vision_sensor import VisionSensor

from rlbench import Environment
from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.backend.observation import Observation
from rlbench.backend.task import TASKS_PATH
from rlbench.backend.task import Task
from rlbench.backend.utils import task_file_to_task_class
from rlbench.observation_config import ObservationConfig
from rlbench.sim2real.domain_randomization import RandomizeEvery, \
    VisualRandomizationConfig

FLAGS = flags.FLAGS

flags.DEFINE_string(
    'save_dir', '/tmp/rlbench_videos/',
    'Where to save the generated videos.')
flags.DEFINE_list(
    'tasks', [], 'The tasks to record. If empty, all tasks are recorded.')
flags.DEFINE_boolean(
    'individual', True, 'One long clip of all the tasks, or individual videos.')
flags.DEFINE_boolean(
    'domain_randomization', False, 'If domain randomization should be applied.')
flags.DEFINE_string(
    'textures_path', '',
    'Where to locate textures if using domain randomization.')
flags.DEFINE_boolean('headless', True, 'Run in headless mode.')
flags.DEFINE_list(
    'camera_resolution', [1280, 720], 'The camera resolution')


class CameraMotion(object):
    def __init__(self, cam: VisionSensor):
        self.cam = cam

    def step(self):
        raise NotImplementedError()

    def save_pose(self):
        self._prev_pose = self.cam.get_pose()

    def restore_pose(self):
        self.cam.set_pose(self._prev_pose)


class CircleCameraMotion(CameraMotion):

    def __init__(self, cam: VisionSensor, origin: Dummy, speed: float):
        super().__init__(cam)
        self.origin = origin
        self.speed = speed  # in radians

    def step(self):
        self.origin.rotate([0, 0, self.speed])


class TaskRecorder(object):

    def __init__(self, env: Environment, cam_motion: CameraMotion, fps=30):
        self._env = env
        self._cam_motion = cam_motion
        self._fps = fps
        self._snaps = []
        self._current_snaps = []

    def take_snap(self, obs: Observation):
        self._cam_motion.step()
        self._current_snaps.append(
            (self._cam_motion.cam.capture_rgb() * 255.).astype(np.uint8))

    def record_task(self, task: Type[Task]):
        task = self._env.get_task(task)
        self._cam_motion.save_pose()
        while True:
            try:
                task.get_demos(
                    1, live_demos=True, callable_each_step=self.take_snap,
                    max_attempts=1)
                break
            except RuntimeError:
                self._cam_motion.restore_pose()
                self._current_snaps = []
        self._snaps.extend(self._current_snaps)
        self._current_snaps = []
        return True

    def save(self, path):
        print('Converting to video ...')
        os.makedirs(os.path.dirname(path), exist_ok=True)
        # OpenCV QT version can conflict with PyRep, so import here
        import cv2
        video = cv2.VideoWriter(
                path, cv2.VideoWriter_fourcc('m', 'p', '4', 'v'), self._fps,
                tuple(self._cam_motion.cam.get_resolution()))
        for image in self._snaps:
            video.write(cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        video.release()
        self._snaps = []


def main(argv):

    obs_config = ObservationConfig(record_gripper_closing=True)
    obs_config.set_all(False)

    vrc = rand_every = None
    frequency = 0
    if FLAGS.domain_randomization:
        vrc = VisualRandomizationConfig(FLAGS.textures_path)
        rand_every = RandomizeEvery.TRANSITION
        frequency = 10

    action_mode = MoveArmThenGripper(
        arm_action_mode=JointVelocity(), gripper_action_mode=Discrete())
    env = Environment(action_mode, obs_config=obs_config,
                      randomize_every=rand_every, frequency=frequency,
                      visual_randomization_config=vrc, headless=FLAGS.headless)
    env.launch()

    # Add the camera to the scene
    cam_placeholder = Dummy('cam_cinematic_placeholder')
    cam = VisionSensor.create(FLAGS.camera_resolution)
    cam.set_pose(cam_placeholder.get_pose())
    cam.set_parent(cam_placeholder)

    cam_motion = CircleCameraMotion(cam, Dummy('cam_cinematic_base'), 0.005)
    tr = TaskRecorder(env, cam_motion, fps=30)

    if len(FLAGS.tasks) > 0:
        task_names = FLAGS.tasks
    else:
        task_names = [t.split('.py')[0] for t in os.listdir(TASKS_PATH)
                      if t != '__init__.py' and t.endswith('.py')]
    task_classes = [task_file_to_task_class(
        task_file) for task_file in task_names]

    for i, (name, cls) in enumerate(zip(task_names, task_classes)):
        good = tr.record_task(cls)
        if FLAGS.individual and good:
            tr.save(os.path.join(FLAGS.save_dir, '%s.avi' % name))

    if not FLAGS.individual:
        tr.save(os.path.join(FLAGS.save_dir, 'recorded_tasks.mp4'))
    env.shutdown()


if __name__ == '__main__':
    app.run(main)
