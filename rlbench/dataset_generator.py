import argparse
import os
import pickle
from multiprocessing import Manager, Process

import numpy as np
from PIL import Image
from pyrep.const import RenderMode

import rlbench.backend.task as task
from rlbench import ObservationConfig
from rlbench.action_modes.action_mode import MoveArmThenGripper
from rlbench.action_modes.arm_action_modes import JointVelocity
from rlbench.action_modes.gripper_action_modes import Discrete
from rlbench.backend import utils
from rlbench.backend.const import *
from rlbench.backend.utils import task_file_to_task_class
from rlbench.environment import Environment


def check_and_make(dir):
    if not os.path.exists(dir):
        os.makedirs(dir)


def save_demo(demo, example_path):

    # Save image data first, and then None the image data, and pickle
    left_shoulder_rgb_path = os.path.join(
        example_path, LEFT_SHOULDER_RGB_FOLDER)
    left_shoulder_depth_path = os.path.join(
        example_path, LEFT_SHOULDER_DEPTH_FOLDER)
    left_shoulder_mask_path = os.path.join(
        example_path, LEFT_SHOULDER_MASK_FOLDER)
    right_shoulder_rgb_path = os.path.join(
        example_path, RIGHT_SHOULDER_RGB_FOLDER)
    right_shoulder_depth_path = os.path.join(
        example_path, RIGHT_SHOULDER_DEPTH_FOLDER)
    right_shoulder_mask_path = os.path.join(
        example_path, RIGHT_SHOULDER_MASK_FOLDER)
    overhead_rgb_path = os.path.join(
        example_path, OVERHEAD_RGB_FOLDER)
    overhead_depth_path = os.path.join(
        example_path, OVERHEAD_DEPTH_FOLDER)
    overhead_mask_path = os.path.join(
        example_path, OVERHEAD_MASK_FOLDER)
    wrist_rgb_path = os.path.join(example_path, WRIST_RGB_FOLDER)
    wrist_depth_path = os.path.join(example_path, WRIST_DEPTH_FOLDER)
    wrist_mask_path = os.path.join(example_path, WRIST_MASK_FOLDER)
    front_rgb_path = os.path.join(example_path, FRONT_RGB_FOLDER)
    front_depth_path = os.path.join(example_path, FRONT_DEPTH_FOLDER)
    front_mask_path = os.path.join(example_path, FRONT_MASK_FOLDER)

    check_and_make(left_shoulder_rgb_path)
    check_and_make(left_shoulder_depth_path)
    check_and_make(left_shoulder_mask_path)
    check_and_make(right_shoulder_rgb_path)
    check_and_make(right_shoulder_depth_path)
    check_and_make(right_shoulder_mask_path)
    check_and_make(overhead_rgb_path)
    check_and_make(overhead_depth_path)
    check_and_make(overhead_mask_path)
    check_and_make(wrist_rgb_path)
    check_and_make(wrist_depth_path)
    check_and_make(wrist_mask_path)
    check_and_make(front_rgb_path)
    check_and_make(front_depth_path)
    check_and_make(front_mask_path)

    for i, obs in enumerate(demo):
        left_shoulder_rgb = Image.fromarray(obs.left_shoulder_rgb)
        left_shoulder_depth = utils.float_array_to_rgb_image(
            obs.left_shoulder_depth, scale_factor=DEPTH_SCALE)
        left_shoulder_mask = Image.fromarray(
            (obs.left_shoulder_mask * 255).astype(np.uint8))
        right_shoulder_rgb = Image.fromarray(obs.right_shoulder_rgb)
        right_shoulder_depth = utils.float_array_to_rgb_image(
            obs.right_shoulder_depth, scale_factor=DEPTH_SCALE)
        right_shoulder_mask = Image.fromarray(
            (obs.right_shoulder_mask * 255).astype(np.uint8))
        overhead_rgb = Image.fromarray(obs.overhead_rgb)
        overhead_depth = utils.float_array_to_rgb_image(
            obs.overhead_depth, scale_factor=DEPTH_SCALE)
        overhead_mask = Image.fromarray(
            (obs.overhead_mask * 255).astype(np.uint8))
        wrist_rgb = Image.fromarray(obs.wrist_rgb)
        wrist_depth = utils.float_array_to_rgb_image(
            obs.wrist_depth, scale_factor=DEPTH_SCALE)
        wrist_mask = Image.fromarray((obs.wrist_mask * 255).astype(np.uint8))
        front_rgb = Image.fromarray(obs.front_rgb)
        front_depth = utils.float_array_to_rgb_image(
            obs.front_depth, scale_factor=DEPTH_SCALE)
        front_mask = Image.fromarray((obs.front_mask * 255).astype(np.uint8))

        left_shoulder_rgb.save(
            os.path.join(left_shoulder_rgb_path, IMAGE_FORMAT % i))
        left_shoulder_depth.save(
            os.path.join(left_shoulder_depth_path, IMAGE_FORMAT % i))
        left_shoulder_mask.save(
            os.path.join(left_shoulder_mask_path, IMAGE_FORMAT % i))
        right_shoulder_rgb.save(
            os.path.join(right_shoulder_rgb_path, IMAGE_FORMAT % i))
        right_shoulder_depth.save(
            os.path.join(right_shoulder_depth_path, IMAGE_FORMAT % i))
        right_shoulder_mask.save(
            os.path.join(right_shoulder_mask_path, IMAGE_FORMAT % i))
        overhead_rgb.save(
            os.path.join(overhead_rgb_path, IMAGE_FORMAT % i))
        overhead_depth.save(
            os.path.join(overhead_depth_path, IMAGE_FORMAT % i))
        overhead_mask.save(
            os.path.join(overhead_mask_path, IMAGE_FORMAT % i))
        wrist_rgb.save(os.path.join(wrist_rgb_path, IMAGE_FORMAT % i))
        wrist_depth.save(os.path.join(wrist_depth_path, IMAGE_FORMAT % i))
        wrist_mask.save(os.path.join(wrist_mask_path, IMAGE_FORMAT % i))
        front_rgb.save(os.path.join(front_rgb_path, IMAGE_FORMAT % i))
        front_depth.save(os.path.join(front_depth_path, IMAGE_FORMAT % i))
        front_mask.save(os.path.join(front_mask_path, IMAGE_FORMAT % i))

        # We save the images separately, so set these to None for pickling.
        obs.left_shoulder_rgb = None
        obs.left_shoulder_depth = None
        obs.left_shoulder_point_cloud = None
        obs.left_shoulder_mask = None
        obs.right_shoulder_rgb = None
        obs.right_shoulder_depth = None
        obs.right_shoulder_point_cloud = None
        obs.right_shoulder_mask = None
        obs.overhead_rgb = None
        obs.overhead_depth = None
        obs.overhead_point_cloud = None
        obs.overhead_mask = None
        obs.wrist_rgb = None
        obs.wrist_depth = None
        obs.wrist_point_cloud = None
        obs.wrist_mask = None
        obs.front_rgb = None
        obs.front_depth = None
        obs.front_point_cloud = None
        obs.front_mask = None

    # Save the low-dimension data
    with open(os.path.join(example_path, LOW_DIM_PICKLE), 'wb') as f:
        pickle.dump(demo, f)


def run(i, lock, task_index, variation_count, results, file_lock, tasks, args):
    """Each thread will choose one task and variation, and then gather
    all the episodes_per_task for that variation."""

    # Initialise each thread with random seed
    np.random.seed(None)
    num_tasks = len(tasks)

    img_size = list(map(int, args.image_size))

    obs_config = ObservationConfig()
    obs_config.set_all(True)
    obs_config.right_shoulder_camera.image_size = img_size
    obs_config.left_shoulder_camera.image_size = img_size
    obs_config.overhead_camera.image_size = img_size
    obs_config.wrist_camera.image_size = img_size
    obs_config.front_camera.image_size = img_size

    # Store depth as 0 - 1
    obs_config.right_shoulder_camera.depth_in_meters = False
    obs_config.left_shoulder_camera.depth_in_meters = False
    obs_config.overhead_camera.depth_in_meters = False
    obs_config.wrist_camera.depth_in_meters = False
    obs_config.front_camera.depth_in_meters = False

    # We want to save the masks as rgb encodings.
    obs_config.left_shoulder_camera.masks_as_one_channel = False
    obs_config.right_shoulder_camera.masks_as_one_channel = False
    obs_config.overhead_camera.masks_as_one_channel = False
    obs_config.wrist_camera.masks_as_one_channel = False
    obs_config.front_camera.masks_as_one_channel = False

    if args.renderer == 'opengl':
        obs_config.right_shoulder_camera.render_mode = RenderMode.OPENGL
        obs_config.left_shoulder_camera.render_mode = RenderMode.OPENGL
        obs_config.overhead_camera.render_mode = RenderMode.OPENGL
        obs_config.wrist_camera.render_mode = RenderMode.OPENGL
        obs_config.front_camera.render_mode = RenderMode.OPENGL
    elif args.renderer == 'opengl3':
        obs_config.right_shoulder_camera.render_mode = RenderMode.OPENGL3
        obs_config.left_shoulder_camera.render_mode = RenderMode.OPENGL3
        obs_config.overhead_camera.render_mode = RenderMode.OPENGL3
        obs_config.wrist_camera.render_mode = RenderMode.OPENGL3
        obs_config.front_camera.render_mode = RenderMode.OPENGL3

    rlbench_env = Environment(
        action_mode=MoveArmThenGripper(JointVelocity(), Discrete()),
        obs_config=obs_config,
        arm_max_velocity=args.arm_max_velocity,
        arm_max_acceleration=args.arm_max_acceleration,
        headless=True)
    rlbench_env.launch()

    task_env = None

    tasks_with_problems = results[i] = ''

    while True:
        # Figure out what task/variation this thread is going to do
        with lock:

            if task_index.value >= num_tasks:
                print('Process', i, 'finished')
                break

            my_variation_count = variation_count.value
            t = tasks[task_index.value]
            task_env = rlbench_env.get_task(t)
            var_target = task_env.variation_count()
            if args.variations >= 0:
                var_target = np.minimum(args.variations, var_target)
            if my_variation_count >= var_target:
                # If we have reached the required number of variations for this
                # task, then move on to the next task.
                variation_count.value = my_variation_count = 0
                task_index.value += 1

            variation_count.value += 1
            if task_index.value >= num_tasks:
                print('Process', i, 'finished')
                break
            t = tasks[task_index.value]

        task_env = rlbench_env.get_task(t)
        task_env.set_variation(my_variation_count)
        descriptions, _ = task_env.reset()

        variation_path = os.path.join(
            args.save_path, task_env.get_name(),
            VARIATIONS_FOLDER % my_variation_count)

        check_and_make(variation_path)

        with open(os.path.join(
                variation_path, VARIATION_DESCRIPTIONS), 'wb') as f:
            pickle.dump(descriptions, f)

        episodes_path = os.path.join(variation_path, EPISODES_FOLDER)
        check_and_make(episodes_path)

        abort_variation = False
        for ex_idx in range(args.episodes_per_task):
            print('Process', i, '// Task:', task_env.get_name(),
                  '// Variation:', my_variation_count, '// Demo:', ex_idx)
            attempts = 10
            while attempts > 0:
                try:
                    # TODO: for now we do the explicit looping.
                    demo, = task_env.get_demos(
                        amount=1,
                        live_demos=True)
                except Exception as e:
                    attempts -= 1
                    if attempts > 0:
                        continue
                    problem = (
                        'Process %d failed collecting task %s (variation: %d, '
                        'example: %d). Skipping this task/variation.\n%s\n' % (
                            i, task_env.get_name(), my_variation_count, ex_idx,
                            str(e))
                    )
                    print(problem)
                    tasks_with_problems += problem
                    abort_variation = True
                    break
                episode_path = os.path.join(episodes_path, EPISODE_FOLDER % ex_idx)
                with file_lock:
                    save_demo(demo, episode_path)
                break
            if abort_variation:
                break

    results[i] = tasks_with_problems
    rlbench_env.shutdown()


def parse_args():
    parser = argparse.ArgumentParser(description="RLBench Dataset Generator")
    parser.add_argument('--save_path', type=str, default='/tmp/rlbench_data/', help='Where to save the demos.')
    parser.add_argument('--tasks', nargs='*', default=[], help='The tasks to collect. If empty, all tasks are collected.')
    parser.add_argument('--image_size', nargs=2, type=int, default=[128, 128], help='The size of the images to save.')
    parser.add_argument('--renderer', type=str, choices=['opengl', 'opengl3'], default='opengl3', help='The renderer to use. opengl does not include shadows, but is faster.')
    parser.add_argument('--processes', type=int, default=1, help='The number of parallel processes during collection.')
    parser.add_argument('--episodes_per_task', type=int, default=10, help='The number of episodes to collect per task.')
    parser.add_argument('--variations', type=int, default=-1, help='Number of variations to collect per task. -1 for all.')
    parser.add_argument('--arm_max_velocity', type=float, default=1.0, help='Max arm velocity used for motion planning.')
    parser.add_argument('--arm_max_acceleration', type=float, default=4.0, help='Max arm acceleration used for motion planning.')
    return parser.parse_args()


def main():
    args = parse_args()

    task_files = [t.replace('.py', '') for t in os.listdir(task.TASKS_PATH)
                  if t != '__init__.py' and t.endswith('.py')]

    if len(args.tasks) > 0:
        for t in args.tasks:
            if t not in task_files:
                raise ValueError('Task %s not recognised!.' % t)
        task_files = args.tasks

    tasks = [task_file_to_task_class(t) for t in task_files]

    manager = Manager()

    result_dict = manager.dict()
    file_lock = manager.Lock()

    task_index = manager.Value('i', 0)
    variation_count = manager.Value('i', 0)
    lock = manager.Lock()

    check_and_make(args.save_path)

    processes = [Process(
        target=run, args=(
            i, lock, task_index, variation_count, result_dict, file_lock,
            tasks, args))
        for i in range(args.processes)]
    [t.start() for t in processes]
    [t.join() for t in processes]

    print('Data collection done!')
    for i in range(args.processes):
        print(result_dict[i])


if __name__ == '__main__':
    main()
