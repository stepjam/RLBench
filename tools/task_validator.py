from rlbench.backend.task import Task
from rlbench.backend.scene import DemoError
from rlbench.observation_config import ObservationConfig
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from rlbench.backend.const import TTT_FILE
from rlbench.backend.scene import Scene
from rlbench.backend.utils import task_file_to_task_class
from rlbench.backend.task import TASKS_PATH
from rlbench.backend.robot import Robot
import numpy as np
import os
import argparse

DEMO_ATTEMPTS = 5
MAX_VARIATIONS = 100


class TaskValidationError(Exception):
    pass


def task_smoke(task: Task, scene: Scene, variation=-1, demos=4, success=0.50,
               max_variations=3, test_demos=True):
    # -1 variations for all.

    print('Running task validator on task: %s' % task.get_name())

    # Loading
    scene.load(task)

    # Number of variations
    variation_count = task.variation_count()
    if variation_count < 0:
        raise TaskValidationError(
            "The method 'variation_count' should return a number > 0.")

    if variation_count > MAX_VARIATIONS:
        raise TaskValidationError(
            "This task had %d variations. Currently the limit is set to %d" %
            (variation_count, MAX_VARIATIONS))

    # Base rotation bounds
    base_pos, base_ori = task.base_rotation_bounds()
    if len(base_pos) != 3 or len(base_ori) != 3:
        raise TaskValidationError(
            "The method 'base_rotation_bounds' should return a tuple "
            "containing a list of floats.")

    # Boundary root
    root = task.boundary_root()
    if not root.still_exists():
        raise TaskValidationError(
            "The method 'boundary_root' should return a Dummy that is the root "
            "of the task.")

    def variation_smoke(i):

        print('Running task validator on variation: %d' % i)

        attempt_result = False
        failed_demos = 0
        for j in range(DEMO_ATTEMPTS):
            failed_demos = run_demos(i)
            attempt_result = (failed_demos / float(demos) <= 1. - success)
            if attempt_result:
                break
            else:
                print('Failed on attempt %d. Trying again...' % j)

        # Make sure we don't fail too often
        if not attempt_result:
            raise TaskValidationError(
                "Too many failed demo runs. %d of %d demos failed." % (
                    failed_demos, demos))
        else:
            print('Variation %d of task %s is good!' % (i, task.get_name()))
            if test_demos:
                print('%d of %d demos were successful.' % (
                    demos - failed_demos, demos))

    def run_demos(variation_num):
        fails = 0
        for dr in range(demos):
            try:
                scene.reset()
                desc = scene.init_episode(variation_num, max_attempts=10)
                if not isinstance(desc, list) or len(desc) <= 0:
                    raise TaskValidationError(
                        "The method 'init_variation' should return a list of "
                        "string descriptions.")
                if test_demos:
                    demo = scene.get_demo(record=True)
                    assert len(demo) > 0
            except DemoError as e:
                fails += 1
                print(e)
            except Exception as e:
                # TODO: check that we don't fall through all of these cases
                fails += 1
                print(e)

        return fails

    variations_to_test = [variation]
    if variation < 0:
        variations_to_test = list(range(
            np.minimum(variation_count, max_variations)))

    # Task set-up
    scene.init_task()
    [variation_smoke(i) for i in variations_to_test]


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("task", help="The task file to test.")
    args = parser.parse_args()

    python_file = os.path.join(TASKS_PATH, args.task)
    if not os.path.isfile(python_file):
        raise RuntimeError('Could not find the task file: %s' % python_file)

    task_class = task_file_to_task_class(args.task)

    DIR_PATH = os.path.dirname(os.path.abspath(__file__))
    sim = PyRep()
    ttt_file = os.path.join(
        DIR_PATH, '..', 'rlbench', TTT_FILE)
    sim.launch(ttt_file, headless=True)
    sim.step_ui()
    sim.set_simulation_timestep(0.005)
    sim.step_ui()
    sim.start()

    robot = Robot(Panda(), PandaGripper())

    active_task = task_class(sim, robot)
    obs = ObservationConfig()
    obs.set_all(False)
    scene = Scene(sim, robot, obs)
    try:
        task_smoke(active_task, scene, variation=2)
    except TaskValidationError as e:
        sim.shutdown()
        raise e
    sim.shutdown()
    print('Validation successful!')
