import unittest
import rlbench.backend.task as task
import os
from rlbench.backend.utils import task_file_to_task_class
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from rlbench.backend.const import TTT_FILE
from tools.task_validator import task_smoke
from rlbench.observation_config import ObservationConfig
from rlbench.backend.scene import Scene
from rlbench.backend.robot import Robot


TASKS = [t for t in os.listdir(task.TASKS_PATH)
         if t != '__init__.py' and t.endswith('.py')]
DIR_PATH = os.path.dirname(os.path.abspath(__file__))

# Task does work, but fails demos often. These should eventually be improved.
FLAKY_TASKS = ['put_all_groceries_in_cupboard', 'take_cup_out_from_cabinet',
               'slide_cabinet_open_and_place_cups', 'slide_cabinet_open']


class TestTasks(unittest.TestCase):
    """Tests all of the tasks via the task_validator tool.

    Given that unit tests shouldn't take forever to run, we only limit
    each validation run to 1 variation. In practice, a newly created task
    should be validated for all variations. Despite this, the test still takes
    a while to run.
    """

    def test_run_task_validator(self):
        for task_file in TASKS:
            test_name = task_file.split('.py')[0]
            with self.subTest(task=test_name):
                if test_name in FLAKY_TASKS:
                    self.skipTest('Flaky task.')
                sim = PyRep()
                ttt_file = os.path.join(
                    DIR_PATH, '..', '..', 'rlbench', TTT_FILE)
                sim.launch(ttt_file, headless=True)
                sim.step_ui()
                sim.set_simulation_timestep(50.0)
                sim.step_ui()
                sim.start()
                robot = Robot(Panda(), PandaGripper())
                obs = ObservationConfig()
                obs.set_all(False)
                scene = Scene(sim, robot, obs)
                sim.start()
                task_class = task_file_to_task_class(task_file)
                active_task = task_class(sim, robot)
                try:
                    task_smoke(active_task, scene, variation=-1,
                               max_variations=2, success=0.25)
                except Exception as e:
                    sim.stop()
                    sim.shutdown()
                    raise e
                sim.stop()
                sim.shutdown()
