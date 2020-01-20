import unittest
import rlbench.backend.task as task
import os
from rlbench.backend.utils import task_file_to_task_class
from pyrep import PyRep
from pyrep.robots.arms.panda import Panda
from pyrep.robots.end_effectors.panda_gripper import PandaGripper
from rlbench.backend.const import TTT_FILE
from tools.task_validator import task_smoke, TaskValidationError
from rlbench.observation_config import ObservationConfig
from rlbench.backend.scene import Scene
from rlbench.backend.robot import Robot


TASKS = [t for t in os.listdir(task.TASKS_PATH)
         if t != '__init__.py' and t.endswith('.py')]
DIR_PATH = os.path.dirname(os.path.abspath(__file__))


class TestTasks(unittest.TestCase):
    """Tests all of the tasks via the task_validator tool but not testing demos.

    This is a lighter-weight test than the full demo smoke that is run in the
    `tasks` test directory.
    """

    def test_run_task_validator(self):
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

        for task_file in TASKS:
            test_name = task_file.split('.py')[0]
            with self.subTest(task=test_name):
                sim.start()
                task_class = task_file_to_task_class(task_file)
                active_task = task_class(sim, robot)
                try:
                    task_smoke(active_task, scene, variation=-1,
                               test_demos=False, max_variations=9999)
                except Exception as e:
                    sim.stop()
                    raise e
                sim.stop()
        sim.shutdown()
