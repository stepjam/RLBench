import os
import re
from os.path import dirname, abspath, join
from typing import List, Tuple, Callable

import numpy as np
from pyrep import PyRep
from pyrep.const import ObjectType
from pyrep.errors import ConfigurationPathError
from pyrep.objects.cartesian_path import CartesianPath
from pyrep.objects.dummy import Dummy
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object

from rlbench.backend.conditions import Condition
from rlbench.backend.exceptions import WaypointError
from rlbench.backend.observation import Observation
from rlbench.backend.robot import Robot
from rlbench.backend.waypoints import Point, PredefinedPath, Waypoint

TASKS_PATH = join(dirname(abspath(__file__)), '../tasks')


class Task(object):

    def __init__(self, pyrep: PyRep, robot: Robot):
        """Constructor.

        :param pyrep: Instance of PyRep.
        :param robot: Instance of Robot.
        """
        self.pyrep = pyrep
        self.name = self.get_name()
        self.robot = robot
        self._waypoints = None
        self._success_conditions = []
        self._graspable_objects = []
        self._base_object = None
        self._waypoint_additional_inits = {}
        self._waypoint_abilities_start = {}
        self._waypoint_abilities_end = {}
        self._waypoints_should_repeat = lambda: False
        self._initial_objs_in_scene = None

    ########################
    # Overriding functions #
    ########################

    def init_task(self) -> None:
        """Initialises the task. Called only once when task loaded.

        Here we can grab references to objects in the task and store them
        as member variables to be used in init_episode. Here we also usually
        set success conditions for the task as well as register what objects
        can be grasped.
        """
        raise NotImplementedError(
            "'init_task' is almost always necessary.")

    def init_episode(self, index: int) -> List[str]:
        """Initialises the episode. Called each time the scene is reset.

        Here we usually define how the task changes across variations. Based on
        this we can change the task descriptions that are returned.

        :param index: The variation index.
        :return: A list of strings describing the task.
        """
        raise NotImplementedError(
            "'init_episode' must be defined and return a list of strings.")

    def variation_count(self) -> int:
        """Number of variations for the task. Can be determined dynamically.

        :return: Number of variations for this task.
        """
        raise NotImplementedError(
            "'variation_count' must be defined and return an int.")

    def get_low_dim_state(self) -> np.ndarray:
        """Gets the pose and various other properties of objects in the task.

        :return: 1D array of low-dimensional task state.
        """

        # Corner cases:
        # (1) Object has been deleted.
        # (2) Object has been grasped (and is now child of gripper).

        state = []
        for obj, objtype in self._initial_objs_in_scene:
            if not obj.still_exists():
                # It has been deleted
                empty_len = 7
                if objtype == ObjectType.JOINT:
                    empty_len += 1
                elif objtype == ObjectType.FORCE_SENSOR:
                    empty_len += 6
                state.extend(np.zeros((empty_len,)).tolist())
            else:
                state.extend(np.array(obj.get_pose()))
                if obj.get_type() == ObjectType.JOINT:
                    state.extend([Joint(obj.get_handle()).get_joint_position()])
                elif obj.get_type() == ObjectType.FORCE_SENSOR:
                    forces, torques = ForceSensor(obj.get_handle()).read()
                    state.extend(forces + torques)

        return np.array(state).flatten()

    def step(self) -> None:
        """Called each time the simulation is stepped. Can usually be left."""
        pass

    def cleanup(self) -> None:
        """Called at the end of the episode. Can usually be left.

        Can be used for complex tasks that spawn many objects.
        """
        pass

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        """Defines how much the task base can rotate during episode placement.

        Default is set such that it can rotate any amount on the z axis.

        :return: A tuple containing the min and max (x, y, z) rotation bounds
            (in radians).
        """
        return (0.0, 0.0, -3.14), (0.0, 0.0, 3.14)

    def boundary_root(self) -> Object:
        """An object that should act as the task root when randomly placing.

        Sometimes the task can involve a large appliance (e.g. dish washer)
        which cannot be placed within the task workspace. This allows us to
        define a proxy root (e.g. a plane in front of the appliance) that
        would allow the task to be placed inside the workspace.

        :return: The PyRep Object that will act as the root.
        """
        return self.get_base()

    def decorate_observation(self, observation: Observation) -> Observation:
        """Can be used for tasks that want to modify the observations.

        Usually not used. Perhpas cabn be used to model

        :param observation: The Observation for this time step.
        :return: The modified Observation.
        """
        return observation

    def is_static_workspace(self) -> bool:
        """Specify if the task should'nt be randomly placed in the workspace.

        :return: True if the task pose should not be sampled.
        """
        return False

    def set_initial_objects_in_scene(self):
        objs = self.get_base().get_objects_in_tree(
            exclude_base=True, first_generation_only=False)
        types = [ob.get_type() for ob in objs]
        self._initial_objs_in_scene = list(zip(objs, types))


    #########################
    # Registering functions #
    #########################

    def register_success_conditions(self, condition: List[Condition]):
        """What conditions need to be met for the task to be a success.

        Note: this replaces any previously registered conditions!

        :param condition: A list of success conditions.
        """
        self._success_conditions = condition

    def register_graspable_objects(self, objects: List[Object]):
        """Register what objects can be grasped with a 'stable' grasp.

        In order for objects to be grasped in a stable way, PyRep attaches an
        objects that need to be grasped as a child of the gripper. This function
        allows one to register a list of objects that can be grasped in
        this 'stable' manner.

        Note: this replaces any previously registered objects!

        :param objects: The list of Objects that can be grasped.
        """
        self._graspable_objects = objects

    def register_waypoint_ability_start(self, waypoint_index: int,
                                        func: Callable[[Waypoint], None]):
        """Register a function to be called before moving to waypoint.

        The registered function should take in a Waypoint object and is called
        when the robot is about to move to the assigned waypoint.


        :param waypoint_index: The waypoint index.
        :param func: A function that takes a Waypoint object.
        """
        self._waypoint_abilities_start[waypoint_index] = func

    def register_waypoint_ability_end(self, waypoint_index: int,
                                        func: Callable[[Waypoint], None]):
        """Register a function to be called after moving to waypoint.

        The registered function should take in a Waypoint object and is called
        when the robot has finished moving to the assigned waypoint.


        :param waypoint_index: The waypoint index.
        :param func: A function that takes a Waypoint object.
        """
        self._waypoint_abilities_end[waypoint_index] = func

    def register_waypoints_should_repeat(self, func: Callable[[], bool]):
        """Register a function that is called when reached the end of a demo.

        The registered function should return a bool if the demo should repeat.
        Can be used in conjunction with `register_waypoint_ability_start` and
        `register_waypoint_ability_end`. Useful for a task such as emptying a
        container, where we want to keep dynamically moving waypoints until
        the container is empty.

        :param func: A function that return a bool if the demo should repeat.
        """
        self._waypoints_should_repeat = func

    ##########################
    # Other public functions #
    ##########################

    def get_name(self) -> str:
        """The name of the task file (without the .py extension).

        :return: The name of the task.
        """
        return re.sub('(?<!^)(?=[A-Z])', '_', self.__class__.__name__).lower()

    def validate(self):
        """If the task placement is valid. """
        self._waypoints = self._get_waypoints()

    def get_waypoints(self):
        if self._waypoints is None:
            self._waypoints = self._get_waypoints()
        return self._waypoints

    def should_repeat_waypoints(self):
        return self._waypoints_should_repeat()

    def get_graspable_objects(self):
        return self._graspable_objects

    def success(self):
        all_met = True
        one_terminate = False
        for cond in self._success_conditions:
            met, terminate = cond.condition_met()
            all_met &= met
            one_terminate |= terminate
        return all_met, all_met

    def load(self) -> Object:
        if Object.exists(self.get_name()):
            return Dummy(self.get_name())
        ttm_file = os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            '../task_ttms/%s.ttm' % self.name)
        if not os.path.isfile(ttm_file):
            raise FileNotFoundError(
                'The following is not a valid task .ttm file: %s' % ttm_file)
        self._base_object = self.pyrep.import_model(ttm_file)
        return self._base_object

    def unload(self) -> None:
        self._waypoints = None
        self.get_base().remove()
        self.clear_registerings()

    def cleanup_(self) -> None:
        for cond in self._success_conditions:
            cond.reset()
        self._waypoints = None
        self.cleanup()

    def clear_registerings(self) -> None:
        self._success_conditions = []
        self._graspable_objects = []
        self._base_object = None
        self._waypoint_additional_inits = {}
        self._waypoint_abilities_start = {}
        self._waypoint_abilities_end = {}

    def get_base(self) -> Dummy:
        self._base_object = Dummy(self.get_name())
        return self._base_object

    def get_state(self) -> Tuple[bytes, int]:
        objs = self.get_base().get_objects_in_tree(exclude_base=False)
        return self.get_base().get_configuration_tree(), len(objs)

    def restore_state(self, state: Tuple[bytes, int]) -> None:
        objs = self.get_base().get_objects_in_tree(exclude_base=False)
        if len(objs) != state[1]:
            raise RuntimeError(
                'Expected to be resetting %d objects, but there were %d.' %
                (state[1], len(objs)))
        self.pyrep.set_configuration_tree(state[0])

    #####################
    # Private functions #
    #####################

    def _feasible(self, waypoints: List[Point]) -> Tuple[bool, int]:
        arm = self.robot.arm
        start_vals = arm.get_joint_positions()
        for i, point in enumerate(waypoints):
            try:
                path = point.get_path(ignore_collisions=True)
            except ConfigurationPathError as err:
                arm.set_joint_positions(start_vals)
                return False, i
            path.set_to_end(allow_force_mode=False)
        # Needed twice otherwise can glitch out.
        arm.set_joint_positions(start_vals, allow_force_mode=False)
        arm.set_joint_positions(start_vals)
        return True, -1

    def _get_waypoints(self, validating=False) -> List[Waypoint]:
        waypoint_name = 'waypoint%d'
        waypoints = []
        additional_waypoint_inits = []
        i = 0
        while True:
            name = waypoint_name % i
            if not Object.exists(name):
                # There are no more waypoints...
                break
            ob_type = Object.get_object_type(name)
            way = None
            if ob_type == ObjectType.DUMMY:
                waypoint = Dummy(name)
                start_func = None
                end_func = None
                if i in self._waypoint_abilities_start:
                    start_func = self._waypoint_abilities_start[i]
                if i in self._waypoint_abilities_end:
                    end_func = self._waypoint_abilities_end[i]
                way = Point(waypoint, self.robot,
                            start_of_path_func=start_func,
                            end_of_path_func=end_func)
            elif ob_type == ObjectType.PATH:
                cartestian_path = CartesianPath(name)
                way = PredefinedPath(cartestian_path, self.robot)
            else:
                raise WaypointError(
                    '%s is an unsupported waypoint type %s' % (
                        name, ob_type), self)

            if name in self._waypoint_additional_inits and not validating:
                additional_waypoint_inits.append(
                    (self._waypoint_additional_inits[name], way))
            waypoints.append(way)
            i += 1

        # Check if all of the waypoints are feasible
        feasible, way_i = self._feasible(waypoints)
        if not feasible:
            raise WaypointError(
                "Infeasible episode. Can't reach waypoint %d." % way_i, self)
        for func, way in additional_waypoint_inits:
            func(way)
        return waypoints
