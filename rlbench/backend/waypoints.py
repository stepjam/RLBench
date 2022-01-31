from pyrep.const import ConfigurationPathAlgorithms as Algos
from pyrep.objects.object import Object
from pyrep.robots.configuration_paths.arm_configuration_path import (
    ArmConfigurationPath)
from rlbench.backend.robot import Robot


class Waypoint(object):

    def __init__(self, waypoint: Object, robot: Robot, ignore_collisions=False,
                 start_of_path_func=None, end_of_path_func=None):
        self._waypoint = waypoint
        self._robot = robot
        self._ext = waypoint.get_extension_string()
        self._ignore_collisions = ignore_collisions
        self._linear_only = False
        self._start_of_path_func = start_of_path_func
        self._end_of_path_func = end_of_path_func
        self.skip = False
        if len(self._ext) > 0:
            self._ignore_collisions = 'ignore_collision' in self._ext
            self._linear_only = 'linear' in self._ext

    def get_path(self, ignore_collisions=False) -> ArmConfigurationPath:
        raise NotImplementedError()

    def get_ext(self) -> str:
        return self._ext

    def get_waypoint_object(self) -> Object:
        return self._waypoint

    def remove(self) -> None:
        self._waypoint.remove()

    def start_of_path(self) -> None:
        if self._start_of_path_func is not None:
            self._start_of_path_func(self)

    def end_of_path(self) -> None:
        if self._end_of_path_func is not None:
            self._end_of_path_func(self)


class Point(Waypoint):

    def get_path(self, ignore_collisions=False) -> ArmConfigurationPath:
        arm = self._robot.arm
        if self._linear_only:
            path = arm.get_linear_path(self._waypoint.get_position(),
                                euler=self._waypoint.get_orientation(),
                                ignore_collisions=(self._ignore_collisions or
                                                   ignore_collisions))
        else:
            path = arm.get_path(self._waypoint.get_position(),
                                euler=self._waypoint.get_orientation(),
                                ignore_collisions=(self._ignore_collisions or
                                                   ignore_collisions),
                                trials=100,
                                max_configs=10,
                                trials_per_goal=10,
                                algorithm=Algos.RRTConnect)
        return path


class PredefinedPath(Waypoint):

    def get_path(self, ignore_collisions=False) -> ArmConfigurationPath:
        arm = self._robot.arm
        path = arm.get_path_from_cartesian_path(self._waypoint)
        return path
