from typing import List
import math
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.robots.end_effectors.gripper import Gripper


class Condition(object):
    def condition_met(self):
        raise NotImplementedError()

    def reset(self):
        # Used if the conditions store any state.
        pass


class ColorCondition(object):
    def _init_(self, shape: Shape, success_rgb: List[float]):
        self.shape = shape
        self.success_rgb = success_rgb

    def condition_met(self):
        obj_rgb = self.shape.get_color()
        met = (obj_rgb == self.success_rgb)
        return met, False


class JointCondition(Condition):
    def __init__(self, joint: Joint, position: float):
        """in radians if revoloute, or meters if prismatic"""
        self._joint = joint
        self._original_pos = joint.get_joint_position()
        self._pos = position

    def condition_met(self):
        met = math.fabs(
            self._joint.get_joint_position() - self._original_pos) > self._pos
        return met, False


class DetectedCondition(Condition):
    def __init__(self, obj: Object, detector: ProximitySensor,
                 negated: bool = False):
        self._obj = obj
        self._detector = detector
        self._negated = negated

    def condition_met(self):
        met = self._detector.is_detected(self._obj)
        if self._negated:
            met = not met
        return met, False


class NothingGrasped(Condition):
    def __init__(self, gripper: Gripper):
        self._gripper = gripper

    def condition_met(self):
        met = len(self._gripper.get_grasped_objects()) == 0
        return met, False


class GraspedCondition(Condition):
    def __init__(self, gripper: Gripper, object: Object):
        self._gripper = gripper
        self._object_handle = object.get_handle()

    def condition_met(self):
        met = len([ob for ob in self._gripper.get_grasped_objects()
                   if self._object_handle == ob.get_handle()]) > 0
        return met, False


class DetectedSeveralCondition(Condition):
    def __init__(self, objects: List[Object], detector: ProximitySensor,
                 number_needed: int):
        self._objects = objects
        self._detector = detector
        self._number_needed = number_needed

    def condition_met(self):
        count = 0
        for ob in self._objects:
            if self._detector.is_detected(ob):
                count += 1
        met = False
        if count >= self._number_needed:
            met = True
        return met, False


class EmptyCondition(Condition):

    def __init__(self, container: list):
        self._container = container

    def condition_met(self):
        met = len(self._container) == 0
        return met, False


class FollowCondition(Condition):

    def __init__(self, obj: Object, points: list,
                 relative_to: Object = None, delta_limit: float = 0.01,
                 start_after_first: bool = True):
        self._obj = obj
        self._ponts = points
        self._relative_to = relative_to
        self._delta_limit = delta_limit
        self._start_after_first = start_after_first
        self._index = 0
        self._strikes = 0

    def condition_met(self):
        pos = self._obj.get_position(self._relative_to)
        first = True
        for i in range(self._index, len(self._ponts)):
            p = self._ponts[i]
            dist = math.sqrt((pos[0] - p[0]) ** 2 +
                             (pos[1] - p[1]) ** 2 +
                             (pos[2] - p[2]) ** 2)
            # Check we aren't too far away from the next point
            if dist > self._delta_limit:
                # Check if we are ignoring until we reach the first point
                if first and not (self._start_after_first and self._index == 0):
                    # If it is the first point, then we have failed
                    if self._strikes > 3:
                        return False, True
                    self._strikes += 1
                # Otherwise we have looped at least once and we may
                # still be following the path
                return False, False
            # Only advance if we are super close
            elif dist > self._delta_limit * 0.5:
                return False, False
            self._index += 1
            self._strikes = 0
            first = False
        return True, False


class ConditionSet(Condition):
    def __init__(self, conditions: List[Condition], order_matters: bool = False,
                 simultaneously_met: bool = True):
        """alternative would be sequentially met"""
        self._conditions = conditions
        self._order_matters = order_matters
        self._simultaneously_met = simultaneously_met  # Probably wont use
        self._current_condition_index = 0

    def condition_met(self):
        met = True
        # term = False
        if self._order_matters:
            if self._current_condition_index < len(self._conditions):
                for cond in self._conditions[self._current_condition_index:]:
                    ismet, term = cond.condition_met()
                    if not ismet:
                        break
                    self._current_condition_index += 1
                # Check again to see if we have now completed the order
                met = self._current_condition_index >= len(self._conditions)
        else:
            for cond in self._conditions:
                ismet, term = cond.condition_met()
                met &= ismet
                # if term:
                #     break
        return met, False

    def reset(self):
        self._current_condition_index = 0


class OrConditions(Condition):
    def __init__(self, conditions: List[Condition]):
        """Logical or over all conditions"""
        self._conditions = conditions

    def condition_met(self):
        met = False
        for cond in self._conditions:
            ismet, term = cond.condition_met()
            met |= ismet
            if met:
                break
        return met, False

    def reset(self):
        self._current_condition_index = 0