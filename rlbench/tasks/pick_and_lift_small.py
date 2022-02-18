from typing import List
from pyrep.objects import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, ConditionSet, \
    GraspedCondition
from rlbench.backend.spawn_boundary import SpawnBoundary

SHAPE_NAMES = ['cube', 'cylinder', 'triangular prism', 'star', 'moon']


class PickAndLiftSmall(Task):

    def init_task(self) -> None:
        self._shapes = [Shape(ob.replace(' ', '_')) for ob in SHAPE_NAMES]
        self._grasp_points = [
            Dummy('%s_grasp_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self._w1 = Dummy('waypoint1')

        self.register_graspable_objects(self._shapes)
        self.boundary = SpawnBoundary([Shape('pick_and_lift_boundary')])
        self.success_detector = ProximitySensor('pick_and_lift_success')

    def init_episode(self, index: int) -> List[str]:
        shape = self._shapes[index]
        self.register_success_conditions([
            GraspedCondition(self.robot.gripper, shape),
            DetectedCondition(shape, self.success_detector)
        ])
        self.boundary.clear()
        self.boundary.sample(
            self.success_detector, min_rotation=(0.0, 0.0, 0.0),
            max_rotation=(0.0, 0.0, 0.0))
        for sh in self._shapes:
            self.boundary.sample(sh, min_distance=0.1)

        self._w1.set_pose(self._grasp_points[index].get_pose())

        return ['pick up the %s and lift it up to the target' %
                SHAPE_NAMES[index],
                'grasp the blue %s to the target' % SHAPE_NAMES[index],
                'lift the blue %s up to the target' % SHAPE_NAMES[index]]

    def variation_count(self) -> int:
        return len(SHAPE_NAMES)
