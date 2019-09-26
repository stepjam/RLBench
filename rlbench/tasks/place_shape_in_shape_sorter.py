from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.spawn_boundary import SpawnBoundary
from rlbench.backend.conditions import DetectedCondition

SHAPE_NAMES = ['cube', 'cylinder', 'triangular prism', 'star', 'moon']


class PlaceShapeInShapeSorter(Task):

    def init_task(self) -> None:
        self.shape_sorter = Shape('shape_sorter')
        self.success_sensor = ProximitySensor('success')
        self.shapes = [Shape(ob.replace(' ', '_')) for ob in SHAPE_NAMES]
        self.drop_points = [
            Dummy('%s_drop_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.grasp_points = [
            Dummy('%s_grasp_point' % ob.replace(' ', '_'))
            for ob in SHAPE_NAMES]
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint4 = Dummy('waypoint4')
        self.register_graspable_objects(self.shapes)

        self.register_waypoint_ability_start(0, self._set_grasp)
        self.register_waypoint_ability_start(3, self._set_drop)
        self.boundary = SpawnBoundary([Shape('boundary')])

    def init_episode(self, index) -> List[str]:
        self.variation_index = index
        shape = SHAPE_NAMES[index]
        self.register_success_conditions(
            [DetectedCondition(self.shapes[index], self.success_sensor)])

        self.boundary.clear()
        [self.boundary.sample(s, min_distance=0.05) for s in self.shapes]

        return ['put the %s in the shape sorter' % shape,
                'pick up the %s and put it in the sorter' % shape,
                'place the %s into its slot in the shape sorter' % shape,
                'slot the %s into the shape sorter' % shape]

    def variation_count(self) -> int:
        return len(SHAPE_NAMES)

    def _set_grasp(self, _):
        gp = self.grasp_points[self.variation_index]
        self.waypoint1.set_pose(gp.get_pose())

    def _set_drop(self, _):
        dp = self.drop_points[self.variation_index]
        self.waypoint4.set_pose(dp.get_pose())
