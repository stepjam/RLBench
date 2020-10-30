from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped, \
    DetectedSeveralCondition
from rlbench.backend.spawn_boundary import SpawnBoundary

GROCERY_NAMES = [
    'crackers',
    'chocolate jello',
    'strawberry jello',
    'soup',
    'spam',
    'mustard',
    'sugar',
]


class PutAllGroceriesInCupboard(Task):

    def init_task(self) -> None:
        self.groceries = [Shape(name.replace(' ', '_'))
                          for name in GROCERY_NAMES]
        self.grasp_points = [Dummy('%s_grasp_point' % name.replace(' ', '_'))
                             for name in GROCERY_NAMES]
        self.goals = [Dummy('goal_%s' % name.replace(' ', '_'))
                      for name in GROCERY_NAMES]
        self.waypoint1 = Dummy('waypoint1')
        self.waypoint3 = Dummy('waypoint3')
        self.register_graspable_objects(self.groceries)
        self.boundary = SpawnBoundary([Shape('groceries_boundary')])

        self.register_waypoint_ability_start(0, self._move_to_next_target)
        self.register_waypoint_ability_start(3, self._move_to_drop_zone)
        self.register_waypoints_should_repeat(self._repeat)
        self.groceries_to_place = len(GROCERY_NAMES)
        self.groceries_placed = 0

    def init_episode(self, index: int) -> List[str]:
        self.groceries_placed = 0
        self.boundary.clear()
        [self.boundary.sample(g, min_distance=0.15) for g in self.groceries]

        self.register_success_conditions(
            [DetectedSeveralCondition(
                self.groceries[:self.groceries_to_place],
                ProximitySensor('success'), self.groceries_to_place),
             NothingGrasped(self.robot.gripper)])
        return ['put all of the groceries in the cupboard',
                'pick up all of the groceries and place them in the cupboard',
                'move the groceries to the shelves',
                'put the groceries on the table into the cupboard',
                'put away the groceries in the cupboard']

    def variation_count(self) -> int:
        return 1

    def boundary_root(self) -> Object:
        return Shape('boundary_root')

    def is_static_workspace(self) -> bool:
        return True

    def _move_to_next_target(self, _):
        if self.groceries_placed >= self.groceries_to_place:
            raise RuntimeError('Should not be here.')
        self.waypoint1.set_pose(
            self.grasp_points[self.groceries_placed].get_pose())

    def _move_to_drop_zone(self, _):
        self.waypoint3.set_pose(
            self.goals[self.groceries_placed].get_pose())

    def _repeat(self):
        self.groceries_placed += 1
        return self.groceries_placed < self.groceries_to_place