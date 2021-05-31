from typing import List, Tuple

from pyrep.objects import Dummy
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PutBooksOnBookshelf(Task):

    def init_task(self) -> None:
        self._success_sensor = ProximitySensor('success')
        self._books = [Shape('book2'), Shape('book1'), Shape('book0')]
        self._waypoints_idxs = [5, 11, -1]
        self.register_graspable_objects(self._books)

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions([
            DetectedCondition(
                b, self._success_sensor) for b in self._books[:index+1]
        ])
        self.register_stop_at_waypoint(self._waypoints_idxs[index])
        return ['put %d books on bookshelf' % (index + 1),
                'pick up %d books and place them on the top shelf' % (index + 1),
                'stack %d books up on the top shelf' % (index + 1)]

    def variation_count(self) -> int:
        return 3

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -3.14/2], [0.0, 0.0, 3.14/2]
