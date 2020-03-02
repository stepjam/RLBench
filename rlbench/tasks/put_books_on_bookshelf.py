from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PutBooksOnBookshelf(Task):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        book0 = Shape('book0')
        book1 = Shape('book1')
        book2 = Shape('book2')
        self.register_success_conditions(
            [DetectedCondition(book0, success_sensor),
             DetectedCondition(book1, success_sensor),
             DetectedCondition(book2, success_sensor),
             NothingGrasped(self.robot.gripper)])
        self.register_graspable_objects([book2, book1, book0])

    def init_episode(self, index: int) -> List[str]:
        return ['put books on bookshelf',
                'pick up the books and place them on the top shelf',
                'stack the books up on the top shelf']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, -3.14/2], [0.0, 0.0, 3.14/2]
