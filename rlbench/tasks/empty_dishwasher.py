from typing import List, Tuple
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class EmptyDishwasher(Task):

    def init_task(self) -> None:
        success_detector = ProximitySensor('success')
        plate = Shape('dishwasher_plate')
        self.register_graspable_objects([plate])
        self.register_success_conditions(
            [DetectedCondition(plate, success_detector, negated=True)])

    def init_episode(self, index: int) -> List[str]:
        return ['empty the dishwasher', 'take dishes out of dishwasher',
                'open the  dishwasher door, slide the rack out and remove the '
                'dishes']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 2.], [0, 0, 3.14 / 2.]

    def boundary_root(self) -> Object:
        return Shape('boundary_root')
