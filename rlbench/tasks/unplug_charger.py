from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class UnplugCharger(Task):

    def init_task(self) -> None:
        charger_success = ProximitySensor('charger_success')
        charger = Shape('charger')
        self.register_graspable_objects([charger])
        self.register_success_conditions(
            [DetectedCondition(charger, charger_success)])

    def init_episode(self, index: int) -> List[str]:
        return ['unplug charger',
                'take the charger out of the wall',
                'grip the black charger and pull it out of its socket',
                'slide the plug out from the wall',
                'remove the charger from the mains',
                'get the charger from the wall plug']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]
