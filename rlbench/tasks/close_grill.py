from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class CloseGrill(Task):

    def init_task(self) -> None:
        self.lid = Shape('lid')
        self.register_success_conditions([
            DetectedCondition(self.lid, ProximitySensor('sensor_handle'),
                              negated=True),
            DetectedCondition(self.lid, ProximitySensor('sensor_grill_top'))
             ])

    def init_episode(self, index: int) -> List[str]:
        return ['close the grill',
                'grasp the handle and lower the grill cover to close it',
                'close the bbq',
                'close the barbecue']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -3.14/2), (0.0, 0.0, 3.14/2)

    def boundary_root(self) -> Object:
        return Shape('grill_root')
