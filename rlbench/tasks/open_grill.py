from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.object import Object
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class OpenGrill(Task):

    def init_task(self) -> None:
        lid = Shape('lid')
        self.register_success_conditions([
            DetectedCondition(lid, ProximitySensor('sensor_handle')),
            # DetectedCondition(lid, ProximitySensor('sensor_grill_top'),
            #                   negated=True)
             ])

    def init_episode(self, index: int) -> List[str]:
        return ['open the grill',
                'grasp tha handle and raise the cover up to open the grill',
                'open the bbq',
                'open the barbecue']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[Tuple[float, float, float],
                                            Tuple[float, float, float]]:
        return (0.0, 0.0, -3.14/2), (0.0, 0.0, 3.14/2)

    def boundary_root(self) -> Object:
        return Shape('grill_root')
