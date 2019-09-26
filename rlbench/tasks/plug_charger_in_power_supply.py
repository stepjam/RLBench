from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary


class PlugChargerInPowerSupply(Task):

    def init_task(self) -> None:
        sensor_right_hole = ProximitySensor('sensor1')
        sensor_left_hole = ProximitySensor('sensor2')
        charger = Shape('charger')
        self.charger_base = Dummy('charger_base')
        charger_right_stick = Shape('detector1')
        charger_left_stick = Shape('detector2')
        self.boundary = SpawnBoundary([Shape('charger_boundary')])
        self.register_graspable_objects([charger])
        self.register_success_conditions(
            [DetectedCondition(charger_right_stick, sensor_right_hole),
             DetectedCondition(charger_left_stick, sensor_left_hole),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        self.boundary.clear()
        self.boundary.sample(self.charger_base)
        return ['plug charger in power supply',
                'pick up the charger and plug in it',
                'plug the charger into the mains',
                'lift the charger up to the wall and plug it in',
                'plug the charger into the wall']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 4.], [0, 0, 3.14 / 4.]
