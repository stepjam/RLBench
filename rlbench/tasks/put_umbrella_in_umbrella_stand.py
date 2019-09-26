from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class PutUmbrellaInUmbrellaStand(Task):

    def init_task(self):
        success_sensor = ProximitySensor('success')
        umbrella = Shape('umbrella')
        self.register_graspable_objects([umbrella])
        self.register_success_conditions(
            [DetectedCondition(umbrella, success_sensor),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['put umbrella in umbrella stand'
                'pick up the umbrella and drop it in its stand',
                'use the handle to lift the umbrella and place it in its holder'
                'set the umbrella down inside the umbrella stand']

    def variation_count(self) -> int:
        return 1
