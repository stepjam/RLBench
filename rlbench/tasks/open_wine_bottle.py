from typing import List
import numpy as np
from pyrep.objects.force_sensor import ForceSensor
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.conditions import JointCondition


class OpenWineBottle(Task):

    def init_task(self) -> None:
        cap_detector = ProximitySensor("cap_detector")
        self.joint = Joint('joint')
        self.force_sensor = ForceSensor('Force_sensor')
        self.cap = Shape('cap')
        self.register_success_conditions(
            [DetectedCondition(self.cap, cap_detector, negated=True)])
        self.cap_turned_condition = JointCondition(
            self.joint, np.deg2rad(150))

    def init_episode(self, index: int) -> List[str]:
        self.cap.set_parent(self.force_sensor)
        self.cap_turned = False
        return ['open wine bottle',
                'screw open the wine bottle',
                'unscrew the bottle cap then remove it from the wine bottle']

    def variation_count(self) -> int:
        return 1

    def step(self) -> None:
        if not self.cap_turned:
            self.cap_turned = self.cap_turned_condition.condition_met()[0]
            if self.cap_turned:
                self.cap.set_parent(self.joint)

    def cleanup(self):
        self.cap.set_parent(self.force_sensor)
