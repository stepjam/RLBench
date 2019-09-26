from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class TakeUsbOutOfComputer(Task):

    def init_task(self) -> None:
        usb = Shape('usb')
        self.register_graspable_objects([usb])
        self.register_success_conditions(
            [DetectedCondition(usb, ProximitySensor('success'), negated=True),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['take usb out of computer',
                'remove the usb stick from its port',
                'retrieve the usb stick',
                'grasp the usb stick and slide it out of the pc',
                'get a hold of the usb stick and pull it out of the desktop '
                'computer']

    def variation_count(self) -> int:
        return 1
