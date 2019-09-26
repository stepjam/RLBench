from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class TakeFrameOffHanger(Task):

    def init_task(self) -> None:
        frame = Shape('frame')
        self.register_graspable_objects([frame])
        self.register_success_conditions(
            [DetectedCondition(frame, ProximitySensor('hanger_detector'),
                               negated=True),
             DetectedCondition(frame, ProximitySensor('success')),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['take frame off hanger',
                'slide the photo off of the hanger and set it down on the '
                'table',
                'grab a hold of the frame, remove it from the hanger and put it'
                ' down',
                'grasping the picture frame, take it off the wall and place it'
                'on the table top',
                'take the picture down',
                'remove the photo frame from the wall']

    def variation_count(self) -> int:
        return 1
