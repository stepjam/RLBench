from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class HangFrameOnHanger(Task):

    def init_task(self) -> None:
        success_sensor = ProximitySensor('success')
        frame = Shape('frame')
        frame_detector = Shape('frame_detector')
        self.register_graspable_objects([frame])
        self.register_success_conditions(
            [DetectedCondition(frame_detector, success_sensor),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['hang frame on hanger',
                'hang the picture up',
                'hang the picture frame up on the wall',
                'place the frame on the hanger']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
