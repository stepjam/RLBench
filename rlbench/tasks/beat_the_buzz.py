from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped


class BeatTheBuzz(Task):

    def init_task(self) -> None:
        middle_sensor = ProximitySensor('middle_sensor')
        right_sensor = ProximitySensor('right_sensor')
        wand = Shape('wand')
        self.register_graspable_objects([wand])
        self.register_fail_conditions(
            [DetectedCondition(wand, middle_sensor)])
        self.register_success_conditions(
            [DetectedCondition(wand, right_sensor),
             NothingGrasped(self.robot.gripper)])

    def init_episode(self, index: int) -> List[str]:
        return ['beat the buzz',
                'slide the ring along the pole without allowing them to touch',
                'slide the ring along the metal pole, avoiding contact between '
                'them',
                'slide the ring from one end of the pole to the other without '
                'allowing them to touch',
                'slide the ring from one end of the pole to the other, '
                'avoiding contact between them',
                'move the ring from one end of the pole to the other, '
                'maintaining a gap between them',
                'move the ring from end of the pole to the other whilst '
                'maintaining separation between them']

    def variation_count(self) -> int:
        return 1
