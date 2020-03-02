from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, ConditionSet, \
    GraspedCondition


class TakeLidOffSaucepan(Task):

    def init_task(self) -> None:
        self.lid = Shape('saucepan_lid_grasp_point')
        success_detector = ProximitySensor('success')
        self.register_graspable_objects([self.lid])
        cond_set = ConditionSet([
            GraspedCondition(self.robot.gripper, self.lid),
            DetectedCondition(self.lid, success_detector)
        ])
        self.register_success_conditions([cond_set])

    def init_episode(self, index: int) -> List[str]:
        return ['take lid off the saucepan',
                'using the handle, lift the lid off of the pan',
                'remove the lid from the pan',
                'grip the saucepan\'s lid and remove it from the pan',
                'leave the pan open',
                'uncover the saucepan']

    def variation_count(self) -> int:
        return 1
