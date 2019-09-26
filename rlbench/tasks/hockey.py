from typing import List
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, GraspedCondition
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor


class Hockey(Task):

    def init_task(self) -> None:
        stick = Shape('hockey_stick')
        self.register_success_conditions([
            DetectedCondition(Shape('hockey_ball'),
                              ProximitySensor('success')),
            GraspedCondition(self.robot.gripper, stick)])
        self.register_graspable_objects([stick])

    def init_episode(self, index: int) -> List[str]:
        return ['hit the ball into the net',
                'use the stick to push the hockey ball into the goal',
                'pick up the hockey stick, then swing at the ball in the '
                'direction of the net',
                'score a hockey goal',
                'grasping one end of the hockey stick, swing it such that the '
                'other end collides with the ball such that the ball goes '
                'into the goal']

    def variation_count(self) -> int:
        return 1
