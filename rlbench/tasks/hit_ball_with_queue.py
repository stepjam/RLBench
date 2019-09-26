from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, GraspedCondition, \
    ConditionSet


class HitBallWithQueue(Task):

    def init_task(self) -> None:
        queue = Shape('queue')
        success_sensor = ProximitySensor('success')
        ball = Shape('ball')
        self.register_graspable_objects([queue])

        cond_set = ConditionSet([
            GraspedCondition(self.robot.gripper, queue),
            DetectedCondition(ball, success_sensor)
        ], order_matters=True)
        self.register_success_conditions([cond_set])

    def init_episode(self, index: int) -> List[str]:
        return ['hit ball with queue in to the goal',
                'pot the ball in the goal',
                'pick up the que and use it to pot the ball into the goal']

    def variation_count(self) -> int:
        return 1
