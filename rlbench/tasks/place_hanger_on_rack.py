from typing import List
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped
from rlbench.backend.spawn_boundary import SpawnBoundary


class PlaceHangerOnRack(Task):

    def init_task(self) -> None:
        self.hanger_holder = Shape('hanger_holder')
        hanger = Shape('clothes_hanger')
        success_detector = ProximitySensor('success_detector0')
        self.register_success_conditions([
            NothingGrasped(self.robot.gripper),
            DetectedCondition(hanger, success_detector)
        ])
        self.register_graspable_objects([hanger])
        self.workspace_boundary = SpawnBoundary([Shape('workspace')])

    def init_episode(self, index: int) -> List[str]:
        self.workspace_boundary.clear()
        self.workspace_boundary.sample(self.hanger_holder)
        return ['pick up the hanger and place in on the rack'
                'put the hanger on the rack',
                'hang the hanger up',
                'grasp the hanger and hang it up']

    def variation_count(self) -> int:
        return 1

    def is_static_workspace(self) -> bool:
        return True
