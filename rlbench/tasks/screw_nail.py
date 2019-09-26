from typing import List, Tuple
from pyrep.objects.shape import Shape
from pyrep.objects.joint import Joint
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition, GraspedCondition, \
    ConditionSet


class ScrewNail(Task):

    def init_task(self) -> None:
        screw_driver = Shape('screw_driver')
        self.block = Shape('block')
        self.register_graspable_objects([screw_driver])
        screw_joint = Joint('screw_joint')

        cond_set = ConditionSet([
            GraspedCondition(self.robot.gripper, screw_driver),
            JointCondition(screw_joint, 1.4)],  # about 90 degrees
            order_matters=True)
        self.register_success_conditions([cond_set])

    def init_episode(self, index: int) -> List[str]:
        return ['screw the nail in to the block',
                'pick up the screwdriver and screw in the protruding nail',
                'grasping the screw driver by its handle, slide its tip into '
                'the groove on the end of the nail, and rotate the screw driver'
                ' in an anti clockwise direction around the vertical axis',
                'lift the screwdriver up by its handle, lower its tip into the '
                'protruding nail, and screw in the nail']

    def variation_count(self) -> int:
        return 1

    def base_rotation_bounds(self) -> Tuple[List[float], List[float]]:
        return [0, 0, -3.14 / 2.], [0, 0, 3.14 / 2.]
