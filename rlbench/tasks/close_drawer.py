from typing import List
from pyrep.objects.joint import Joint
from pyrep.objects.dummy import Dummy
from rlbench.backend.task import Task
from rlbench.backend.conditions import JointCondition

OPTIONS = ['bottom', 'middle', 'top']


class CloseDrawer(Task):

    def init_task(self) -> None:
        self.anchors = [Dummy('waypoint_anchor_%s' % opt)
                        for opt in OPTIONS]
        self.joints = [Joint('drawer_joint_%s' % opt)
                       for opt in OPTIONS]
        self.waypoint0 = Dummy('waypoint0')

    def init_episode(self, index: int) -> List[str]:
        option = OPTIONS[index]
        self.joints[index].set_joint_position(0.1)
        self.register_success_conditions(
            [JointCondition(self.joints[index], 0.06)])
        x, y, z = self.waypoint0.get_position()
        _, _, target_z = self.anchors[index].get_position()
        self.waypoint0.set_position([x, y, target_z])

        return ['close %s drawer' % (option,),
                'shut the %s drawer' % (option,),
                'slide the %s drawer shut' % (option,)]

    def variation_count(self) -> int:
        return 3
