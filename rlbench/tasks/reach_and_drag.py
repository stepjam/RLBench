from typing import List
from rlbench.backend.task import Task
from rlbench.const import colors
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from pyrep.objects.shape import Shape
from pyrep.objects.proximity_sensor import ProximitySensor


class ReachAndDrag(Task):

    def init_task(self) -> None:
        self.stick = Shape('stick')
        self.register_graspable_objects([self.stick])
        self.cube = Shape('cube')
        self.target = Shape('target0')

    def init_episode(self, index: int) -> List[str]:
        self.register_success_conditions([
            DetectedCondition(self.cube, ProximitySensor('success0'))])
        color_name, color_rgb = colors[index]
        self.target.set_color(color_rgb)
        return ['use the stick to drag the cube onto the %s target'
                % color_name,
                'pick up the stick and use it to push or pull the cube '
                'onto the %s target' % color_name,
                'drag the block towards the %s square on the table top'
                % color_name,
                'grasping the stick by one end, pick it up and use the its '
                'other end to move the block onto the %s target' % color_name]

    def variation_count(self) -> int:
        return len(colors)
