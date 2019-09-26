from typing import List
from pyrep.objects.shape import Shape
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition, NothingGrasped

OPTIONS = ['black', 'red', 'purple']


class TakePlateOffColoredDishRack(Task):

    def init_task(self) -> None:
        self.plate = Shape('plate')
        self.success_poses = [Dummy('success_pos%d' % i)
                              for i in range(3)]
        self.register_graspable_objects([self.plate])
        self.register_success_conditions([
            DetectedCondition(
                self.plate, ProximitySensor('success_source'), negated=True),
            DetectedCondition(self.plate, ProximitySensor('success_target')),
            NothingGrasped(self.robot.gripper)
        ])

    def init_episode(self, index: int) -> List[str]:
        option = OPTIONS[index]
        target_pos = self.success_poses[index]
        x, y, _ = target_pos.get_position()
        _, _, z = self.plate.get_position()
        self.plate.set_position([x, y, z])
        return ['take plate off the %s colored rack' % option,
                'remove the dish from the %s rack' % option,
                'find the dish placed between the %s spokes of the dish rack '
                'and drop it on the table' % option,
                'grasp the plate from the %s part of the dish rack, lift it up'
                ' off of the rack, and leave it on the table top' % option,
                'move the plate from the %s rack to the table\'s surface'
                % option,
                'lift the plate up off the %s colored rack and drop it onto the'
                ' table from a considerable height' % option]

    def variation_count(self) -> int:
        return 3
