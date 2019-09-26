from typing import List
import numpy as np
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.task import Task
from rlbench.backend.conditions import DetectedCondition


class PutRubbishInBin(Task):

    def init_task(self):
        success_sensor = ProximitySensor('success')
        self.rubbish = Shape('rubbish')
        self.register_graspable_objects([self.rubbish])
        self.register_success_conditions(
            [DetectedCondition(self.rubbish, success_sensor)])

    def init_episode(self, index: int) -> List[str]:
        tomato1 = Shape('tomato1')
        tomato2 = Shape('tomato2')
        x1, y1, z1 = tomato2.get_position()
        x2, y2, z2 = self.rubbish.get_position()
        x3, y3, z3 = tomato1.get_position()
        pos = np.random.randint(3)
        if pos == 0:
            self.rubbish.set_position([x1, y1, z2])
            tomato2.set_position([x2, y2, z1])
        elif pos == 2:
            self.rubbish.set_position([x3, y3, z2])
            tomato1.set_position([x2, y2, z3])

        return ['put rubbish in bin',
                'drop the rubbish into the bin',
                'pick up the rubbish and leave it in the trash can',
                'throw away the trash, leaving any other objects alone',
                'chuck way any rubbish on the table rubbish']

    def variation_count(self) -> int:
        return 1
