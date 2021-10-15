from typing import List
from pyrep.objects.dummy import Dummy
from pyrep.objects.proximity_sensor import ProximitySensor
from pyrep.objects.shape import Shape
from rlbench.backend.conditions import NothingGrasped, DetectedCondition
from rlbench.backend.task import Task

MEAT = ['chicken', 'steak']


class MeatOnGrill(Task):

    def init_task(self) -> None:
        self._steak = Shape('steak')
        self._chicken = Shape('chicken')
        self._success_sensor = ProximitySensor('success')
        self.register_graspable_objects([self._chicken, self._steak])
        self._w1 = Dummy('waypoint1')
        self._w1z = self._w1.get_position()[2]

    def init_episode(self, index: int) -> List[str]:
        conditions = [NothingGrasped(self.robot.gripper)]
        if index == 0:
            x, y, _ = self._chicken.get_position()
            self._w1.set_position([x, y, self._w1z])
            conditions.append(
                DetectedCondition(self._chicken, self._success_sensor))
        else:
            x, y, _ = self._steak.get_position()
            self._w1.set_position([x, y, self._w1z])
            conditions.append(
                DetectedCondition(self._steak, self._success_sensor))
        self.register_success_conditions(conditions)
        return ['put the %s on the grill' % MEAT[index],
                'pick up the %s and place it on the grill' % MEAT[index],
                'grill the %s' % MEAT[index]]

    def variation_count(self) -> int:
        return 2
