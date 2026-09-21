"""Count distinct cups rather than requiring an unmentioned cup-ID prefix."""
import itertools
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from rlbench.backend.conditions import DetectedCondition, OrConditions
from rlbench.tasks.place_cups import PlaceCups


class TestPlaceCupsConditions(unittest.TestCase):
    def test_all_sensor_states_and_variations(self):
        cups = [object() for _ in range(3)]
        gripper = Mock()
        task = PlaceCups(None, SimpleNamespace(gripper=gripper))
        task._cups = cups
        task._cups_boundary = object()
        sensors = [Mock() for _ in range(3)]
        task._on_peg_conditions = [OrConditions([
            DetectedCondition(cup, sensor) for sensor in sensors
        ]) for cup in cups]
        with patch('rlbench.tasks.place_cups.SpawnBoundary'):
            for variation in range(3):
                task.init_episode(variation)
                for bits in itertools.product((False, True), repeat=9):
                    matrix = [bits[i * 3:(i + 1) * 3] for i in range(3)]
                    for j, sensor in enumerate(sensors):
                        sensor.is_detected.side_effect = (
                            lambda cup, j=j: matrix[cups.index(cup)][j])
                    for held in (False, True):
                        gripper.get_grasped_objects.return_value = [cups[0]] if held else []
                        expected = not held and sum(map(any, matrix)) >= variation + 1
                        with self.subTest(variation=variation, bits=bits, held=held):
                            self.assertEqual(bool(task.success()[0]), expected)


if __name__ == '__main__':
    unittest.main()
