import unittest
from rlbench.tasks import (FS10_V1, FS25_V1, FS50_V1, FS95_V1, MT15_V1,
                           MT30_V1, MT55_V1, MT100_V1)


FS_V1 = [
    (FS10_V1, 10, 5),
    (FS25_V1, 25, 5),
    (FS50_V1, 50, 5),
    (FS95_V1, 95, 5)]

MT_V1 = [
    (MT15_V1, 15),
    (MT30_V1, 30),
    (MT55_V1, 55),
    (MT100_V1, 100)]


class TestTaskSet(unittest.TestCase):

    def test_fs_v1(self):
        for ts, train, test in FS_V1:
            with self.subTest(task_set='FS%d_V1' % train):
                self.assertEqual(len(ts['train']), train)
                self.assertEqual(len(ts['test']), test)
                # Test no duplicates
                self.assertEqual(len(ts['train'] + ts['test']),
                                 len(set(ts['train'] + ts['test'])))
                self.assertFalse(any(i in ts['test'] for i in ts['train']))

    def test_mt_v1(self):
        for ts, train in MT_V1:
            with self.subTest(task_set='MT%d_V1' % train):
                self.assertEqual(len(ts['train']), train)
                # Test no duplicates
                self.assertEqual(len(ts['train']), len(set(ts['train'])))


