"""Unit tests for rlbench.backend.spawn_boundary.

These tests do not require CoppeliaSim/PyRep: the module under test only
uses pyrep for type annotations, so the `pyrep` package is stubbed when it
is not importable, and `spawn_boundary.py` / `exceptions.py` are loaded
directly from the repository. This keeps the tests runnable in any
environment with numpy installed.
"""
import importlib.util
import os
import sys
import types
import unittest

import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
BOUNDARY_PATH = os.path.join(
    REPO_ROOT, 'rlbench', 'backend', 'spawn_boundary.py')
EXCEPTIONS_PATH = os.path.join(
    REPO_ROOT, 'rlbench', 'backend', 'exceptions.py')

# Boundary with bbox (min_x, max_x, min_y, max_y, min_z, max_z) => not a plane.
BOUNDARY_BBOX = (-1.0, 1.0, -1.0, 1.0, 0.0, 1.0)
# A small object that always fits within the boundary, including the worst
# case where any rotation about the origin is applied to it.
OBJ_BBOX = (-0.05, 0.05, -0.05, 0.05, 0.48, 0.52)
# An object that can never fit within the boundary.
HUGE_OBJ_BBOX = (-10.0, 10.0, -10.0, 10.0, -10.0, 10.0)


def _load_from_file(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


def _stub_missing_dependencies():
    """Make spawn_boundary.py importable without pyrep/gymnasium.

    Real modules are always preferred: we only register lightweight stubs
    when the real imports fail (i.e. PyRep/CoppeliaSim is not available),
    so CI machines with PyRep installed are unaffected.
    """
    try:
        import pyrep  # noqa: F401
    except Exception:
        pyrep_mod = types.ModuleType('pyrep')
        objects_mod = types.ModuleType('pyrep.objects')
        object_mod = types.ModuleType('pyrep.objects.object')

        class Object(object):
            pass

        object_mod.Object = Object
        pyrep_mod.objects = objects_mod
        objects_mod.object = object_mod
        sys.modules.setdefault('pyrep', pyrep_mod)
        sys.modules.setdefault('pyrep.objects', objects_mod)
        sys.modules.setdefault('pyrep.objects.object', object_mod)

    try:
        import rlbench.backend.exceptions  # noqa: F401
    except Exception:
        # Loads the real exceptions.py standalone and registers it so that
        # `from rlbench.backend.exceptions import BoundaryError` resolves
        # without importing the rlbench package (which needs gymnasium).
        _load_from_file('rlbench.backend.exceptions', EXCEPTIONS_PATH)


_stub_missing_dependencies()
_spawn_boundary = _load_from_file(
    'rlbench_backend_spawn_boundary_under_test', BOUNDARY_PATH)
SpawnBoundary = _spawn_boundary.SpawnBoundary
BoundaryError = _spawn_boundary.BoundaryError


class FakeObject(object):
    """Duck-typed stand-in for a pyrep Object (boundary or placed object)."""

    def __init__(self, bbox, is_model=False, collides=False):
        self._bbox = bbox
        self._is_model = is_model
        self._collides = collides
        self.position = None
        self.rotation = None

    def is_model(self):
        return self._is_model

    def get_model_bounding_box(self):
        return self._bbox

    def get_bounding_box(self):
        return self._bbox

    def get_position(self, relative_to=None):
        return np.array([0.0, 0.0, 0.0])

    def set_position(self, position, relative_to=None):
        self.position = np.array(position, dtype=float)

    def rotate(self, rotation):
        self.rotation = list(rotation)

    def get_objects_in_tree(self, exclude_base=True,
                            first_generation_only=False):
        return [self]

    def check_collision(self, other):
        return self._collides


class TestSpawnBoundaryDeterminism(unittest.TestCase):

    def test_same_seed_produces_identical_placements(self):
        boundary_a, boundary_b = FakeObject(BOUNDARY_BBOX), FakeObject(
            BOUNDARY_BBOX)
        sb_a = SpawnBoundary([boundary_a], rng=np.random.default_rng(7))
        sb_b = SpawnBoundary([boundary_b], rng=np.random.default_rng(7))
        obj_a, obj_b = FakeObject(OBJ_BBOX), FakeObject(OBJ_BBOX)

        sb_a.sample(obj_a)
        sb_b.sample(obj_b)

        self.assertIsNotNone(obj_a.position)
        np.testing.assert_allclose(obj_a.position, obj_b.position)
        np.testing.assert_allclose(obj_a.rotation, obj_b.rotation)

    def test_same_seed_identical_across_multiple_boundaries(self):
        boundaries = [FakeObject(BOUNDARY_BBOX),
                      FakeObject((0.0, 1.0, 0.0, 1.0, 0.0, 1.0))]
        sb_a = SpawnBoundary(
            [FakeObject(b._bbox) for b in boundaries],
            rng=np.random.default_rng(3))
        sb_b = SpawnBoundary(
            [FakeObject(b._bbox) for b in boundaries],
            rng=np.random.default_rng(3))
        obj_a, obj_b = FakeObject(OBJ_BBOX), FakeObject(OBJ_BBOX)

        sb_a.sample(obj_a)
        sb_b.sample(obj_b)

        np.testing.assert_allclose(obj_a.position, obj_b.position)
        np.testing.assert_allclose(obj_a.rotation, obj_b.rotation)

    def test_different_seed_produces_different_placements(self):
        sb_a = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.default_rng(7))
        sb_b = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.default_rng(8))
        obj_a, obj_b = FakeObject(OBJ_BBOX), FakeObject(OBJ_BBOX)

        sb_a.sample(obj_a)
        sb_b.sample(obj_b)

        self.assertFalse(np.allclose(obj_a.position, obj_b.position))

    def test_random_state_is_supported(self):
        sb_a = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.RandomState(5))
        sb_b = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.RandomState(5))
        obj_a, obj_b = FakeObject(OBJ_BBOX), FakeObject(OBJ_BBOX)

        sb_a.sample(obj_a)
        sb_b.sample(obj_b)

        np.testing.assert_allclose(obj_a.position, obj_b.position)
        np.testing.assert_allclose(obj_a.rotation, obj_b.rotation)

    def test_no_rng_keeps_global_numpy_behavior(self):
        np.random.seed(11)
        sb_a = SpawnBoundary([FakeObject(BOUNDARY_BBOX)])
        obj_a = FakeObject(OBJ_BBOX)
        sb_a.sample(obj_a)

        np.random.seed(11)
        sb_b = SpawnBoundary([FakeObject(BOUNDARY_BBOX)])
        obj_b = FakeObject(OBJ_BBOX)
        sb_b.sample(obj_b)

        self.assertIsNotNone(obj_a.position)
        np.testing.assert_allclose(obj_a.position, obj_b.position)
        np.testing.assert_allclose(obj_a.rotation, obj_b.rotation)

    def test_positions_stay_within_boundary(self):
        sb = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.default_rng(0))
        for _ in range(20):
            obj = FakeObject(OBJ_BBOX)
            sb.sample(obj)
            self.assertTrue(np.all(obj.position >= np.array([-1., -1., 0.])))
            self.assertTrue(np.all(obj.position <= np.array([1., 1., 1.])))

    def test_boundary_error_still_raised_for_oversized_object(self):
        sb = SpawnBoundary(
            [FakeObject(BOUNDARY_BBOX)], rng=np.random.default_rng(0))
        with self.assertRaises(BoundaryError):
            sb.sample(FakeObject(HUGE_OBJ_BBOX))


if __name__ == '__main__':
    unittest.main()
