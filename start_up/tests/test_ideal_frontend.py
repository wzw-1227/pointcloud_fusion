#!/usr/bin/env python3
"""Check registration direction, fixed reference, range filtering and timing."""
import unittest
import numpy as np
from stage1_tools.geometry import inverse_pose
from stage1_tools.ideal_frontend import TruthBuffer, initial_reference, register_scan


def pose(yaw, translation):
    c, s = np.cos(yaw), np.sin(yaw)
    result = np.eye(4)
    result[:3, :3] = [[c, -s, 0], [s, c, 0], [0, 0, 1]]
    result[:3, 3] = translation
    return result


class IdealFrontendTests(unittest.TestCase):
    def test_timing_requires_brackets_without_extrapolation(self):
        buffer = TruthBuffer()
        buffer.add(1., pose(0, [1, 2, 3])); buffer.add(1.04, pose(np.pi/2, [3, 2, 3]))
        np.testing.assert_allclose(buffer.at(1.02), pose(np.pi/4, [2, 2, 3]), atol=1e-12)
        for stamp in (.99, 1.05):
            with self.assertRaises(ValueError): buffer.at(stamp)

    def test_stale_truth_cannot_be_used(self):
        buffer = TruthBuffer()
        buffer.add(1., np.eye(4)); buffer.add(1.1, np.eye(4))
        with self.assertRaises(ValueError): buffer.at(1.05)
        self.assertFalse(buffer.add(.9, np.eye(4)))

    def test_moving_scan_keeps_fixed_world_surface(self):
        first = pose(.4, [10, -5, 2]); reference = initial_reference(first)
        world = np.array([[12., -3, 3], [11, -4, 1], [13, -4, 2]])
        expected = world @ reference[:3, :3] - reference[:3, :3].T @ reference[:3, 3]
        for current in (first, pose(1.2, [11, -4, 2.5])):
            local = world @ current[:3, :3] - current[:3, :3].T @ current[:3, 3]
            registered, transform, _ = register_scan(local, current, reference, .1, 30)
            np.testing.assert_allclose(registered, expected, atol=1e-12)
            np.testing.assert_allclose(transform, inverse_pose(reference) @ current, atol=1e-12)

    def test_reference_preserves_gravity_and_filter_removes_self_and_invalid(self):
        matrix = pose(.8, [2, 3, 4]); tilt = .2
        roll = np.eye(4); roll[1:3, 1:3] = [[np.cos(tilt), -np.sin(tilt)], [np.sin(tilt), np.cos(tilt)]]
        reference = initial_reference(matrix @ roll)
        np.testing.assert_allclose(reference[:3, 2], [0, 0, 1])
        points = [[.1, 0, 0], [2, 0, 0], [31, 0, 0], [float('nan'), 0, 0]]
        registered, _, removed = register_scan(points, reference, reference, .5, 30)
        np.testing.assert_allclose(registered, [[2, 0, 0]], atol=1e-12); self.assertEqual(removed, 3)


if __name__ == '__main__': unittest.main()
