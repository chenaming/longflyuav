#!/usr/bin/env python3
"""Exercise actual bridge transforms without creating publishers or flight actions."""
import importlib.util
import json
import math
from pathlib import Path
import unittest

import numpy as np
from sensor_msgs.msg import CompressedImage

spec = importlib.util.spec_from_file_location('bridge_under_test', Path(__file__).with_name('vla_direct_bridge.py'))
b = importlib.util.module_from_spec(spec)
spec.loader.exec_module(b)


class Sink:
    def publish(self, _):
        pass


class Coordinates(unittest.TestCase):
    def sample(self, origin=(10.,20.,1.), yaw0=math.pi/2):
        image = CompressedImage()
        return {'origin_p': np.array(origin), 'origin_yaw': yaw0,
                'source_frame': 'map', 'target_frame': 'map',
                'proprio': [0.,0.,0.,0.], 'control_p': np.array(origin),
                'control_yaw': yaw0, 'image': image}

    def bridge(self, paired=False):
        node = b.VLABridge.__new__(b.VLABridge)
        node.preview_pair_alignment = paired
        node.candidate_pub = Sink()
        return node

    def test_cm_to_m_once(self):
        response = b.vec4([[20.,0.,0.,0.]], 'action')
        response[:3] /= 100.
        p,y = self.bridge().transform_target(self.sample(yaw0=0), response)
        np.testing.assert_allclose(p, [10.2,20.,1.], atol=1e-9)

    def test_initial_heading_90_forward_is_world_Y(self):
        p,y = self.bridge().transform_target(self.sample(), np.array([.2,0,0,0]))
        np.testing.assert_allclose(p, [10.,20.2,1.], atol=1e-9)
        self.assertAlmostEqual(y, math.pi/2)

    def test_initial_heading_180(self):
        p,y = self.bridge().transform_target(self.sample(yaw0=math.pi), np.array([.2,0,0,0]))
        np.testing.assert_allclose(p, [9.8,20.,1.], atol=1e-9)

    def test_current_pose_not_added_twice(self):
        sample = self.sample(yaw0=0)
        sample['proprio'] = [100.,0.,0.,0.]
        sample['control_p'] = np.array([11.,20.,1.])
        p,y = self.bridge().transform_target(sample, np.array([1.2,0,0,0]))
        np.testing.assert_allclose(p, [11.2,20.,1.], atol=1e-9)

    def test_pair_delta_rotated_to_mavros(self):
        sample = self.sample(origin=(2.,3.,0.), yaw0=0)
        sample['proprio'] = [100.,0.,0.,0.]
        sample['control_p'] = np.array([10.,20.,1.])
        sample['control_yaw'] = math.pi/2
        p,y = self.bridge(paired=True).transform_target(sample, np.array([1.2,0.,0.,0.]))
        np.testing.assert_allclose(p, [10.,20.2,1.], atol=1e-9)

    def test_vertical_sign_preserved(self):
        p,y = self.bridge().transform_target(self.sample(), np.array([0,0,.1,0]))
        self.assertAlmostEqual(p[2], 1.1)

    def test_yaw_wrap(self):
        p,y = self.bridge().transform_target(self.sample(yaw0=math.radians(179)), np.array([0,0,0,math.radians(3)]))
        self.assertAlmostEqual(math.degrees(y), -178.)

    def test_reject_nonfinite(self):
        with self.assertRaises(ValueError):
            b.vec4([[float('nan'),0,0,0]], 'action')

    def test_reject_wrong_shape(self):
        with self.assertRaises(ValueError):
            b.vec4([1,2,3], 'action')


if __name__ == '__main__':
    result = unittest.TextTestRunner(verbosity=2).run(unittest.defaultTestLoader.loadTestsFromTestCase(Coordinates))
    report = {'tests': result.testsRun, 'failures': len(result.failures),
              'errors': len(result.errors), 'passed': result.wasSuccessful(),
              'physical_axes_calibrated': False,
              'meaning': 'software coordinate transforms only; no vehicle motion'}
    Path('/home/nvidia/.local/state/vla-commissioning/coordinate_tests.json').write_text(json.dumps(report, indent=2))
    raise SystemExit(0 if result.wasSuccessful() else 1)
