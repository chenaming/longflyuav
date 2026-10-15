#!/usr/bin/env python3
"""CPU/offline validation: real loopback HTTP, mocked model and ROS messages."""
import argparse
import importlib.util
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "server"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--report", type=Path)
    args = parser.parse_args()
    suite = unittest.TestSuite()
    for index, path in enumerate([
        ROOT / "agile_ws/src/agile_openvla_bridge/tests/test_geometry.py",
        ROOT / "server/test_server.py",
        ROOT / "tests/test_end_to_end.py",
        ROOT / "tests/test_node_offline.py",
        ROOT / "tests/test_step_cycle.py",
    ]):
        spec = importlib.util.spec_from_file_location("integration_test_%d" % index, path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        suite.addTests(unittest.defaultTestLoader.loadTestsFromModule(module))
    result = unittest.TextTestRunner(verbosity=2).run(suite)
    if args.report:
        args.report.write_text(json.dumps(dict(
            tests_run=result.testsRun, passed=result.wasSuccessful(),
            failures=[str(t) for t, _ in result.failures],
            errors=[str(t) for t, _ in result.errors],
            validation="CPU geometry, bridge guards with ROS stand-ins, native API with fake agent, real loopback HTTP with deterministic inference",
            real_gpu_inference_tested=False, real_ros_graph_tested=False,
            drone_motion_tested=False, remote_installed=False,
        ), indent=2) + "\n")
    return 0 if result.wasSuccessful() else 1


if __name__ == "__main__":
    sys.exit(main())
