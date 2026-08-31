import os
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
RUN_PATH = os.path.join(PROJECT_ROOT, "run_my_control.py")
RUNTIME_PATH = os.path.join(PROJECT_ROOT, "experiment", "runtime.py")


def read(path):
    with open(path, "r", encoding="utf-8") as handle:
        return handle.read()


class AuthoritativeRuntimeIntegrationTest(unittest.TestCase):
    def test_cli_exposes_authoritative_baseline_without_changing_the_default(self):
        source = read(RUN_PATH)
        self.assertIn('"--controller-implementation"', source)
        self.assertIn('choices=("optimized", "authoritative_baseline")', source)
        self.assertIn('default="optimized"', source)
        self.assertIn('"controller_implementation": args.controller_implementation', source)
        self.assertIn('"authoritative_baseline_audit": _controller_implementation_audit(args)', source)
        self.assertIn('from control.authoritative_baseline import authoritative_audit', source)

    def test_runtime_routes_only_the_authoritative_label_to_the_new_factory(self):
        source = read(RUNTIME_PATH)
        self.assertIn('from control.authoritative_baseline import create_authoritative_controller', source)
        self.assertIn('getattr(args, "controller_implementation", "optimized")', source)
        self.assertIn('create_authoritative_controller(controller_name, vehicle, args)', source)
        self.assertIn('create_tracking_controller(controller_name, vehicle, args)', source)


if __name__ == "__main__":
    unittest.main()
