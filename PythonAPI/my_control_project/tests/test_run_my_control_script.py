import os
import unittest


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPT_PATH = os.path.join(PROJECT_ROOT, "scripts", "run_my_control.ps1")


class RunMyControlScriptTest(unittest.TestCase):
    def test_health_check_uses_the_carla_conda_environment(self):
        with open(SCRIPT_PATH, "r", encoding="utf-8-sig") as handle:
            text = handle.read()

        self.assertIn("[switch]$HealthCheck", text)
        self.assertIn("carla_health.py", text)
        self.assertIn("$HealthPort", text)


if __name__ == "__main__":
    unittest.main()
