import os
import sys
import types
import unittest
from unittest import mock


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)


class CarlaHealthTests(unittest.TestCase):
    def setUp(self):
        self.original_carla = sys.modules.get("carla")
        self.original_health = sys.modules.pop("carla_health", None)
        self.carla_module = types.ModuleType("carla")
        sys.modules["carla"] = self.carla_module

    def tearDown(self):
        sys.modules.pop("carla_health", None)
        if self.original_health is not None:
            sys.modules["carla_health"] = self.original_health
        if self.original_carla is None:
            sys.modules.pop("carla", None)
        else:
            sys.modules["carla"] = self.original_carla

    def test_probe_requires_a_successful_get_world_call(self):
        from carla_health import probe_carla_world

        client = mock.Mock()
        client.get_world.return_value = object()
        with mock.patch.object(self.carla_module, "Client", return_value=client, create=True):
            healthy, reason = probe_carla_world("127.0.0.1", 2000, timeout_s=1.0)

        self.assertTrue(healthy)
        self.assertEqual(reason, "")
        client.set_timeout.assert_called_once_with(1.0)
        client.get_world.assert_called_once_with()

    def test_probe_rejects_a_listening_but_unresponsive_server(self):
        from carla_health import probe_carla_world

        client = mock.Mock()
        client.get_world.side_effect = RuntimeError("time-out of 20000ms")
        with mock.patch.object(self.carla_module, "Client", return_value=client, create=True):
            healthy, reason = probe_carla_world("127.0.0.1", 2000, timeout_s=1.0)

        self.assertFalse(healthy)
        self.assertIn("time-out", reason)


if __name__ == "__main__":
    unittest.main()
