import importlib.util
import os
import sys
import tempfile
import unittest
from pathlib import Path
from subprocess import CalledProcessError, CompletedProcess
from unittest import mock


PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if PROJECT_ROOT not in sys.path:
    sys.path.insert(0, PROJECT_ROOT)

MODULE_PATH = os.path.join(PROJECT_ROOT, "experiment", "evaluation_report.py")
SPEC = importlib.util.spec_from_file_location("evaluation_report_under_test", MODULE_PATH)
REPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(REPORT)


class EvaluationReportTest(unittest.TestCase):
    def test_artifact_contains_required_sections_and_native_charts(self):
        artifact = REPORT.build_artifact(REPORT.fixture_bundle(title="<unsafe>"))
        markdown = "\n".join(
            block.get("body", "") for block in artifact["manifest"]["blocks"]
            if block["type"] == "markdown"
        )
        for section in (
            "技术摘要", "硬门槛结果", "纯控制器公平对比", "完整优化系统对比",
            "鲁棒性与最差场景", "实时性", "精度/舒适/效率取舍",
            "最差失败窗口", "适用场景与最终推荐", "方法、局限与引用",
        ):
            self.assertIn(section, markdown)
        self.assertEqual(artifact["surface"], "report")
        self.assertGreaterEqual(len(artifact["manifest"]["charts"]), 6)
        self.assertNotIn("html", {block["type"] for block in artifact["manifest"]["blocks"]})

    def test_semantic_preview_escapes_source_text(self):
        preview = REPORT.render_semantic_preview(REPORT.fixture_bundle(title="<unsafe>"))
        self.assertNotIn("<unsafe>", preview)
        self.assertIn("&lt;unsafe&gt;", preview)

    def test_artifact_json_is_written_for_packaged_html_delivery(self):
        with tempfile.TemporaryDirectory() as temp_dir:
            path = os.path.join(temp_dir, "artifact.json")
            REPORT.write_artifact(REPORT.build_artifact(REPORT.fixture_bundle()), path)
            self.assertTrue(os.path.exists(path))
            self.assertGreater(os.path.getsize(path), 1000)

    def test_failed_laps_count_in_headline_but_not_metric_charts(self):
        bundle = REPORT.fixture_bundle()
        bundle["raw_rows"].append({
            "evaluation_case_id": "failed_case",
            "scenario": "失败场景",
            "layer": "integrated_stress",
            "controller": "pid",
            "status": "failed",
            "hard_gate_passed": False,
        })

        artifact = REPORT.build_artifact(bundle)
        datasets = artifact["snapshot"]["datasets"]

        self.assertEqual(datasets["headline"][0]["planned_laps"], 28)
        self.assertEqual(datasets["headline"][0]["failure_count"], 1)
        self.assertEqual(len(datasets["raw_laps"]), 27)
        self.assertNotIn("失败场景", {row["scenario"] for row in datasets["scenario_matrix"]})

    def test_browser_timeout_falls_back_to_structurally_valid_report(self):
        timeout = CalledProcessError(1, ["node"], stderr='{"code":"browser_timeout"}')
        built = CompletedProcess(["node"], 0, stdout='{"ok":true}\n', stderr="")
        with mock.patch.object(REPORT, "_delivery_script", return_value=Path("deliver_portable_artifact.mjs")), \
                mock.patch.object(REPORT.subprocess, "run", side_effect=[timeout, built]) as run:
            result = REPORT.deliver_artifact("artifact.json", "report.html")

        self.assertEqual(run.call_count, 2)
        self.assertIn("structural_only", result.stdout)


if __name__ == "__main__":
    unittest.main()
