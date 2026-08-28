#!/usr/bin/env python3

import subprocess
import unittest
from pathlib import Path


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / "scripts"
    / "05d_record_drlvo_teacher_auto.sh"
)


class AutoCaptureShellContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.source = SCRIPT.read_text(encoding="utf-8")

    def test_script_has_valid_bash_syntax(self):
        subprocess.run(["bash", "-n", str(SCRIPT)], check=True)

    def test_same_bag_recovery_does_not_enable_new_bag_restart(self):
        self.assertIn(
            'AUTO_RESTART_FAILED_ATTEMPTS_VALUE="${AUTO_RESTART_FAILED_ATTEMPTS:-0}"',
            self.source,
        )
        self.assertIn("automatic new-bag restart is disabled", self.source)

    def test_safe_relocation_contract_is_forwarded_to_launch_and_scheduler(self):
        required_fragments = (
            "start_online_ppo_training:=false start_auto_capture:=true",
            'robot_reset_service:="${ROBOT_RESET_SERVICE_VALUE}"',
            'robot_entity_name:="${ROBOT_ENTITY_NAME_VALUE}"',
            "relocation_after_failures",
            "relocation_service_timeout_sec",
            "relocation_odom_timeout_sec",
            "relocation_odom_tolerance_m",
            'relocation_target=2.0,2.0,0.0',
            "--setting \"relocation_after_failures=",
        )
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

    def test_recorder_armed_watchdogs_finalize_and_preserve(self):
        required_fragments = (
            'capture_armed_wall="${SECONDS}"',
            "AUTO_FIRST_EPISODE_WALL_SEC_VALUE",
            'grep -q "AUTO_EPISODE_STARTED"',
            "AUTO_MAX_CAPTURE_WALL_SEC_VALUE",
            'available_disk_kib "${bag_dir}"',
            "AUTO_MIN_FREE_DISK_GIB_VALUE",
            "finalizing and preserving the current bag",
        )
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

    def test_finalized_failed_attempts_get_nonfatal_trajectory_reports(self):
        required_fragments = (
            "render_current_attempt_trajectories()",
            'CURRENT_TRAJECTORIES_ATTEMPTED=0',
            'if [[ "${result}" != "0" ]]; then',
            'render_current_attempt_trajectories',
            "capture result is unchanged",
            'status_args=(--status-json "${status_json}")',
        )
        for fragment in required_fragments:
            with self.subTest(fragment=fragment):
                self.assertIn(fragment, self.source)

    def test_success_rendering_stays_before_supervision_export(self):
        render = self.source.index(
            "  render_current_attempt_trajectories\n",
            self.source.index("start_attempt()"),
        )
        export = self.source.index(
            'if [[ "${AUTO_SUPERVISION_EXPORT_VALUE}" == "1" ]]; then',
            render,
        )
        self.assertLess(render, export)

    def test_signal_exit_cleanup_finalizes_then_renders_without_changing_status(self):
        start = self.source.index("cleanup_all() {")
        end = self.source.index("\n}", start)
        cleanup = self.source[start:end]
        self.assertIn('local cleanup_status="$?"', cleanup)
        self.assertLess(
            cleanup.index("cleanup_attempt"),
            cleanup.index("render_current_attempt_trajectories"),
        )
        self.assertIn('return "${cleanup_status}"', cleanup)
        self.assertIn("trap 'exit 130' INT", self.source)
        self.assertIn("trap 'exit 143' TERM", self.source)


if __name__ == "__main__":
    unittest.main()
