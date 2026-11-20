import json
import unittest
from unittest.mock import patch

import bundle
import prepare
import test_bundle


class PrepareTests(unittest.TestCase):
    def setUp(self):
        test_bundle.BundleTests.setUp(self)
        test_bundle.BundleTests.build(self)
        self.manifest = self.root / "cell-manifest.json"
        self.fixture = self.root / "fixture.json"
        self.reference = self.root / "home.json"
        self.manifest.write_bytes(bundle.encode({"robot_axes": {"calibration": "unchanged"}, "nodes": [
            {"node": "realtime-core", "peer": "fixture-peer", "runtime_paths": ["/old/component", "/old/unrelated"],
             "process_intents": {"component": {"env": ["HOME_KEY=original"]}}}]}))
        self.fixture.write_bytes(bundle.encode({"intents": [{"target_role": "realtime-core", "restart": True,
            "process": {"name": "component", "path": "/old/component", "sha256": "old", "args": ["--config", "/external/cell"], "env": ["REF=original"]}}]}))
        self.reference.write_bytes(b"external home configuration")
        self.selection = {"component": {"target_role": "realtime-core", "path": "/slots/release/component"}}
        self.prepared = self.root / "prepared"

    def run_prepare(self):
        return prepare.prepare(self.spec, self.output, self.manifest, self.fixture, self.selection,
                               {"cell": self.manifest, "home": self.reference}, self.prepared)

    def test_preparation_preserves_config_and_uses_canonical_cli(self):
        original = self.manifest.read_bytes(), self.fixture.read_bytes(), self.reference.read_bytes()
        with patch.object(bundle.subprocess, "run", side_effect=AssertionError("no execution")), patch.object(bundle.subprocess, "check_output", side_effect=AssertionError("no execution")):
            plan = self.run_prepare()
        self.assertEqual(original, (self.manifest.read_bytes(), self.fixture.read_bytes(), self.reference.read_bytes()))
        self.assertEqual((self.prepared / "previous-manifest.json").read_bytes(), original[0])
        expected = json.loads(original[0])
        expected["nodes"][0]["runtime_paths"][0] = "/slots/release/component"
        self.assertEqual(json.loads((self.prepared / "manifest.json").read_bytes()), expected)
        intent = json.loads((self.prepared / "intents.json").read_bytes())["intents"][0]
        self.assertEqual(intent["process"]["env"], ["REF=original"])
        self.assertEqual(intent["process"]["sha256"], "sha256:" + self.spec["components"][0]["sha256"])
        self.assertEqual(plan["dry_run_argv"][:2], ["go", "-C"])
        self.assertEqual(plan["dry_run_argv"][3:7], ["run", ".", "process-intents", "publish"])
        self.assertEqual(plan["dry_run_argv"][-1], "--dry-run")

    def test_corrupt_bundle_or_bad_selection_has_no_output(self):
        self.selection["component"]["path"] = "/old/component"
        with self.assertRaises(ValueError):
            self.run_prepare()
        self.assertFalse(self.prepared.exists())
        self.selection["component"]["path"] = "/slots/release/component"
        (self.output / "artifacts/component").write_bytes(b"corrupted")
        with self.assertRaises(ValueError):
            self.run_prepare()
        self.assertFalse(self.prepared.exists())

    def test_repeat_resume_and_conflicting_recovery(self):
        self.run_prepare()
        before = {p.name: p.read_bytes() for p in self.prepared.iterdir() if p.is_file()}
        self.run_prepare()
        self.assertEqual(before, {p.name: p.read_bytes() for p in self.prepared.iterdir() if p.is_file()})
        (self.prepared / "plan.json").unlink()
        self.run_prepare()
        self.assertEqual((self.prepared / "plan.json").read_bytes(), before["plan.json"])
        (self.prepared / "plan.json").write_bytes(b"partial interrupted file")
        (self.prepared / "intents.json").unlink()
        with self.assertRaises(ValueError):
            self.run_prepare()
        self.assertFalse((self.prepared / "intents.json").exists())
        self.assertEqual((self.prepared / "plan.json").read_bytes(), b"partial interrupted file")

    def test_resume_restores_modes_after_interrupted_chmod(self):
        self.run_prepare()
        artifact = self.prepared / "artifacts/component"
        artifact.chmod(0o644)
        manifest = self.prepared / "manifest.json"
        manifest.chmod(0o644)
        self.run_prepare()
        self.assertEqual(artifact.stat().st_mode & 0o777, 0o755)
        self.assertEqual(manifest.stat().st_mode & 0o777, 0o600)
