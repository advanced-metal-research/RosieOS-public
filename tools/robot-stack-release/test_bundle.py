import copy
import json
from pathlib import Path
import tempfile
import unittest
import tarfile
from unittest.mock import patch

import bundle


class BundleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.inputs = self.root / "inputs"
        self.inputs.mkdir()
        (self.inputs / "artifact").write_bytes(b"fixture")
        (self.inputs / "receipt").write_bytes(b"retained receipt")
        self.spec = {"schema": "rosie.robot-stack.release.v1", "release": "test", "qualification_files": ["receipt"], "components": [
            {"name": "component", "artifact": "artifact", "sha256": bundle.digest(b"fixture"), "source": "a" * 40, "source_files": ["Makefile"], "receipts": ["receipt"]}]}
        self.output = self.root / "bundle"

    def build(self):
        with patch.object(bundle.subprocess, "check_output", return_value=b"source recipe"):
            bundle.package(self.spec, self.inputs, self.root, self.output)

    def test_roundtrip_no_overwrite(self):
        self.build()
        bundle.verify(self.spec, self.output)
        with self.assertRaises(FileExistsError):
            self.build()

    def test_missing_or_wrong_artifact_prevents_output(self):
        for value in [b"wrong", None]:
            p = self.inputs / "artifact"
            if value is None:
                p.unlink()
            else:
                p.write_bytes(value)
            with self.assertRaises(ValueError):
                self.build()
            self.assertFalse(self.output.exists())

    def test_tampering_completeness_and_extra_files(self):
        self.build()
        p = self.output / "artifacts/component"
        p.write_bytes(b"changed")
        with self.assertRaises(ValueError):
            bundle.verify(self.spec, self.output)
        p.write_bytes(b"fixture")
        extra = self.output / "unexpected"
        extra.write_bytes(b"not allowed")
        with self.assertRaises(ValueError):
            bundle.verify(self.spec, self.output)
        extra.unlink()
        (self.output / "provenance/component/source/Makefile").unlink()
        with self.assertRaises(ValueError):
            bundle.verify(self.spec, self.output)

    def test_manifest_and_path_traversal(self):
        self.build()
        manifest = json.loads((self.output / "manifest.json").read_bytes())
        for path in ["../escape", "/absolute", "a/../../escape", "a\\escape", "C:/escape", ".", "a\nb"]:
            bad = copy.deepcopy(manifest)
            bad["files"][path] = "0" * 64
            (self.output / "manifest.json").write_bytes(bundle.encode(bad))
            with self.assertRaises(ValueError):
                bundle.verify(self.spec, self.output)
        manifest["release"]["components"][0]["source"] = "b" * 40
        (self.output / "manifest.json").write_bytes(bundle.encode(manifest))
        with self.assertRaises(ValueError):
            bundle.verify(self.spec, self.output)

    def test_symlinks_rejected(self):
        self.build()
        p = self.output / "artifacts/component"
        p.unlink()
        p.symlink_to(self.inputs / "artifact")
        with self.assertRaises(ValueError):
            bundle.verify(self.spec, self.output)

    def test_missing_pinned_source_has_actionable_error_and_no_output(self):
        with patch.object(bundle.subprocess, "check_output", side_effect=bundle.subprocess.CalledProcessError(128, ["git", "show"])):
            with self.assertRaisesRegex(ValueError, "fetch the manifest-selected component tags"):
                bundle.package(self.spec, self.inputs, self.root, self.output)
        self.assertFalse(self.output.exists())

    def test_generated_table_is_derived_and_archives_are_complete(self):
        self.spec["platform"] = "linux-arm64"
        self.spec["components"][0].update(version="0.1.0", release_tag="component-v0.1.0")
        self.build()
        self.assertEqual((self.output / "COMPONENTS.md").read_bytes(), bundle.component_table(self.spec))
        first, second = self.root / "assets1", self.root / "assets2"
        bundle.archives(self.spec, self.output, first)
        bundle.archives(self.spec, self.output, second)
        for asset in first.iterdir():
            self.assertEqual(asset.read_bytes(), (second / asset.name).read_bytes())
        with tarfile.open(first / "component-v0.1.0-linux-arm64.tar.gz") as archive:
            self.assertEqual(set(archive.getnames()), {"artifacts/component", "manifest.json", "COMPONENTS.md", "SHA256SUMS", "provenance/component/source/Makefile", "provenance/component/receipts/receipt"})
            for line in archive.extractfile("SHA256SUMS").read().decode().splitlines():
                expected, name = line.split("  ", 1)
                self.assertEqual(expected, bundle.digest(archive.extractfile(name).read()))
        with self.assertRaises(FileExistsError):
            bundle.archives(self.spec, self.output, first)
        (self.output / "COMPONENTS.md").write_bytes(b"stale")
        with self.assertRaisesRegex(ValueError, "table mismatch"):
            bundle.verify(self.spec, self.output)

    def test_pinned_runtime_receipt_roundtrip_and_input_refusal(self):
        """Pin complete runtime payloads without changing the primary-binary contract."""
        self.spec["receipt_sha256"] = {"component": {"receipt": bundle.digest(b"retained receipt")}}
        (self.inputs / "receipt").write_bytes(b"changed runtime archive")
        with self.assertRaisesRegex(ValueError, "receipt hash mismatch"):
            self.build()
        self.assertFalse(self.output.exists())
        (self.inputs / "receipt").write_bytes(b"retained receipt")
        self.build()
        bundle.verify(self.spec, self.output)

    def test_pinned_receipt_rejects_rewritten_manifest_and_checksums(self):
        """Trust the specification's receipt pin even if an attacker rewrites inventories."""
        self.spec.update(platform="linux-amd64", receipt_sha256={"component": {"receipt": bundle.digest(b"retained receipt")}})
        self.spec["components"][0]["release_tag"] = "component-v0.1.0"
        self.build()
        name = "provenance/component/receipts/receipt"
        (self.output / name).write_bytes(b"changed runtime archive")
        manifest = json.loads((self.output / "manifest.json").read_bytes())
        manifest["files"][name] = bundle.digest(b"changed runtime archive")
        (self.output / "manifest.json").write_bytes(bundle.encode(manifest))
        files = {p: bundle.read(self.output, p) for p in [*manifest["files"], "manifest.json"]}
        (self.output / "SHA256SUMS").write_bytes("".join(f"{bundle.digest(raw)}  {p}\n" for p, raw in sorted(files.items())).encode())
        with self.assertRaisesRegex(ValueError, "receipt hash mismatch"):
            bundle.verify(self.spec, self.output)
        assets = self.root / "assets"
        with self.assertRaisesRegex(ValueError, "receipt hash mismatch"):
            bundle.archives(self.spec, self.output, assets)
        self.assertFalse(assets.exists())

    def test_receipt_pins_reject_unknown_components_paths_and_bad_hashes(self):
        """Reject misspelled or malformed pins instead of silently leaving payloads unpinned."""
        for value in [None, [], {"unknown": {}}, {"component": []},
                      {"component": {"missing": "a" * 64}},
                      {"component": {"../receipt": "a" * 64}},
                      {"component": {"receipt": "a" * 63}},
                      {"component": {"receipt": 0}}]:
            with self.subTest(value=value):
                self.spec["receipt_sha256"] = value
                with self.assertRaises(ValueError):
                    bundle.validate_spec(self.spec)


class ReleaseSpecificationTests(unittest.TestCase):
    def test_next_selects_native_and_preserves_historical_rollback(self):
        """Keep historical v4 rollback descriptors distinct from native selection."""
        releases = Path(__file__).resolve().parents[2] / "releases/robot-stack"
        specifications = []
        slices = []
        for release in ["v0.1.0", "next"]:
            text = (releases / release / "release.json").read_bytes().decode("utf-8")
            specifications.append({c["name"]: c for c in json.loads(text)["components"]})
            decoder = json.JSONDecoder()
            offset = text.index("[", text.index('"components"')) + 1
            objects = {}
            while True:
                while text[offset].isspace() or text[offset] == ",":
                    offset += 1
                if text[offset] == "]":
                    break
                component, end = decoder.raw_decode(text, offset)
                objects[component["name"]] = text[offset:end]
                offset = end
            self.assertEqual(set(objects), set(specifications[-1]))
            slices.append(objects)
        self.assertEqual(set(specifications[0]), {
            "robot-v4-rtcore", "robot-v4-rtcore-nats-bridge",
            "robot-v4-pair-owner", "robot-v4-cartesiand",
        })
        self.assertEqual(set(specifications[1]), {"rosie-rt-core"})


if __name__ == "__main__":
    unittest.main()
