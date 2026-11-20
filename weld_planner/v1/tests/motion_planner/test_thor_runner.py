from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MANIFEST = (ROOT / "pixi.toml").read_text(encoding="utf-8")
RUNNER = (ROOT / "tools" / "run_thor_motion.sh").read_text(encoding="utf-8")


def test_thor_environment_is_separate_from_x86_torch() -> None:
    assert 'motion = { features = ["motion", "motion-x86"] }' in MANIFEST
    assert (
        'motion-thor = { features = ["motion", "thor"], '
        'no-default-feature = true }'
    ) in MANIFEST
    assert "[feature.motion-x86.dependencies]" in MANIFEST
    assert 'pytorch-gpu = ">=2.7"' in MANIFEST
    assert "[feature.thor.dependencies]" in MANIFEST
    assert 'python = ">=3.12,<3.13"' in MANIFEST
    assert "[feature.motion.activation.env]" in MANIFEST
    assert 'TORCH_ALLOW_TF32_CUBLAS_OVERRIDE = "0"' in MANIFEST


def test_thor_runner_is_pinned_and_never_pulls() -> None:
    assert "edbbf7492e7a9111974a0e58debbc51371b131e19acf4aa033577cc39d4399a5" in RUNNER
    assert "--pull=never" in RUNNER
    assert "motion-thor --locked" in RUNNER
    assert "/usr/local/lib/python3.12/dist-packages" in RUNNER
    assert 'torch.ones((8,8),device="cuda")' in RUNNER
    assert 'TORCH_ALLOW_TF32_CUBLAS_OVERRIDE:-0' in RUNNER
    assert '-e TORCH_ALLOW_TF32_CUBLAS_OVERRIDE="$TF32_OVERRIDE"' in RUNNER
    assert 'must be 0 or 1' in RUNNER
