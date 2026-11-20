# Locked host build environment

`flake.lock` pins the host-side compiler and tools used to run deterministic
RosieOS checks. It is derived from the demonstrated `c08f8ae6` deployment
runbook: C++17 Motion Server builds through its Makefile; Go owns configuration
generators and migration tools; shell/JQ inspect receipts.

```sh
nix flake check
nix build .#motion-server-host
nix develop
```

This flake does **not** make its output a deployable Jetson or RevPi artifact.
The qualified deployment used a manifest-selected Ubuntu 22.04 / glibc 2.35
target environment, retained source and build-recipe hashes, and checked the
target loader before promotion. Nix's store identity is an additional build
input identity; it does not replace the target ABI/sysroot, artifact hashes,
deployment manifest, configuration identities, installation readback or
runtime qualification.

Go 1.26.x is the compatibility line; the locked Nix snapshot supplies Go 1.26.7,
while the demonstrated target recipe used RosieOS's declared Go 1.26.2.
Compatibility does not require rejecting another 1.26.x patch. Reproducing a
release does require its exact patch/toolchain identity pinned in the release
inputs and recorded with its source, recipe and artifact hashes. These are
separate claims: host checks with 1.26.7 do not recreate the deployed 1.26.2
producer or establish target-loader compatibility.

Package-manager locks remain authoritative inside their ecosystems: Go
`go.mod`/`go.sum`, Rust `Cargo.lock`, Node `package-lock.json`, and Pixi locks.
The Nix lock pins the surrounding host tool environment.

The first slice intentionally covers only commands exercised by the qualified
runbook. Add another package or target only with a demonstrated build and its
own credible execution or loader judge.

## Independent verification

On 2026-09-13, source `36231ad94683b09ae6a6510a4ac80683de5ac755` passed
`nix flake check path:/src --no-write-lock-file --print-build-logs` on
`aarch64-linux` in the existing official `nixos/nix:2.30.2` image, digest
`sha256:7894650fb65234b35c80010e6ca44149b70a4a216118a6b7e5c6f6ae377c8d21`.
All three checks built: Motion Server, its three `*_runtime_test.cpp` tests,
and Home-profile migration with the focused Go tests. The same source passed
`--all-systems --no-build`; this is evaluation only for x86_64 Linux and ARM
Darwin, not build/runtime evidence for those systems.

The raw log (kept in the private repository) includes commands' output,
derivation paths and output NAR hashes. Its SHA-256 is
`8ec176da78226f4eeb86487804013b09bbc5ea8e8fd320802b2d7c75f9dc990a`.
The pinned build inputs were unchanged during verification: `flake.nix`
SHA-256 `ffab6f7a03a1c25eda21a8897063fb1eb40c137c7471192699472406a09340fd`,
`flake.lock` SHA-256 `0ae8d7124e7a06626bd764744beac322a1f032eb6f4a0d223b18781ab0f16afc`.
The outputs reference GCC 15.3/glibc 2.42 and are host checks only.

Verdict: passed for the declared ARM Linux host checks and all-system
evaluation. Target deployment and other-platform builds remain not_evaluated.
