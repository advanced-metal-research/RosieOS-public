# Source provenance

**Status at `403063dc`.** P4-01 replaced the original monolithic composition with MachineState and workers; P1-01/P1-02/P1-05/P1-06 add generated native contracts, the Go compiler and Go/C++/TypeScript contracts; P7-01 adds identified packages; P1-10 removed every owned Python file, so the Python descriptors named below survive only as lineage. The body remains the source-lineage record, not a statement that the extracted source still matches its upstream snapshot. P8-11 old-runtime retirement is verified and P8-08 cleanup is merged; current architecture is in [ARCHITECTURE.md](ARCHITECTURE.md).

The initial executor and IPC/CiA402 definitions were extracted from GradientOS
commit `d47bc53b40ebd31f01b53526148c554580f2df04`, from `src/gradient_rt_motion/{main.cpp,ipc_v1.hpp,ds402.hpp,ecrt_detect.hpp}`.
The A6 drive descriptors come from that revision's `ethercat_drive_catalog.py`
and `profiles/drive/a6ec_ds402.py`. That code was first published under MIT by its author and is relicensed with the rest of the repository under Apache-2.0 (`../LICENSE`).
There is no runtime import or source-path dependency on GradientOS.

The EtherCAT host foundation was copied from RosieOS revision
`72d6ab91895a8d8c0b537fc21634e601e4bdff06`. The initially copied bridge, pair owner,
NATS support, telemetry patterns and URDF assets were subsequently removed from
this component; their existing application owners remain in RosieOS.

The current dense-plan decoder and reference fixture were copied from
`offline-programming/v1/internal/densejointtraj` and `motion-server/v2/testdata`
on this testing branch. Their existing format/digest tests are retained locally.
The `robot.v4.dense-joint-trajectory.v1` identifier denotes the input format;
it is not a runtime dependency on `robot/v4`.

IgH 1.6.9 is an external dependency, with source SHA-256
`2bc4d6b69ed980b896ecae9ea6603b327562dfacd3b8c1ed4e9e599a4cc4dfc3`; the
Raspberry Pi host profile selects 1.6.12, SHA-256
`6aa49381da3daf08131913c3fade8fd24b650836567459ef1d35991da1549310`
(see [EtherCAT host profiles](MIGRATION.md#ethercat-host-profiles)).
Its userspace and kernel components retain their respective upstream licenses;
the downloaded source archive contains those notices. The local build helper
does not install the kernel master.

## Drive references

A6-EC manufacturer material copied from GradientOS `docs/resources` and `docs/ethercat` at commit d47bc53b on 2026-09-14 into `docs/drives/a6ec/` (see its README for the file-by-file mapping to rt-core profile values). GradientOS proved every control it exposed on the robot with these drives; the notes are the authority behind the native Home transaction, the startup SDO set and the coordinate evidence design.
