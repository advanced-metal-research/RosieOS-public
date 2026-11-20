# Third-party notices

RosieOS is licensed under the Apache License, Version 2.0 ([LICENSE](LICENSE)).
That license covers the work of Gradient Industrial Robotics Corporation
(doing business as Advanced Metal Research) only. The material below was
written by others: it keeps its own license or terms and is not relicensed by
this repository. Package-manager dependencies (Go modules, Cargo crates, npm
packages, pixi/conda and PyPI packages) are fetched at build time and carry
their own license files; they are not listed here.

## Vendor documentation and device files

The manufacturers' publicly available documents, kept beside the code that
configures the drives and bus for ease of reference. They remain the property
of their owners and are not covered by the Apache License.

| Material | Location | Owner |
| --- | --- | --- |
| A6-EC series servo drive manual and chapter excerpts | `rt-core/docs/drives/a6ec/*.pdf` | STEPPERONLINE |
| A6 EtherCAT slave information (ESI) file | `rt-core/docs/drives/a6ec/esi/STEPPERONLINE_A6_Servo_V0.02.xml` | STEPPERONLINE |

## Excerpts

| Material | Location | License |
| --- | --- | --- |
| IgH EtherCAT Master systemd start script, embedded as a test fixture | `rt-core/host/minibench/inspection_test.go` | GPL-2.0, Copyright (C) 2006-2021 Florian Pose, Ingenieurgemeinschaft IgH |

The IgH EtherCAT Master itself is not vendored. `rt-core/tools/build-igh-userlib.sh`
downloads a pinned release and builds only its userspace library (LGPL-2.1),
which rt-core links dynamically. Distributions that ship `libethercat.so`
must include its LGPL-2.1 license and source offer.
