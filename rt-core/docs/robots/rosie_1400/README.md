# Rosie arm mechanical sources

Extracted 2026-09-14 for robot definitions `rosie_1400_v3` and `rosie_1420_v1`.
The copied sources establish the historical Rosie 1400 CAD component mapping.
The owner decision dated 2026-09-15 below assigns the deployed 100:1 J5 to
the 1420; the 1400 retains its documented 27:2 belt-stage configuration.

| Joint | CAD assembly and motor | Component and retained ratio | Source and qualification |
|---|---|---|---|
| J1 | L1, A6M80-1000H2B1-M17 | HSS-40-100-I-D19, 100/1 | [CAD L1](component-list.json), [sheet J1](spec-sheet-axis-table.md): 180 deg/s at 3,000 rpm. Mechanical part/reduction mapping confirmed in the 1400 sources |
| J2 | L1, second A6M80-1000H2B1-M17 | Second HSS-40-100-I-D19, 100/1 | [CAD L1](component-list.json), [sheet J2](spec-sheet-axis-table.md): 180 deg/s. Two motors and two gearbox occurrences serve J1/J2; CAD list alone does not distinguish their serial numbers |
| J3 | L3, A6M80-1000H2B1-M17 | HSS-40-100-I-D19, 100/1 | [CAD L3](component-list.json), [sheet J3](spec-sheet-axis-table.md): 180 deg/s. Mechanical part/reduction mapping confirmed in the 1400 sources |
| J4 | L3, A6M60-400H2B1-M17 | HRP130_D14; purchased HRP130KK-G18, 18/1 | [CAD L3](component-list.json), purchased-part statement below, [sheet J4](spec-sheet-axis-table.md): 1,000 deg/s. L3 also contains 齿轮Z10_M0.65X10 |
| J5 | L4, A6M60-400H2B1-M17 | Belt tensioner, TG60-G20 and 60ring gear2; 27/2 | [CAD L4](component-list.json), [history](J5-history.md), [sheet J5](spec-sheet-axis-table.md): 1,333 deg/s. GradientOS gradient05 commit 2945e3ff uses a 10:1 stage then 27T:20T. Physical pulley teeth on the current 1400 build remain unconfirmed by the owner; the deployed 100:1 belongs to the 1420 |
| J6 | L5, A6M60-400H2B1-M17 | HRP85_D14, retain 10/1 | [CAD L5](component-list.json), [pair record](source-excerpts.md), [sheet J6](spec-sheet-axis-table.md): 1,800 deg/s. Part identified; HRP85 variant's part-number ratio remains unconfirmed |

## Extraction and purchased-part statement

[component-list.json](component-list.json) records the original path, SHA-256,
extraction date, source generation date, JSON pointers and source line numbers.
It groups all `rosie_1400:1` leaf occurrences by their CAD assembly paths,
including leaves assigned to static residual render groups. The rendering
group `link_5`, for example, contains leaves from both L4 and L5; render-group
membership must not be used to assign a gearbox to a joint.
Names with GBK bytes represented as Latin-1 are decoded per path segment;
already-decoded Unicode names are retained. Parent component occurrences are
deduplicated without collapsing the two L1 motors or gearboxes.

The robotdef3 task received 2026-09-14 identifies the purchased **HRP130KK-G18
(18:1)** as the HRP130 platform. This owner-supplied purchase statement, the
CAD HRP130_D14 component and sheet speed support the J4 mapping. No invoice,
serial-number inspection or vendor drawing was supplied with the statement.
The CAD part is HSS-40-100-I-D19 for J1..J3; the previously cited purchased
HHT-40-100 candidate is not the CAD part name.

## Rosie 1420 transfer and encoders

The supplied 1420 export (`urdf/v1/rosie_1420_v1/` in the predecessor
monorepo) has geometry and articulation exports but no `component_analysis.json`.
Using the 1400 mechanism for 1420 J1..J4 and J6 is an **unverified
identical-mechanism assumption**, including the motor variants. Each 1420 arm
ratio except J5 retains that qualification. The 1420's J5 mechanism differs
from the 1400's belt stage; its 100/1 ratio is owner-confirmed, while its motor
variant remains unverified. The 1420 pair record corroborates its configured
ratios. The 1400 positioner J7..J9 reductions remain
provisional 10/1 from RevPi readback; the unmapped 1420 pair slots contain 1/1.
Arm CAD gearing is not evidence for the positioner gearing.

The [sheet controller defaults](spec-sheet-axis-table.md) say
“17-bit motor-side absolute count model”: `2^17 = 131072` counts per motor
revolution. The arm CAD motors are A6M80-1000H2B1-M17 (J1..J3) and
A6M60-400H2B1-M17 (J4..J6); their M17 suffix supports that model. Independent
counts-per-physical-revolution verification remains open. Bare bench motors
and positioner motor variants are not identified by this arm mapping.

## Retained deployment and bench conventions

[source-excerpts.md](source-excerpts.md) copies the v4 pair ratios, dated RevPi
drive ratios, encoder constant, qualified model masks and deployed signs.
These preserve the deployment record; the model-specific J5 decision below
supersedes transferring its 100:1 value to the 1400. Signs still require comparison with the mapped URDF.
The 1400 has nine mapped axes and the 1420 six; logical cell order is J1..J9
while the 1400 URDF movable order is J1,J2,J3,J4,J5,J6,J9,J7,J8.

The bench owner direction recorded 2026-09-14 is preserved from
`robot_description/robots/bench_nine_motors_1to1/rtcore_definition.json` (then `config/robots/bench_nine_motors_1to1.json`) revision 2 at rt-core commit
`d0097c43`: bare shafts have exact 1/1 gearing; verify that no mechanism has
been attached. Motors left to right are axes 0..8 (J1..J9). On axis 0,
sign -1 makes a positive logical move turn the shaft arrow clockwise on camera
cam1. Signs on axes 1..8 remain configured and unverified. Their retained
sign vector is `[-1,1,-1,-1,-1,-1,1,1,1]`. The v4 software constant 131072 is
copied in source-excerpts.md; neither that constant nor a Home offset readback
measures the bare motors' resolution.

Since 2026-09-20 the geometry rt-core bounds the axes by is the description's own
`robot_description/robots/<model_id>/robot.urdf`, served with the rest of the
description; the archived factory URDFs below are history
(the `robot/v4/realtime-core/natsbridge/*_factory.urdf` originals were retired by P8-11):
`steamdeck/real/v4/internal/legacyarchive/rosie_1400_v3_factory.urdf` (deleted
2026-11-18 with the unused v4 migration tool; `git show 07c22f2c1:<path>`), SHA-256
`86408cd4d7856b12976a465bff410b25f32d6a676961249c223dffd6a580e8be`,
and `urdf/v1/rosie_1420_v1/robot.urdf`, SHA-256
`b85fd4fe9a8b36f7b408934fad988c233b0d287f44a013bacae8d348078db0d8`.
The 1400 factory file's lines 2-3 identify Autodesk Fusion active document
`full_cell_iges` and regeneration by
`urdf/v1/rosie_1400_v3/tools/build_bundle_from_fusion_export.mjs`.
The retained 1420 definition provenance identifies the `rosie_1420` active
CAD document and part exports dated 2026-07-28; the factory bytes match the
bundle's `robot.urdf`. These establish geometry, joint order and limits,
not the gear ratios.

## Owner decision, 2026-09-15

Asked which robot the deployed pair record (`rosie_1420_v1`,
`gear_ratios[4] = 100`), drive readback and the 2026-08-03 team-chat report of pair revision 3
change from 99:1 to 100:1 describe, the owner answered: "yeah this is rosie
1420; if there's that then we go 1420." The 1420 v1 J5 is therefore **100/1**,
confirmed by the owner with the pair record and readback as corroboration.

The 1400 v3 J5 keeps **27/2** from GradientOS gradient05 commit `2945e3ff`
(10:1 stage then 27T:20T), the task-cited DEVLOG 2026-05-01 and the sheet's
1,333 deg/s rated-speed row, consistent with the CAD L4 belt-stage mechanism.
Physical pulley teeth on the current 1400 build remain unverified until the
owner or a mechanical document confirms them. See [J5 history](J5-history.md)
for the historical sources and DEVLOG availability qualification. J6's HRP85
variant ratio remains unconfirmed.

Both arm definitions are revision 3; the bench definition remains revision 3.
The 1400 cell pin changes its compiled J5 startup writes to numerator 27,
denominator 2 and changes native machine and publication identities. The 1420
keeps its native 100/1 J5 writes. The separate bare-motor bench cell and its
compiled digest remain unchanged.
