# Rosie specification sheet: pages 1 and 2

Transcribed 2026-09-14 from the published specification sheet,
`AMR_ROSIE_ROBOT_SPEC_SHEET.pdf`.
SHA-256: `c98eca6cbea7b4ef34f32f7a3a3dbe0f1beff685efe634cf9639e4b87e5d322c`.
The PDF has three pages; the two pages relevant here are transcribed below.
The PDF is not copied. These are published specifications, not independent
measurements of the current build.

## Page 1: robot platform

Advanced Metal Research — Building the next industrial workforce.
Robot specification sheet. A six-axis articulated robot platform powered by
AMR's GradientOS, the open-source industrial robotics controller.

Open source; no fees; expandable IO; hollow routing. Open controller: full
industrial robotics controller, open source, with no hidden software fees.
Easy integration: open GradientOS stack reduces lock-in and keeps functionality
accessible. Expandable IO: ready for cell devices, tooling, safety signals and
end-effector integration. Hollow cable paths: 62 mm shoulder and 33 mm wrist
pass-throughs for cleaner cable routing.

| Specification | Published value | Description |
|---|---|---|
| Payload | 15 kg | Nominal handling capacity for Rosie |
| Horizontal reach | 1,509 mm | Wide base-to-tool coverage for benches, fixtures and compact automation cells |
| Vertical reach | 1,638 mm | Above-base vertical reach; total envelope span is a separate sweep value |
| Repeatability | 0.08 mm | Repeatable positioning for fixture work, process validation and automated cell routines |

Footer: Advanced Metal Research;
Rosie robot specification sheet.

## Page 2: performance and open control

Advanced Metal Research. Robot dimensions. Axis specifications.

| Axis | Role | Motion range | Rated speed | Peak speed |
|---|---|---|---|---|
| J1 | Base | +/- 361.0 deg | 180 deg/s | 360 deg/s |
| J2 | Shoulder | +/- 108.9 deg | 180 deg/s | 360 deg/s |
| J3 | Elbow | -240.6 / +87.7 deg | 180 deg/s | 360 deg/s |
| J4 | Wrist roll | +/- 361.0 deg | 1,000 deg/s | 2,000 deg/s |
| J5 | Wrist pitch | +/- 361.0 deg | 1,333 deg/s | 2,667 deg/s |
| J6 | Tool roll | +/- 573.0 deg | 1,800 deg/s | 3,600 deg/s |

Open controller: GradientOS is an open-source full industrial robotics controller,
built to make integration simple without hidden software fees or artificially
limited functionality. Expandable IO supports cell devices, tooling, safety
signals and end-effector integration.

### Controller defaults

| Field | Published value |
|---|---|
| Robot ID | gradient-05 Gradient05Config |
| Physical actuators | 6 EtherCAT axes IDs 0-5 |
| Profile velocity | 0.1 m/s default |
| Profile acceleration | 0.05 m/s^2 default |
| Cartesian jog caps | 0.2 m/s linear; 180 deg/s angular |
| Encoder model | 17-bit motor-side absolute count model |
| Power | 200-240 VAC 3PH 35A |
| Integration | Expandable IO; open-source controller |

Dimension callouts: 1,509 mm horizontal reach; 1,638 mm vertical reach;
62 mm hollow shoulder pass-through; 33 mm hollow wrist pass-through.
Footer: Advanced Metal Research;
Rosie robot specification sheet.

## Interpretation, separate from the PDF transcription

GradientOS `marketing/GRADIENT_05_ROBOT_SPEC_SHEET.md:109,150-152` explicitly
uses rated 3,000 rpm and peak 6,000 rpm with
`joint deg/s = motor rpm * 6 / gear_ratio`; see the copied
[source excerpts](source-excerpts.md). The rated rows correspond to reductions
100, 100, 100, 18, 27/2 and 10. J5's 1,333 deg/s is rounded from
1,333 1/3 deg/s; the rounded row alone does not establish an exact tooth ratio.
GradientOS `config.py:93-94`, commit `2945e3ff` (2026-04-30), provides the exact
27/2 value. This sheet and config are related sources, not independent tests.

The encoder model implies `2^17 = 131072` counts per motor revolution and agrees
with the A6M80/A6M60 motor names ending in M17 in [the CAD list](component-list.json).
This does not identify the encoder installed on a bare bench motor or measure
the resolution of the current build. Published motion ranges and speeds do not
replace the pinned URDF bounds, cell limits or hardware qualification.
