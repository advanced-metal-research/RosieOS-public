# GradientOS and deployed source excerpts

Extracted 2026-09-14. Paths below are read-only originals. Hashes identify the complete source bytes; numbered excerpts preserve original lines. Statements describe the cited configuration or record, not a physical inspection.

## GradientOS src/gradient_os/arm_controller/robots/gradient05/config.py
Source: `C:/Users/operator/Desktop/projects/GradientOS/src/gradient_os/arm_controller/robots/gradient05/config.py`
SHA-256: `fdcece6eb54fb22789f9772d8b895d232188e88b3d41a3f9b2b06ef8b6d8a7e5`

```text
77:         """
78:         Absolute encoder resolution per motor revolution.
79:
80:         Current Gradient-05 EtherCAT commissioning assumes 17-bit motor-side
81:         encoder counts from the drive position objects.
82:         """
83:         return [131072] * 6
84:
85:     @property
86:     def actuator_gear_ratios(self) -> list[float]:
87:         """
88:         Motor-revolutions to joint-revolution reduction ratios.
89:
90:         These values come from the current Gradient-05 EtherCAT bring-up notes
91:         and are used by the RTCore backend to convert raw counts into radians.
92:         """
93:         # J5 uses a 10:1 stage followed by 27T:20T, for an exact 27:2 ratio.
94:         return [100.0, 100.0, 100.0, 18.0, 27.0 / 2.0, 10.0]
```

## GradientOS .cursor/memory/AGENT_SCRATCHPAD.md
Source: `C:/Users/operator/Desktop/projects/GradientOS/.cursor/memory/AGENT_SCRATCHPAD.md`
SHA-256: `e809b5f6a8d10591786e03117a693ec29b95d7298a789c01bcfb37372c27ef8c`

```text
67: ### 2026-04-30 - Gradient-05 spec speed formula confirmed; J5 is 27:2
68: - [self] For Gradient-05 axis speed specs, use the documented formula `joint_deg_s = motor_rpm * 6 / gear_ratio`: 3,000 RPM rated and 6,000 RPM peak. This matches J1-J4/J6 in `marketing/GRADIENT_05_ROBOT_SPEC_SHEET.md`.
69: - [user] Correction: Gradient-05 J5 is going to use `10 * 27 / 20 = 27 / 2 = 13.5`, superseding the immediate `100/11` comparison and the older spec-sheet `18:1` row.
70: - [self] With J5 `27:2`, the derived row is rated `1,333 deg/s`, peak `2,667 deg/s`, and `131072 * 13.5 = 1,769,472` encoder counts per joint rev.
```

```text
491: ### 2026-04-12 - A6-EC rotation mode must not boot with default 1:1 gear ratio
493: - [tool] The robot config already carries the correct per-axis reductions `[100, 100, 100, 18, 31.25, 10]`, which map to startup ratio SDOs `C10.18/C10.19 = [100/1, 100/1, 100/1, 18/1, 125/4, 10/1]`.
```

```text
2476: ### 2026-04-25 - Realtime one-shot joint jog implementation landed offline
2484: - [user] Follow-up correction: the J5 gear ratio carry-over from another session must be exactly `10.0`, not `100/11`. Updated `gradient05/config.py` and expectations accordingly.
```

```text
2557: ### 2026-04-26 - J5 ratio corrected to exact 100:11; always source start.sh
2558: - [user] Correction supersedes the 2026-04-25 note that set J5 to `10.0` and the brief 2026-04-26 correction that set J5 to `11.0`: Gradient-05 J5 is `100/11`. Active robot config should be `[100.0, 100.0, 100.0, 18.0, 100.0 / 11.0, 10.0]`; derived rounded J5 wrap period is `round(131072 * 100 / 11) = 1_191_564` counts, and A6-EC startup ratio registers should resolve to `C10.18=100`, `C10.19=11`.
```

```text
2783: ### 2026-04-30 - Rosie sheet follow-up specs and layout corrections
2787: - [user] J5 gear ratio is `18:1`; update derived spec-sheet locations to match. If speed values are shown from rated/peak motor RPM math, J5 becomes `1,000 deg/s` rated and `2,000 deg/s` peak.
```


## GradientOS marketing/GRADIENT_05_ROBOT_SPEC_SHEET.md
Source: `C:/Users/operator/Desktop/projects/GradientOS/marketing/GRADIENT_05_ROBOT_SPEC_SHEET.md`
SHA-256: `00ceb41a073257f252f0b9613ca831f7aaa4caccbb8aba9db609785071adf2a8`

```text
99: | Axis | Joint role    | Motion range       | Rated joint speed | Peak joint speed | Encoder counts / joint rev | Sign |
100: | ---- | ------------- | ------------------ | ----------------- | ---------------- | -------------------------- | ---- |
101: | J1   | Base rotation | +/- 361.0 deg      | 180 deg/s         | 360 deg/s        | 13,107,200                 | -    |
102: | J2   | Shoulder      | +/- 108.9 deg      | 180 deg/s         | 360 deg/s        | 13,107,200                 | +    |
103: | J3   | Elbow         | -240.6 / +87.7 deg | 180 deg/s         | 360 deg/s        | 13,107,200                 | -    |
104: | J4   | Wrist roll    | +/- 361.0 deg      | 1,000 deg/s       | 2,000 deg/s      | 2,359,296                  | -    |
105: | J5   | Wrist pitch   | +/- 361.0 deg      | 1,333 deg/s       | 2,667 deg/s      | 1,769,472                  | -    |
106: | J6   | Tool roll     | +/- 573.0 deg      | 1,800 deg/s       | 3,600 deg/s      | 1,310,720                  | -    |
107:
108:
109: Rated and peak joint speeds are derived from the motor shaft speed through each joint gear ratio: `joint deg/s = motor RPM * 6 / gear_ratio`. Rated speed uses 3,000 RPM; peak speed uses 6,000 RPM. Controller jog, profiled-motion, thermal, and safety policy limits may command lower speeds than these derived mechanical maxima.
```

```text
143: | Robot ID                             | `gradient-05`                                                                |
144: | Robot config class                   | `Gradient05Config`                                                           |
145: | Physical actuators                   | 6 EtherCAT axes, IDs 0-5                                                     |
146: | Default profile velocity             | 0.1 m/s                                                                      |
147: | Default profile acceleration         | 0.05 m/s^2                                                                   |
148: | Max Cartesian jog linear cap         | 0.2 m/s                                                                      |
149: | Max Cartesian jog angular cap        | 180 deg/s                                                                    |
150: | Motor rated speed                    | 3,000 RPM                                                                    |
151: | Motor peak speed                     | 6,000 RPM                                                                    |
152: | Drive-native ratios                  | J1 100:1, J2 100:1, J3 100:1, J4 18:1, J5 27:2, J6 10:1                      |
153: | Power                                | 200-240 VAC 3PH, 35A                                                         |
```

## Pair record
Source: `C:/Users/operator/Desktop/projects/RosieOS/robot/v4/realtime-core/natsbridge/testdata/home-profile-migration/new-record.json:1`
SHA-256: `b62bc36bcee864ef4cb24dd2b1920d788ad25ae65d3007df04d9b9bf9c37cddb`
The single source line contains these fields (other fields omitted; host name redacted):
```text
line 1: {"revision": 6, "committed_at": "2026-09-13T08:39:56.807469Z"}
line 1 snapshot: {"jetson_id": "<host>", "robot_model_id": "rosie_1420_v1", "mapped_mask": 63, "gear_ratios": [100, 100, 100, 18, 100, 10, 1, 1, 1]}
```

## readback.txt
Source: `C:/Users/operator/Desktop/projects/RosieOS/rt-core/docs/evidence/<bench evidence, 2026-09-13>/readback.txt`
SHA-256: `66ff02b701afb52029784bbb4ac8d4741b2cec8edae24973ea7561b11a9c01b4`
```text
1: === done lines 686
2: === readback 2026-09-13T08:59:23+00:00 master IgH EtherCAT master 1.6.9 unknown
55: gear_numerator 0x2010:0x19 uint16 = 0x0064 100
56: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
130: gear_numerator 0x2010:0x19 uint16 = 0x0064 100
131: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
205: gear_numerator 0x2010:0x19 uint16 = 0x0064 100
206: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
280: gear_numerator 0x2010:0x19 uint16 = 0x0012 18
281: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
355: gear_numerator 0x2010:0x19 uint16 = 0x0064 100
356: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
430: gear_numerator 0x2010:0x19 uint16 = 0x000a 10
431: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
505: gear_numerator 0x2010:0x19 uint16 = 0x000a 10
506: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
580: gear_numerator 0x2010:0x19 uint16 = 0x000a 10
581: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
655: gear_numerator 0x2010:0x19 uint16 = 0x000a 10
656: gear_denominator 0x2010:0x1a uint16 = 0x0001 1
```

## bridge.go
Source: `C:/Users/operator/Desktop/projects/RosieOS/robot/v4/realtime-core/natsbridge/bridge.go`
SHA-256: `e83528c7d8c35d3fc8307c56e66a044809886209f3e35a61763c0a427a3aa93b`
```text
77: 	rtcoreCountsPerRev                    = 131072.0
```

## bridge_commands.go
Source: `C:/Users/operator/Desktop/projects/RosieOS/robot/v4/realtime-core/natsbridge/bridge_commands.go` (host names redacted)
SHA-256: `ff759fe6925ce27d258db1099ba697b8a1605628fbbcd4b402324127bf603e7a`
```text
1675: 	qualifiedPair := (config.jetsonID == "<host>" && config.robotModelID == "rosie_1420_v1" && config.mappedAxisMask == 63) ||
1676: 		(config.jetsonID == "<bench host>" && config.robotModelID == "rosie_1400_v3" && config.mappedAxisMask == 511)
```

## revpi-deploy.txt
Source: `C:/Users/operator/Desktop/projects/RosieOS/rt-core/docs/evidence/<bench evidence, 2026-09-13>/revpi-deploy.txt`
SHA-256: `28a49ecdf8976d3789cdfa84da6648601993c744ffb1ce53d9c0a09eaf81758b`
```text
26: Environment=ROBOT_V4_ROBOT_AXES=0:J1:J1:arm:-1:1;1:J2:J2:arm:1:1;2:J3:J3:arm:-1:1;3:J4:J4:arm:-1:1;4:J5:J5:arm:-1:1;5:J6:J6:arm:-1:1;6:J7:J7:external:1:0;7:J8:J8:external:1:0;8:J9:J9:external:1:0
```
