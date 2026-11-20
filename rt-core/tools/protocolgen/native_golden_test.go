package main

import wire "rosieos/rt-core/ipcclient"

// Pinned values from the original C++ layouts; do not regenerate with schema changes.
func nativeGoldenCases() []goldenCase {
	return append(cellIOGoldenCases(), []goldenCase{
		{name: "CmdRestoreReferenceVerifiedDeltaV1", size: 8, expected: wire.CmdRestoreReferenceVerifiedDeltaV1{AxisMask: 5},
			decode: func(p []byte) (any, error) { return wire.DecodeCmdRestoreReferenceVerifiedDeltaV1Frame(p) }, encode: func(v any) []byte {
				p := wire.EncodeCmdRestoreReferenceVerifiedDeltaV1(v.(wire.CmdRestoreReferenceVerifiedDeltaV1))
				return p[:]
			}},
		{name: "HaltRequest", size: 16, expected: wire.HaltRequest{NativeGeneration: 2, ApplicationGeneration: 3},
			decode: func(p []byte) (any, error) { return wire.DecodeHaltRequestFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeHaltRequest(v.(wire.HaltRequest)); return p[:] }},
		{name: "CommandResult", size: 40, decode: func(p []byte) (any, error) { return wire.DecodeCommandResultFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeCommandResult(v.(wire.CommandResult)); return p[:] },
			expected: func() any {
				value := wire.CommandResult{}
				value.Sequence = 650777868590383874
				value.Handle = 1229499251294997258
				value.Generation = 1808220633999610642
				value.Operation = 488381210
				value.Result = 555753246
				value.Reason = 623125282
				value.AxisMask = 690497318
				return value
			}()},
		{name: "TrajectoryHandle", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeTrajectoryHandleFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeTrajectoryHandle(v.(wire.TrajectoryHandle)); return p[:] },
			expected: func() any {
				value := wire.TrajectoryHandle{}
				value.Handle = 650777868590383874
				return value
			}()},
		{name: "LocalJogUpdate", size: 224, decode: func(p []byte) (any, error) { return wire.DecodeLocalJogUpdateFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeLocalJogUpdate(v.(wire.LocalJogUpdate)); return p[:] },
			expected: func() any {
				value := wire.LocalJogUpdate{}
				value.SessionId[0] = 2
				value.SessionId[1] = 3
				value.SessionId[2] = 4
				value.SessionId[3] = 5
				value.SessionId[4] = 6
				value.SessionId[5] = 7
				value.SessionId[6] = 8
				value.SessionId[7] = 9
				value.SessionId[8] = 10
				value.SessionId[9] = 11
				value.SessionId[10] = 12
				value.SessionId[11] = 13
				value.SessionId[12] = 14
				value.SessionId[13] = 15
				value.SessionId[14] = 16
				value.SessionId[15] = 17
				value.SessionId[16] = 18
				value.SessionId[17] = 19
				value.SessionId[18] = 20
				value.SessionId[19] = 21
				value.SessionId[20] = 22
				value.SessionId[21] = 23
				value.SessionId[22] = 24
				value.SessionId[23] = 25
				value.SessionId[24] = 26
				value.SessionId[25] = 27
				value.SessionId[26] = 28
				value.SessionId[27] = 29
				value.SessionId[28] = 30
				value.SessionId[29] = 31
				value.SessionId[30] = 32
				value.SessionId[31] = 33
				value.ClockIncarnation[0] = 34
				value.ClockIncarnation[1] = 35
				value.ClockIncarnation[2] = 36
				value.ClockIncarnation[3] = 37
				value.ClockIncarnation[4] = 38
				value.ClockIncarnation[5] = 39
				value.ClockIncarnation[6] = 40
				value.ClockIncarnation[7] = 41
				value.ClockIncarnation[8] = 42
				value.ClockIncarnation[9] = 43
				value.ClockIncarnation[10] = 44
				value.ClockIncarnation[11] = 45
				value.ClockIncarnation[12] = 46
				value.ClockIncarnation[13] = 47
				value.ClockIncarnation[14] = 48
				value.ClockIncarnation[15] = 49
				value.GrantGeneration = 4123106164818064178
				value.JogGeneration = 4701827547522677562
				value.SourceSequence = 5280548930227290946
				value.OriginHostNs = 5859270312931904330
				value.DeadlineHostNs = 6437991695636517714
				value.AxisMask = 1566333786
				value.Velocity[0] = 97.25
				value.Velocity[1] = 105.25
				value.Velocity[2] = 113.25
				value.Velocity[3] = 121.25
				value.Velocity[4] = 129.25
				value.Velocity[5] = 137.25
				value.Velocity[6] = 145.25
				value.Velocity[7] = 153.25
				value.Velocity[8] = 161.25
				value.Velocity[9] = 169.25
				value.Velocity[10] = 177.25
				value.Velocity[11] = 185.25
				value.Velocity[12] = 193.25
				value.Velocity[13] = 201.25
				value.Velocity[14] = 209.25
				value.Velocity[15] = 217.25
				return value
			}()},
		{name: "JogControl", size: 112, decode: func(p []byte) (any, error) { return wire.DecodeJogControlFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeJogControl(v.(wire.JogControl)); return p[:] },
			expected: func() any {
				value := wire.JogControl{}
				value.Ticket = 650777868590383874
				value.ConnectionId = 1229499251294997258
				value.NativeGeneration = 1808220633999610642
				value.CapabilityId[0] = 26
				value.CapabilityId[1] = 27
				value.CapabilityId[2] = 28
				value.CapabilityId[3] = 29
				value.CapabilityId[4] = 30
				value.CapabilityId[5] = 31
				value.CapabilityId[6] = 32
				value.CapabilityId[7] = 33
				value.CapabilityId[8] = 34
				value.CapabilityId[9] = 35
				value.CapabilityId[10] = 36
				value.CapabilityId[11] = 37
				value.CapabilityId[12] = 38
				value.CapabilityId[13] = 39
				value.CapabilityId[14] = 40
				value.CapabilityId[15] = 41
				value.AppGrantGeneration = 3544384782113450794
				value.ConfigurationEpoch = 4123106164818064178
				value.HomeEpoch = 4701827547522677562
				value.ClockMappingGeneration = 5280548930227290946
				value.JogGeneration = 5859270312931904330
				value.OriginHostNs = 6437991695636517714
				value.DeadlineHostNs = 7016713078341131098
				value.AxisMask = 1701077858
				value.Operation = 1768449894
				return value
			}()},
		{name: "JogUpdate", size: 240, decode: func(p []byte) (any, error) { return wire.DecodeJogUpdateFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeJogUpdate(v.(wire.JogUpdate)); return p[:] },
			expected: func() any {
				value := wire.JogUpdate{}
				value.ConnId = 650777868590383874
				value.NativeGeneration = 1229499251294997258
				value.CapabilityId[0] = 18
				value.CapabilityId[1] = 19
				value.CapabilityId[2] = 20
				value.CapabilityId[3] = 21
				value.CapabilityId[4] = 22
				value.CapabilityId[5] = 23
				value.CapabilityId[6] = 24
				value.CapabilityId[7] = 25
				value.CapabilityId[8] = 26
				value.CapabilityId[9] = 27
				value.CapabilityId[10] = 28
				value.CapabilityId[11] = 29
				value.CapabilityId[12] = 30
				value.CapabilityId[13] = 31
				value.CapabilityId[14] = 32
				value.CapabilityId[15] = 33
				value.AppGrantGeneration = 2965663399408837410
				value.ConfigEpoch = 3544384782113450794
				value.HomeEpoch = 4123106164818064178
				value.JogGeneration = 4701827547522677562
				value.SourceSequence = 5280548930227290946
				value.EarliestOriginHostNs = 5859270312931904330
				value.ImmutableDeadlineHostNs = 6437991695636517714
				value.ClockMappingGeneration = 7016713078341131098
				value.AxisMask = 1701077858
				value.Velocity[0] = 105.25
				value.Velocity[1] = 113.25
				value.Velocity[2] = 121.25
				value.Velocity[3] = 129.25
				value.Velocity[4] = 137.25
				value.Velocity[5] = 145.25
				value.Velocity[6] = 153.25
				value.Velocity[7] = 161.25
				value.Velocity[8] = 169.25
				value.Velocity[9] = 177.25
				value.Velocity[10] = 185.25
				value.Velocity[11] = 193.25
				value.Velocity[12] = 201.25
				value.Velocity[13] = 209.25
				value.Velocity[14] = 217.25
				value.Velocity[15] = 225.25
				return value
			}()},
		{name: "JogObservation", size: 160, decode: func(p []byte) (any, error) { return wire.DecodeJogObservationFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeJogObservation(v.(wire.JogObservation)); return p[:] },
			expected: func() any {
				value := wire.JogObservation{}
				value.Ticket = 650777868590383874
				value.ConnectionId = 1229499251294997258
				value.NativeGeneration = 1808220633999610642
				value.CapabilityId[0] = 26
				value.CapabilityId[1] = 27
				value.CapabilityId[2] = 28
				value.CapabilityId[3] = 29
				value.CapabilityId[4] = 30
				value.CapabilityId[5] = 31
				value.CapabilityId[6] = 32
				value.CapabilityId[7] = 33
				value.CapabilityId[8] = 34
				value.CapabilityId[9] = 35
				value.CapabilityId[10] = 36
				value.CapabilityId[11] = 37
				value.CapabilityId[12] = 38
				value.CapabilityId[13] = 39
				value.CapabilityId[14] = 40
				value.CapabilityId[15] = 41
				value.AppGrantGeneration = 3544384782113450794
				value.ConfigurationEpoch = 4123106164818064178
				value.HomeEpoch = 4701827547522677562
				value.ClockMappingGeneration = 5280548930227290946
				value.JogGeneration = 5859270312931904330
				value.SourceSequence = 6437991695636517714
				value.InputDeadlineHostNs = 7016713078341131098
				value.NowHostNs = 7595434461045744482
				value.ObservedSourceSequence = 8174155843750357866
				value.ObservedOriginHostNs = 8752877226454971250
				value.FirstObservedHostNs = 144536844300483450
				value.ControlReason = 100992003
				value.ControlAccepted = 168364039
				value.UpdateReason = 235736075
				value.StateReason = 303108111
				value.AxisMask = 370480147
				value.Open = 437852183
				value.HasInput = 505224219
				return value
			}()},
		{name: "LocalPathSamples", size: 2272, decode: func(p []byte) (any, error) { return wire.DecodeLocalPathSamplesFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeLocalPathSamples(v.(wire.LocalPathSamples)); return p[:] },
			expected: func() any {
				// Protocol 1.14 layout, pinned when the record was introduced.
				value := wire.LocalPathSamples{GrantGeneration: 4123106164818064178, JogGeneration: 4701827547522677562,
					ChunkSequence: 5280548930227290946, FirstIndex: 5859270312931904330, Spacing: 0.001, MaxSpeed: 0.4,
					Count: 16, Flags: 1, EndReason: 1701077858}
				for i := range value.SessionId {
					value.SessionId[i] = byte(2 + i)
				}
				for i := range value.Caps {
					value.Caps[i] = 0.5 + float64(i)
				}
				for i := range value.Samples {
					value.Samples[i] = float64(i)*0.125 - 16
				}
				return value
			}()},
		{name: "PathSamples", size: 2296, decode: func(p []byte) (any, error) { return wire.DecodePathSamplesFrame(p) },
			encode: func(v any) []byte { p := wire.EncodePathSamples(v.(wire.PathSamples)); return p[:] },
			expected: func() any {
				// Protocol 1.14 layout, pinned when the record was introduced.
				value := wire.PathSamples{ConnId: 650777868590383874, NativeGeneration: 1229499251294997258,
					AppGrantGeneration: 2965663399408837410, ConfigEpoch: 3544384782113450794, HomeEpoch: 4123106164818064178,
					JogGeneration: 4701827547522677562, ClockMappingGeneration: 5280548930227290946,
					ChunkSequence: 5859270312931904330, FirstIndex: 6437991695636517714, Spacing: 0.001, MaxSpeed: 0.4,
					Count: 16, Flags: 1, EndReason: 1835821930}
				for i := range value.CapabilityId {
					value.CapabilityId[i] = byte(18 + i)
				}
				for i := range value.Caps {
					value.Caps[i] = 0.125 + float64(i)
				}
				for i := range value.Samples {
					value.Samples[i] = float64(i)*0.25 - 32
				}
				return value
			}()},
		{name: "PathObservation", size: 184, decode: func(p []byte) (any, error) { return wire.DecodePathObservationFrame(p) },
			encode: func(v any) []byte { p := wire.EncodePathObservation(v.(wire.PathObservation)); return p[:] },
			expected: func() any {
				// Protocol 1.14 layout, pinned when the record was introduced.
				value := wire.PathObservation{ConnId: 650777868590383874, NativeGeneration: 1229499251294997258,
					AppGrantGeneration: 2965663399408837410, ConfigEpoch: 3544384782113450794, HomeEpoch: 4123106164818064178,
					JogGeneration: 4701827547522677562, ClockMappingGeneration: 5280548930227290946,
					ChunkSequenceSeen: 5859270312931904330, ChunkResult: 1499818530, Flags: 31,
					Received: 7016713078341131098, PlayableEndIndex: 7595434461045744482, ConsumedIndex: 8174155843750357866,
					PathS: 0.25, PathSpeed: 0.375, PathAcceleration: -1.5, Underruns: 8752877226454971250,
					SpeedSequence: 144536844300483450, SpeedDeadlineHostNs: 723258227005096834,
					NowHostNs: 1301979609709710218, Governing: 6, StopReason: 4, EndReason: 2105599098}
				for i := range value.CapabilityId {
					value.CapabilityId[i] = byte(18 + i)
				}
				return value
			}()},
		{name: "FastHeader", size: 96, decode: func(p []byte) (any, error) { return wire.DecodeFastHeaderFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeFastHeader(v.(wire.FastHeader)); return p[:] },
			expected: func() any {
				value := wire.FastHeader{}
				value.Magic = 650777868590383874
				value.VersionMajor = 1
				value.VersionMinor = 6
				value.RegionBytes = 1808220633999610642
				value.DaemonIncarnation[0] = 26
				value.DaemonIncarnation[1] = 27
				value.DaemonIncarnation[2] = 28
				value.DaemonIncarnation[3] = 29
				value.DaemonIncarnation[4] = 30
				value.DaemonIncarnation[5] = 31
				value.DaemonIncarnation[6] = 32
				value.DaemonIncarnation[7] = 33
				value.DaemonIncarnation[8] = 34
				value.DaemonIncarnation[9] = 35
				value.DaemonIncarnation[10] = 36
				value.DaemonIncarnation[11] = 37
				value.DaemonIncarnation[12] = 38
				value.DaemonIncarnation[13] = 39
				value.DaemonIncarnation[14] = 40
				value.DaemonIncarnation[15] = 41
				value.ConnectionId = 3544384782113450794
				value.ConfigurationDigestLo[0] = 50
				value.ConfigurationDigestLo[1] = 51
				value.ConfigurationDigestLo[2] = 52
				value.ConfigurationDigestLo[3] = 53
				value.ConfigurationDigestLo[4] = 54
				value.ConfigurationDigestLo[5] = 55
				value.ConfigurationDigestLo[6] = 56
				value.ConfigurationDigestLo[7] = 57
				value.ConfigurationDigestLo[8] = 58
				value.ConfigurationDigestLo[9] = 59
				value.ConfigurationDigestLo[10] = 60
				value.ConfigurationDigestLo[11] = 61
				value.ConfigurationDigestLo[12] = 62
				value.ConfigurationDigestLo[13] = 63
				value.ConfigurationDigestLo[14] = 64
				value.ConfigurationDigestLo[15] = 65
				value.ConfigurationDigestHi[0] = 66
				value.ConfigurationDigestHi[1] = 67
				value.ConfigurationDigestHi[2] = 68
				value.ConfigurationDigestHi[3] = 69
				value.ConfigurationDigestHi[4] = 70
				value.ConfigurationDigestHi[5] = 71
				value.ConfigurationDigestHi[6] = 72
				value.ConfigurationDigestHi[7] = 73
				value.ConfigurationDigestHi[8] = 74
				value.ConfigurationDigestHi[9] = 75
				value.ConfigurationDigestHi[10] = 76
				value.ConfigurationDigestHi[11] = 77
				value.ConfigurationDigestHi[12] = 78
				value.ConfigurationDigestHi[13] = 79
				value.ConfigurationDigestHi[14] = 80
				value.ConfigurationDigestHi[15] = 81
				value.CycleNs = 6437991695636517714
				value.ConfiguredMask = 1566333786
				return value
			}()},
		{name: "GrantRequest", size: 80, decode: func(p []byte) (any, error) { return wire.DecodeGrantRequestFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeGrantRequest(v.(wire.GrantRequest)); return p[:] },
			expected: func() any {
				value := wire.GrantRequest{}
				value.Ticket = 650777868590383874
				value.ConnectionId = 1229499251294997258
				value.NativeGeneration = 1808220633999610642
				value.CapabilityId[0] = 26
				value.CapabilityId[1] = 27
				value.CapabilityId[2] = 28
				value.CapabilityId[3] = 29
				value.CapabilityId[4] = 30
				value.CapabilityId[5] = 31
				value.CapabilityId[6] = 32
				value.CapabilityId[7] = 33
				value.CapabilityId[8] = 34
				value.CapabilityId[9] = 35
				value.CapabilityId[10] = 36
				value.CapabilityId[11] = 37
				value.CapabilityId[12] = 38
				value.CapabilityId[13] = 39
				value.CapabilityId[14] = 40
				value.CapabilityId[15] = 41
				value.AppGrantGeneration = 3544384782113450794
				value.OriginHostNs = 4123106164818064178
				value.DeadlineHostNs = 4701827547522677562
				value.AxisMask = 1162101570
				value.Operation = 1229473606
				return value
			}()},
		{name: "GrantObservation", size: 128, decode: func(p []byte) (any, error) { return wire.DecodeGrantObservationFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeGrantObservation(v.(wire.GrantObservation)); return p[:] },
			expected: func() any {
				value := wire.GrantObservation{}
				value.Ticket = 650777868590383874
				value.ConnectionId = 1229499251294997258
				value.NativeGeneration = 1808220633999610642
				value.AppGrantGeneration = 2386942016704224026
				value.CapabilityId[0] = 34
				value.CapabilityId[1] = 35
				value.CapabilityId[2] = 36
				value.CapabilityId[3] = 37
				value.CapabilityId[4] = 38
				value.CapabilityId[5] = 39
				value.CapabilityId[6] = 40
				value.CapabilityId[7] = 41
				value.CapabilityId[8] = 42
				value.CapabilityId[9] = 43
				value.CapabilityId[10] = 44
				value.CapabilityId[11] = 45
				value.CapabilityId[12] = 46
				value.CapabilityId[13] = 47
				value.CapabilityId[14] = 48
				value.CapabilityId[15] = 49
				value.DeadlineHostNs = 4123106164818064178
				value.NowHostNs = 4701827547522677562
				value.ConfigurationEpoch = 5280548930227290946
				value.HomeEpoch = 5859270312931904330
				value.StopAckId = 6437991695636517714
				value.GrantReason = 1566333786
				value.GrantActive = 1633705822
				value.AxisMask = 1701077858
				return value
			}()},
		{name: "HelloV1", size: 56, decode: func(p []byte) (any, error) { return wire.DecodeHelloV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeHelloV1(v.(wire.HelloV1)); return p[:] },
			expected: func() any {
				value := wire.HelloV1{}
				value.Magic = 84148994
				value.VerMajor = 1
				value.VerMinor = 6
				value.Bytes = 218893066
				value.Role = 286265102
				value.Pid = 1808220633999610642
				return value
			}()},
		{name: "WelcomeV1", size: 96, decode: func(p []byte) (any, error) { return wire.DecodeWelcomeV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeWelcomeV1(v.(wire.WelcomeV1)); return p[:] },
			expected: func() any {
				value := wire.WelcomeV1{}
				value.Magic = 84148994
				value.VerMajor = 1
				value.VerMinor = 6
				value.Bytes = 218893066
				value.NumAxes = 286265102
				value.CycleNs = 2386942016704224026
				value.TopologyHash = 2965663399408837410
				value.CmdRingCapacity = 757869354
				value.CmdMsgBytes = 825241390
				value.StatusRingCapacity = 892613426
				value.StatusMsgBytes = 959985462
				value.BuildIdHash = 4701827547522677562
				return value
			}()},
		{name: "ShmHeaderV1", size: 128, decode: func(p []byte) (any, error) { return wire.DecodeShmHeaderV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeShmHeaderV1(v.(wire.ShmHeaderV1)); return p[:] },
			expected: func() any {
				value := wire.ShmHeaderV1{}
				value.Magic = 84148994
				value.VerMajor = 1
				value.VerMinor = 6
				value.Bytes = 218893066
				value.Kind = 286265102
				value.NumAxes = 353637138
				value.CycleNs = 2386942016704224026
				value.TopologyHash = 2965663399408837410
				value.RingOffset = 757869354
				value.RingCapacity = 825241390
				value.RingMsgBytes = 892613426
				return value
			}()},
		{name: "RingHeaderV1", size: 28, decode: func(p []byte) (any, error) { return wire.DecodeRingHeaderV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeRingHeaderV1(v.(wire.RingHeaderV1)); return p[:] },
			expected: func() any {
				value := wire.RingHeaderV1{}
				value.Magic = 84148994
				value.Capacity = 151521030
				value.MsgBytes = 218893066
				value.WriteIdx = 286265102
				value.ReadIdx = 353637138
				value.Dropped = 421009174
				return value
			}()},
		{name: "MsgHeader", size: 24, decode: func(p []byte) (any, error) { return wire.DecodeMsgHeaderFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeMsgHeader(v.(wire.MsgHeader)); return p[:] },
			expected: func() any {
				value := wire.MsgHeader{}
				value.Type = 770
				value.Bytes = 151521030
				value.Seq = 1229499251294997258
				value.TimeNs = 1808220633999610642
				return value
			}()},
		{name: "AxisStatusV1", size: 56, decode: func(p []byte) (any, error) { return wire.DecodeAxisStatusV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeAxisStatusV1(v.(wire.AxisStatusV1)); return p[:] },
			expected: func() any {
				value := wire.AxisStatusV1{}
				value.PosCounts = -84148994
				value.TorqueRaw = -1798
				value.Statusword = 2312
				value.ErrorCode = 2826
				value.ModeDisplay = 12
				value.Ds402State = 13
				value.ManufacturerErrorCode = 286265102
				value.DiBits = 353637138
				value.AxisFaultFlags = 421009174
				value.BrakeState = 488381210
				value.VelocityActualCountsPerS = -123456
				value.FollowingErrorCounts = 789
				value.VelocityActualValid = 1
				value.FollowingErrorValid = 1
				value.MaxAbsVelocityActualCountsPerS = 2147483648
				value.MaxAbsFollowingErrorCounts = 789
				return value
			}()},
		{name: "StatusSnapshotV1", size: 3400, decode: func(p []byte) (any, error) { return wire.DecodeStatusSnapshotV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeStatusSnapshotV1(v.(wire.StatusSnapshotV1)); return p[:] },
			expected: func() any {
				value := wire.StatusSnapshotV1{}
				value.NumAxes = 84148994
				value.WkcExpected = 151521030
				value.WkcActual = 218893066
				value.MasterState = 286265102
				value.DcOffsetNs = -1808220633999610642
				value.CycleJitterNs = -2386942016704224026
				value.TopologyHash = 2965663399408837410
				value.Axes[0].PosCounts = -757869354
				value.Axes[0].TorqueRaw = -12078
				value.Axes[0].Statusword = 12592
				value.Axes[0].ErrorCode = 13106
				value.Axes[0].ModeDisplay = 52
				value.Axes[0].Ds402State = 53
				value.Axes[0].ManufacturerErrorCode = 959985462
				value.Axes[0].DiBits = 1027357498
				value.Axes[0].AxisFaultFlags = 1094729534
				value.Axes[0].BrakeState = 1162101570
				value.Axes[0].VelocityActualCountsPerS = -123456
				value.Axes[0].FollowingErrorCounts = 789
				value.Axes[0].VelocityActualValid = 1
				value.Axes[0].FollowingErrorValid = 1
				value.Axes[0].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[0].MaxAbsFollowingErrorCounts = 789
				value.Axes[1].PosCounts = -1229473606
				value.Axes[1].TorqueRaw = -19274
				value.Axes[1].Statusword = 19788
				value.Axes[1].ErrorCode = 20302
				value.Axes[1].ModeDisplay = 80
				value.Axes[1].Ds402State = 81
				value.Axes[1].ManufacturerErrorCode = 1431589714
				value.Axes[1].DiBits = 1498961750
				value.Axes[1].AxisFaultFlags = 1566333786
				value.Axes[1].BrakeState = 1633705822
				value.Axes[1].VelocityActualCountsPerS = -123457
				value.Axes[1].FollowingErrorCounts = 790
				value.Axes[1].VelocityActualValid = 1
				value.Axes[1].FollowingErrorValid = 1
				value.Axes[1].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[1].MaxAbsFollowingErrorCounts = 790
				value.Axes[2].PosCounts = -1701077858
				value.Axes[2].TorqueRaw = -26470
				value.Axes[2].Statusword = 26984
				value.Axes[2].ErrorCode = 27498
				value.Axes[2].ModeDisplay = 108
				value.Axes[2].Ds402State = 109
				value.Axes[2].ManufacturerErrorCode = 1903193966
				value.Axes[2].DiBits = 1970566002
				value.Axes[2].AxisFaultFlags = 2037938038
				value.Axes[2].BrakeState = 2105310074
				value.Axes[2].VelocityActualCountsPerS = -123458
				value.Axes[2].FollowingErrorCounts = 791
				value.Axes[2].VelocityActualValid = 1
				value.Axes[2].FollowingErrorValid = 1
				value.Axes[2].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[2].MaxAbsFollowingErrorCounts = 791
				value.Axes[3].PosCounts = -33652606
				value.Axes[3].TorqueRaw = -1027
				value.Axes[3].Statusword = 1541
				value.Axes[3].ErrorCode = 2055
				value.Axes[3].ModeDisplay = 9
				value.Axes[3].Ds402State = 10
				value.Axes[3].ManufacturerErrorCode = 235736075
				value.Axes[3].DiBits = 303108111
				value.Axes[3].AxisFaultFlags = 370480147
				value.Axes[3].BrakeState = 437852183
				value.Axes[3].VelocityActualCountsPerS = -123459
				value.Axes[3].FollowingErrorCounts = 792
				value.Axes[3].VelocityActualValid = 1
				value.Axes[3].FollowingErrorValid = 1
				value.Axes[3].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[3].MaxAbsFollowingErrorCounts = 792
				value.Axes[4].PosCounts = -505224219
				value.Axes[4].TorqueRaw = -8223
				value.Axes[4].Statusword = 8737
				value.Axes[4].ErrorCode = 9251
				value.Axes[4].ModeDisplay = 37
				value.Axes[4].Ds402State = 38
				value.Axes[4].ManufacturerErrorCode = 707340327
				value.Axes[4].DiBits = 774712363
				value.Axes[4].AxisFaultFlags = 842084399
				value.Axes[4].BrakeState = 909456435
				value.Axes[4].VelocityActualCountsPerS = -123460
				value.Axes[4].FollowingErrorCounts = 793
				value.Axes[4].VelocityActualValid = 1
				value.Axes[4].FollowingErrorValid = 1
				value.Axes[4].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[4].MaxAbsFollowingErrorCounts = 793
				value.Axes[5].PosCounts = -976828471
				value.Axes[5].TorqueRaw = -15419
				value.Axes[5].Statusword = 15933
				value.Axes[5].ErrorCode = 16447
				value.Axes[5].ModeDisplay = 65
				value.Axes[5].Ds402State = 66
				value.Axes[5].ManufacturerErrorCode = 1178944579
				value.Axes[5].DiBits = 1246316615
				value.Axes[5].AxisFaultFlags = 1313688651
				value.Axes[5].BrakeState = 1381060687
				value.Axes[5].VelocityActualCountsPerS = -123461
				value.Axes[5].FollowingErrorCounts = 794
				value.Axes[5].VelocityActualValid = 1
				value.Axes[5].FollowingErrorValid = 1
				value.Axes[5].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[5].MaxAbsFollowingErrorCounts = 794
				value.Axes[6].PosCounts = -1448432723
				value.Axes[6].TorqueRaw = -22615
				value.Axes[6].Statusword = 23129
				value.Axes[6].ErrorCode = 23643
				value.Axes[6].ModeDisplay = 93
				value.Axes[6].Ds402State = 94
				value.Axes[6].ManufacturerErrorCode = 1650548831
				value.Axes[6].DiBits = 1717920867
				value.Axes[6].AxisFaultFlags = 1785292903
				value.Axes[6].BrakeState = 1852664939
				value.Axes[6].VelocityActualCountsPerS = -123462
				value.Axes[6].FollowingErrorCounts = 795
				value.Axes[6].VelocityActualValid = 1
				value.Axes[6].FollowingErrorValid = 1
				value.Axes[6].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[6].MaxAbsFollowingErrorCounts = 795
				value.Axes[7].PosCounts = -1920036975
				value.Axes[7].TorqueRaw = -29811
				value.Axes[7].Statusword = 30325
				value.Axes[7].ErrorCode = 30839
				value.Axes[7].ModeDisplay = 121
				value.Axes[7].Ds402State = 122
				value.Axes[7].ManufacturerErrorCode = 2122153083
				value.Axes[7].DiBits = 50463103
				value.Axes[7].AxisFaultFlags = 117835012
				value.Axes[7].BrakeState = 185207048
				value.Axes[7].VelocityActualCountsPerS = -123463
				value.Axes[7].FollowingErrorCounts = 796
				value.Axes[7].VelocityActualValid = 1
				value.Axes[7].FollowingErrorValid = 1
				value.Axes[7].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[7].MaxAbsFollowingErrorCounts = 796
				value.Axes[8].PosCounts = -252579084
				value.Axes[8].TorqueRaw = -4368
				value.Axes[8].Statusword = 4882
				value.Axes[8].ErrorCode = 5396
				value.Axes[8].ModeDisplay = 22
				value.Axes[8].Ds402State = 23
				value.Axes[8].ManufacturerErrorCode = 454695192
				value.Axes[8].DiBits = 522067228
				value.Axes[8].AxisFaultFlags = 589439264
				value.Axes[8].BrakeState = 656811300
				value.Axes[8].VelocityActualCountsPerS = -123464
				value.Axes[8].FollowingErrorCounts = 797
				value.Axes[8].VelocityActualValid = 1
				value.Axes[8].FollowingErrorValid = 1
				value.Axes[8].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[8].MaxAbsFollowingErrorCounts = 797
				value.Axes[9].PosCounts = -724183336
				value.Axes[9].TorqueRaw = -11564
				value.Axes[9].Statusword = 12078
				value.Axes[9].ErrorCode = 12592
				value.Axes[9].ModeDisplay = 50
				value.Axes[9].Ds402State = 51
				value.Axes[9].ManufacturerErrorCode = 926299444
				value.Axes[9].DiBits = 993671480
				value.Axes[9].AxisFaultFlags = 1061043516
				value.Axes[9].BrakeState = 1128415552
				value.Axes[9].VelocityActualCountsPerS = -123465
				value.Axes[9].FollowingErrorCounts = 798
				value.Axes[9].VelocityActualValid = 1
				value.Axes[9].FollowingErrorValid = 1
				value.Axes[9].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[9].MaxAbsFollowingErrorCounts = 798
				value.Axes[10].PosCounts = -1195787588
				value.Axes[10].TorqueRaw = -18760
				value.Axes[10].Statusword = 19274
				value.Axes[10].ErrorCode = 19788
				value.Axes[10].ModeDisplay = 78
				value.Axes[10].Ds402State = 79
				value.Axes[10].ManufacturerErrorCode = 1397903696
				value.Axes[10].DiBits = 1465275732
				value.Axes[10].AxisFaultFlags = 1532647768
				value.Axes[10].BrakeState = 1600019804
				value.Axes[10].VelocityActualCountsPerS = -123466
				value.Axes[10].FollowingErrorCounts = 799
				value.Axes[10].VelocityActualValid = 1
				value.Axes[10].FollowingErrorValid = 1
				value.Axes[10].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[10].MaxAbsFollowingErrorCounts = 799
				value.Axes[11].PosCounts = -1667391840
				value.Axes[11].TorqueRaw = -25956
				value.Axes[11].Statusword = 26470
				value.Axes[11].ErrorCode = 26984
				value.Axes[11].ModeDisplay = 106
				value.Axes[11].Ds402State = 107
				value.Axes[11].ManufacturerErrorCode = 1869507948
				value.Axes[11].DiBits = 1936879984
				value.Axes[11].AxisFaultFlags = 2004252020
				value.Axes[11].BrakeState = 2071624056
				value.Axes[11].VelocityActualCountsPerS = -123467
				value.Axes[11].FollowingErrorCounts = 800
				value.Axes[11].VelocityActualValid = 1
				value.Axes[11].FollowingErrorValid = 1
				value.Axes[11].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[11].MaxAbsFollowingErrorCounts = 800
				value.Axes[12].PosCounts = -2138996092
				value.Axes[12].TorqueRaw = -513
				value.Axes[12].Statusword = 1027
				value.Axes[12].ErrorCode = 1541
				value.Axes[12].ModeDisplay = 7
				value.Axes[12].Ds402State = 8
				value.Axes[12].ManufacturerErrorCode = 202050057
				value.Axes[12].DiBits = 269422093
				value.Axes[12].AxisFaultFlags = 336794129
				value.Axes[12].BrakeState = 404166165
				value.Axes[12].VelocityActualCountsPerS = -123468
				value.Axes[12].FollowingErrorCounts = 801
				value.Axes[12].VelocityActualValid = 1
				value.Axes[12].FollowingErrorValid = 1
				value.Axes[12].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[12].MaxAbsFollowingErrorCounts = 801
				value.Axes[13].PosCounts = -471538201
				value.Axes[13].TorqueRaw = -7709
				value.Axes[13].Statusword = 8223
				value.Axes[13].ErrorCode = 8737
				value.Axes[13].ModeDisplay = 35
				value.Axes[13].Ds402State = 36
				value.Axes[13].ManufacturerErrorCode = 673654309
				value.Axes[13].DiBits = 741026345
				value.Axes[13].AxisFaultFlags = 808398381
				value.Axes[13].BrakeState = 875770417
				value.Axes[13].VelocityActualCountsPerS = -123469
				value.Axes[13].FollowingErrorCounts = 802
				value.Axes[13].VelocityActualValid = 1
				value.Axes[13].FollowingErrorValid = 1
				value.Axes[13].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[13].MaxAbsFollowingErrorCounts = 802
				value.Axes[14].PosCounts = -943142453
				value.Axes[14].TorqueRaw = -14905
				value.Axes[14].Statusword = 15419
				value.Axes[14].ErrorCode = 15933
				value.Axes[14].ModeDisplay = 63
				value.Axes[14].Ds402State = 64
				value.Axes[14].ManufacturerErrorCode = 1145258561
				value.Axes[14].DiBits = 1212630597
				value.Axes[14].AxisFaultFlags = 1280002633
				value.Axes[14].BrakeState = 1347374669
				value.Axes[14].VelocityActualCountsPerS = -123470
				value.Axes[14].FollowingErrorCounts = 803
				value.Axes[14].VelocityActualValid = 1
				value.Axes[14].FollowingErrorValid = 1
				value.Axes[14].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[14].MaxAbsFollowingErrorCounts = 803
				value.Axes[15].PosCounts = -1414746705
				value.Axes[15].TorqueRaw = -22101
				value.Axes[15].Statusword = 22615
				value.Axes[15].ErrorCode = 23129
				value.Axes[15].ModeDisplay = 91
				value.Axes[15].Ds402State = 92
				value.Axes[15].ManufacturerErrorCode = 1616862813
				value.Axes[15].DiBits = 1684234849
				value.Axes[15].AxisFaultFlags = 1751606885
				value.Axes[15].BrakeState = 1818978921
				value.Axes[15].VelocityActualCountsPerS = -123471
				value.Axes[15].FollowingErrorCounts = 804
				value.Axes[15].VelocityActualValid = 1
				value.Axes[15].FollowingErrorValid = 1
				value.Axes[15].MaxAbsVelocityActualCountsPerS = 2147483648
				value.Axes[15].MaxAbsFollowingErrorCounts = 804
				return value
			}()},
		{name: "EventV1", size: 24, decode: func(p []byte) (any, error) { return wire.DecodeEventV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeEventV1(v.(wire.EventV1)); return p[:] },
			expected: func() any {
				value := wire.EventV1{}
				value.Code = 84148994
				value.Axis = 151521030
				value.Value0 = -1229499251294997258
				value.Value1 = -1808220633999610642
				return value
			}()},
		{name: "StatusHelloV1", size: 40, decode: func(p []byte) (any, error) { return wire.DecodeStatusHelloV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeStatusHelloV1(v.(wire.StatusHelloV1)); return p[:] },
			expected: func() any {
				value := wire.StatusHelloV1{}
				value.BuildIdHash = 650777868590383874
				value.TopologyHash = 1229499251294997258
				value.CycleNs = 1808220633999610642
				value.NumAxes = 488381210
				value.DriveProfileId = 555753246
				value.WkcExpected = 623125282
				return value
			}()},
		{name: "StatusAxisConfigV1", size: 424, decode: func(p []byte) (any, error) { return wire.DecodeStatusAxisConfigV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeStatusAxisConfigV1(v.(wire.StatusAxisConfigV1)); return p[:] },
			expected: func() any {
				value := wire.StatusAxisConfigV1{}
				value.NumAxes = 84148994
				value.CountsPerRev[0] = 218893066
				value.CountsPerRev[1] = 286265102
				value.CountsPerRev[2] = 353637138
				value.CountsPerRev[3] = 421009174
				value.CountsPerRev[4] = 488381210
				value.CountsPerRev[5] = 555753246
				value.CountsPerRev[6] = 623125282
				value.CountsPerRev[7] = 690497318
				value.CountsPerRev[8] = 757869354
				value.CountsPerRev[9] = 825241390
				value.CountsPerRev[10] = 892613426
				value.CountsPerRev[11] = 959985462
				value.CountsPerRev[12] = 1027357498
				value.CountsPerRev[13] = 1094729534
				value.CountsPerRev[14] = 1162101570
				value.CountsPerRev[15] = 1229473606
				value.GearRatio[0] = 73.25
				value.GearRatio[1] = 81.25
				value.GearRatio[2] = 89.25
				value.GearRatio[3] = 97.25
				value.GearRatio[4] = 105.25
				value.GearRatio[5] = 113.25
				value.GearRatio[6] = 121.25
				value.GearRatio[7] = 129.25
				value.GearRatio[8] = 137.25
				value.GearRatio[9] = 145.25
				value.GearRatio[10] = 153.25
				value.GearRatio[11] = 161.25
				value.GearRatio[12] = 169.25
				value.GearRatio[13] = 177.25
				value.GearRatio[14] = 185.25
				value.GearRatio[15] = 193.25
				value.Sign[0] = -1313688651
				value.Sign[1] = -1381060687
				value.Sign[2] = -1448432723
				value.Sign[3] = -1515804759
				value.Sign[4] = -1583176795
				value.Sign[5] = -1650548831
				value.Sign[6] = -1717920867
				value.Sign[7] = -1785292903
				value.Sign[8] = -1852664939
				value.Sign[9] = -1920036975
				value.Sign[10] = -1987409011
				value.Sign[11] = -2054781047
				value.Sign[12] = -2122153083
				value.Sign[13] = -50463103
				value.Sign[14] = -117835012
				value.Sign[15] = -185207048
				value.AxisType[0] = 12
				value.AxisType[1] = 13
				value.AxisType[2] = 14
				value.AxisType[3] = 15
				value.AxisType[4] = 16
				value.AxisType[5] = 17
				value.AxisType[6] = 18
				value.AxisType[7] = 19
				value.AxisType[8] = 20
				value.AxisType[9] = 21
				value.AxisType[10] = 22
				value.AxisType[11] = 23
				value.AxisType[12] = 24
				value.AxisType[13] = 25
				value.AxisType[14] = 26
				value.AxisType[15] = 27
				value.CountsPerUnit[0] = 297.25
				value.CountsPerUnit[1] = 305.25
				value.CountsPerUnit[2] = 313.25
				value.CountsPerUnit[3] = 321.25
				value.CountsPerUnit[4] = 329.25
				value.CountsPerUnit[5] = 337.25
				value.CountsPerUnit[6] = 345.25
				value.CountsPerUnit[7] = 353.25
				value.CountsPerUnit[8] = 361.25
				value.CountsPerUnit[9] = 369.25
				value.CountsPerUnit[10] = 377.25
				value.CountsPerUnit[11] = 385.25
				value.CountsPerUnit[12] = 393.25
				value.CountsPerUnit[13] = 401.25
				value.CountsPerUnit[14] = 409.25
				value.CountsPerUnit[15] = 417.25
				return value
			}()},
		{name: "StatusMotionStateV1", size: 64, decode: func(p []byte) (any, error) { return wire.DecodeStatusMotionStateV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeStatusMotionStateV1(v.(wire.StatusMotionStateV1)); return p[:] },
			expected: func() any {
				value := wire.StatusMotionStateV1{}
				value.ActiveMode = 84148994
				value.State = 151521030
				value.ActiveTrajId = 1229499251294997258
				value.CurrentPointIndex = 353637138
				value.QueueDepth = 421009174
				value.QueueCapacity = 488381210
				value.LastEventCode = 555753246
				value.UnderrunCount = 623125282
				value.StaleCommandFlag = 690497318
				value.MotionDone = 757869354
				value.CapabilityFlags = 6
				value.ActiveCommandSeq = 4123106164818064178
				value.LastUpdateNs = 4701827547522677562
				return value
			}()},
		{name: "StatusJogDebugV1", size: 360, decode: func(p []byte) (any, error) { return wire.DecodeStatusJogDebugV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeStatusJogDebugV1(v.(wire.StatusJogDebugV1)); return p[:] },
			expected: func() any {
				value := wire.StatusJogDebugV1{}
				value.NumAxes = 84148994
				value.ActiveJog = 151521030
				value.ActiveJogAxisMask = 218893066
				value.CommandSpMask = 286265102
				value.HaveHoldMask = 353637138
				value.HaveJogTargetMask = 421009174
				value.SnapHoldMask = 488381210
				value.LatestCmdAxisMask = 555753246
				value.LatestCmdFlags = 623125282
				value.LastStopReason = 690497318
				value.LastStopAxisMask = 757869354
				value.StopArrestMask = 825241390
				value.SampleTimeNs = 4123106164818064178
				value.ActiveJogCmdSeq = 4701827547522677562
				value.LatestJogSeqSeen = 5280548930227290946
				value.ActiveJogDeadlineNs = 5859270312931904330
				value.LatestCmdTimeoutNs = 6437991695636517714
				value.LastStopTimeNs = 7016713078341131098
				value.LastStopCmdSeq = 7595434461045744482
				value.FeedbackPosCounts[0] = -1835821930
				value.FeedbackPosCounts[1] = -1903193966
				value.FeedbackPosCounts[2] = -1970566002
				value.FeedbackPosCounts[3] = -2037938038
				value.FeedbackPosCounts[4] = -2105310074
				value.FeedbackPosCounts[5] = -33652606
				value.FeedbackPosCounts[6] = -100992003
				value.FeedbackPosCounts[7] = -168364039
				value.FeedbackPosCounts[8] = -235736075
				value.FeedbackPosCounts[9] = -303108111
				value.FeedbackPosCounts[10] = -370480147
				value.FeedbackPosCounts[11] = -437852183
				value.FeedbackPosCounts[12] = -505224219
				value.FeedbackPosCounts[13] = -572596255
				value.FeedbackPosCounts[14] = -639968291
				value.FeedbackPosCounts[15] = -707340327
				value.HoldTargetCounts[0] = -774712363
				value.HoldTargetCounts[1] = -842084399
				value.HoldTargetCounts[2] = -909456435
				value.HoldTargetCounts[3] = -976828471
				value.HoldTargetCounts[4] = -1044200507
				value.HoldTargetCounts[5] = -1111572543
				value.HoldTargetCounts[6] = -1178944579
				value.HoldTargetCounts[7] = -1246316615
				value.HoldTargetCounts[8] = -1313688651
				value.HoldTargetCounts[9] = -1381060687
				value.HoldTargetCounts[10] = -1448432723
				value.HoldTargetCounts[11] = -1515804759
				value.HoldTargetCounts[12] = -1583176795
				value.HoldTargetCounts[13] = -1650548831
				value.HoldTargetCounts[14] = -1717920867
				value.HoldTargetCounts[15] = -1785292903
				value.OutputTargetCounts[0] = -1852664939
				value.OutputTargetCounts[1] = -1920036975
				value.OutputTargetCounts[2] = -1987409011
				value.OutputTargetCounts[3] = -2054781047
				value.OutputTargetCounts[4] = -2122153083
				value.OutputTargetCounts[5] = -50463103
				value.OutputTargetCounts[6] = -117835012
				value.OutputTargetCounts[7] = -185207048
				value.OutputTargetCounts[8] = -252579084
				value.OutputTargetCounts[9] = -319951120
				value.OutputTargetCounts[10] = -387323156
				value.OutputTargetCounts[11] = -454695192
				value.OutputTargetCounts[12] = -522067228
				value.OutputTargetCounts[13] = -589439264
				value.OutputTargetCounts[14] = -656811300
				value.OutputTargetCounts[15] = -724183336
				value.OutputTargetVelocityCountsPerS[0] = -791555372
				value.OutputTargetVelocityCountsPerS[1] = -858927408
				value.OutputTargetVelocityCountsPerS[2] = -926299444
				value.OutputTargetVelocityCountsPerS[3] = -993671480
				value.OutputTargetVelocityCountsPerS[4] = -1061043516
				value.OutputTargetVelocityCountsPerS[5] = -1128415552
				value.OutputTargetVelocityCountsPerS[6] = -1195787588
				value.OutputTargetVelocityCountsPerS[7] = -1263159624
				value.OutputTargetVelocityCountsPerS[8] = -1330531660
				value.OutputTargetVelocityCountsPerS[9] = -1397903696
				value.OutputTargetVelocityCountsPerS[10] = -1465275732
				value.OutputTargetVelocityCountsPerS[11] = -1532647768
				value.OutputTargetVelocityCountsPerS[12] = -1600019804
				value.OutputTargetVelocityCountsPerS[13] = -1667391840
				value.OutputTargetVelocityCountsPerS[14] = -1734763876
				value.OutputTargetVelocityCountsPerS[15] = -1802135912
				return value
			}()},
		{name: "AxisStatusExtV1", size: 24, decode: func(p []byte) (any, error) { return wire.DecodeAxisStatusExtV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeAxisStatusExtV1(v.(wire.AxisStatusExtV1)); return p[:] },
			expected: func() any {
				value := wire.AxisStatusExtV1{}
				value.PositionErrorCounts = -84148994
				value.MultiTurnLo = -151521030
				value.MultiTurnHi = -218893066
				value.BusVoltageRaw = 3854
				value.LoadRateRaw = 4368
				value.IgbtTempRaw = -4882
				value.MotorTempRaw = -5396
				value.DriveNotReadyBits = 5910
				value.MotorNotRotatingCode = 6424
				return value
			}()},
		{name: "StatusExtendedSnapshotV1", size: 400, decode: func(p []byte) (any, error) { return wire.DecodeStatusExtendedSnapshotV1Frame(p) },
			encode: func(v any) []byte {
				p := wire.EncodeStatusExtendedSnapshotV1(v.(wire.StatusExtendedSnapshotV1))
				return p[:]
			},
			expected: func() any {
				value := wire.StatusExtendedSnapshotV1{}
				value.NumAxes = 84148994
				value.ValidAxisMask = 151521030
				value.SampleTimeNs = 1229499251294997258
				value.Axes[0].PositionErrorCounts = -353637138
				value.Axes[0].MultiTurnLo = -421009174
				value.Axes[0].MultiTurnHi = -488381210
				value.Axes[0].BusVoltageRaw = 7966
				value.Axes[0].LoadRateRaw = 8480
				value.Axes[0].IgbtTempRaw = -8994
				value.Axes[0].MotorTempRaw = -9508
				value.Axes[0].DriveNotReadyBits = 10022
				value.Axes[0].MotorNotRotatingCode = 10536
				value.Axes[1].PositionErrorCounts = -757869354
				value.Axes[1].MultiTurnLo = -825241390
				value.Axes[1].MultiTurnHi = -892613426
				value.Axes[1].BusVoltageRaw = 14134
				value.Axes[1].LoadRateRaw = 14648
				value.Axes[1].IgbtTempRaw = -15162
				value.Axes[1].MotorTempRaw = -15676
				value.Axes[1].DriveNotReadyBits = 16190
				value.Axes[1].MotorNotRotatingCode = 16704
				value.Axes[2].PositionErrorCounts = -1162101570
				value.Axes[2].MultiTurnLo = -1229473606
				value.Axes[2].MultiTurnHi = -1296845642
				value.Axes[2].BusVoltageRaw = 20302
				value.Axes[2].LoadRateRaw = 20816
				value.Axes[2].IgbtTempRaw = -21330
				value.Axes[2].MotorTempRaw = -21844
				value.Axes[2].DriveNotReadyBits = 22358
				value.Axes[2].MotorNotRotatingCode = 22872
				value.Axes[3].PositionErrorCounts = -1566333786
				value.Axes[3].MultiTurnLo = -1633705822
				value.Axes[3].MultiTurnHi = -1701077858
				value.Axes[3].BusVoltageRaw = 26470
				value.Axes[3].LoadRateRaw = 26984
				value.Axes[3].IgbtTempRaw = -27498
				value.Axes[3].MotorTempRaw = -28012
				value.Axes[3].DriveNotReadyBits = 28526
				value.Axes[3].MotorNotRotatingCode = 29040
				value.Axes[4].PositionErrorCounts = -1970566002
				value.Axes[4].MultiTurnLo = -2037938038
				value.Axes[4].MultiTurnHi = -2105310074
				value.Axes[4].BusVoltageRaw = 32638
				value.Axes[4].LoadRateRaw = 513
				value.Axes[4].IgbtTempRaw = -1027
				value.Axes[4].MotorTempRaw = -1541
				value.Axes[4].DriveNotReadyBits = 2055
				value.Axes[4].MotorNotRotatingCode = 2569
				value.Axes[5].PositionErrorCounts = -235736075
				value.Axes[5].MultiTurnLo = -303108111
				value.Axes[5].MultiTurnHi = -370480147
				value.Axes[5].BusVoltageRaw = 6167
				value.Axes[5].LoadRateRaw = 6681
				value.Axes[5].IgbtTempRaw = -7195
				value.Axes[5].MotorTempRaw = -7709
				value.Axes[5].DriveNotReadyBits = 8223
				value.Axes[5].MotorNotRotatingCode = 8737
				value.Axes[6].PositionErrorCounts = -639968291
				value.Axes[6].MultiTurnLo = -707340327
				value.Axes[6].MultiTurnHi = -774712363
				value.Axes[6].BusVoltageRaw = 12335
				value.Axes[6].LoadRateRaw = 12849
				value.Axes[6].IgbtTempRaw = -13363
				value.Axes[6].MotorTempRaw = -13877
				value.Axes[6].DriveNotReadyBits = 14391
				value.Axes[6].MotorNotRotatingCode = 14905
				value.Axes[7].PositionErrorCounts = -1044200507
				value.Axes[7].MultiTurnLo = -1111572543
				value.Axes[7].MultiTurnHi = -1178944579
				value.Axes[7].BusVoltageRaw = 18503
				value.Axes[7].LoadRateRaw = 19017
				value.Axes[7].IgbtTempRaw = -19531
				value.Axes[7].MotorTempRaw = -20045
				value.Axes[7].DriveNotReadyBits = 20559
				value.Axes[7].MotorNotRotatingCode = 21073
				value.Axes[8].PositionErrorCounts = -1448432723
				value.Axes[8].MultiTurnLo = -1515804759
				value.Axes[8].MultiTurnHi = -1583176795
				value.Axes[8].BusVoltageRaw = 24671
				value.Axes[8].LoadRateRaw = 25185
				value.Axes[8].IgbtTempRaw = -25699
				value.Axes[8].MotorTempRaw = -26213
				value.Axes[8].DriveNotReadyBits = 26727
				value.Axes[8].MotorNotRotatingCode = 27241
				value.Axes[9].PositionErrorCounts = -1852664939
				value.Axes[9].MultiTurnLo = -1920036975
				value.Axes[9].MultiTurnHi = -1987409011
				value.Axes[9].BusVoltageRaw = 30839
				value.Axes[9].LoadRateRaw = 31353
				value.Axes[9].IgbtTempRaw = -31867
				value.Axes[9].MotorTempRaw = -32381
				value.Axes[9].DriveNotReadyBits = 383
				value.Axes[9].MotorNotRotatingCode = 770
				value.Axes[10].PositionErrorCounts = -117835012
				value.Axes[10].MultiTurnLo = -185207048
				value.Axes[10].MultiTurnHi = -252579084
				value.Axes[10].BusVoltageRaw = 4368
				value.Axes[10].LoadRateRaw = 4882
				value.Axes[10].IgbtTempRaw = -5396
				value.Axes[10].MotorTempRaw = -5910
				value.Axes[10].DriveNotReadyBits = 6424
				value.Axes[10].MotorNotRotatingCode = 6938
				value.Axes[11].PositionErrorCounts = -522067228
				value.Axes[11].MultiTurnLo = -589439264
				value.Axes[11].MultiTurnHi = -656811300
				value.Axes[11].BusVoltageRaw = 10536
				value.Axes[11].LoadRateRaw = 11050
				value.Axes[11].IgbtTempRaw = -11564
				value.Axes[11].MotorTempRaw = -12078
				value.Axes[11].DriveNotReadyBits = 12592
				value.Axes[11].MotorNotRotatingCode = 13106
				value.Axes[12].PositionErrorCounts = -926299444
				value.Axes[12].MultiTurnLo = -993671480
				value.Axes[12].MultiTurnHi = -1061043516
				value.Axes[12].BusVoltageRaw = 16704
				value.Axes[12].LoadRateRaw = 17218
				value.Axes[12].IgbtTempRaw = -17732
				value.Axes[12].MotorTempRaw = -18246
				value.Axes[12].DriveNotReadyBits = 18760
				value.Axes[12].MotorNotRotatingCode = 19274
				value.Axes[13].PositionErrorCounts = -1330531660
				value.Axes[13].MultiTurnLo = -1397903696
				value.Axes[13].MultiTurnHi = -1465275732
				value.Axes[13].BusVoltageRaw = 22872
				value.Axes[13].LoadRateRaw = 23386
				value.Axes[13].IgbtTempRaw = -23900
				value.Axes[13].MotorTempRaw = -24414
				value.Axes[13].DriveNotReadyBits = 24928
				value.Axes[13].MotorNotRotatingCode = 25442
				value.Axes[14].PositionErrorCounts = -1734763876
				value.Axes[14].MultiTurnLo = -1802135912
				value.Axes[14].MultiTurnHi = -1869507948
				value.Axes[14].BusVoltageRaw = 29040
				value.Axes[14].LoadRateRaw = 29554
				value.Axes[14].IgbtTempRaw = -30068
				value.Axes[14].MotorTempRaw = -30582
				value.Axes[14].DriveNotReadyBits = 31096
				value.Axes[14].MotorNotRotatingCode = 31610
				value.Axes[15].PositionErrorCounts = -2138996092
				value.Axes[15].MultiTurnLo = -67305985
				value.Axes[15].MultiTurnHi = -134678021
				value.Axes[15].BusVoltageRaw = 2569
				value.Axes[15].LoadRateRaw = 3083
				value.Axes[15].IgbtTempRaw = -3597
				value.Axes[15].MotorTempRaw = -4111
				value.Axes[15].DriveNotReadyBits = 4625
				value.Axes[15].MotorNotRotatingCode = 5139
				return value
			}()},
		{name: "CmdArmV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdArmV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdArmV1(v.(wire.CmdArmV1)); return p[:] },
			expected: func() any {
				value := wire.CmdArmV1{}
				value.Arm = 84148994
				return value
			}()},
		{name: "CmdAxisMaskV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdAxisMaskV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdAxisMaskV1(v.(wire.CmdAxisMaskV1)); return p[:] },
			expected: func() any {
				value := wire.CmdAxisMaskV1{}
				value.AxisMask = 84148994
				return value
			}()},
		{name: "CmdFaultResetV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdFaultResetV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdFaultResetV1(v.(wire.CmdFaultResetV1)); return p[:] },
			expected: func() any {
				value := wire.CmdFaultResetV1{}
				value.AxisMask = 84148994
				return value
			}()},
		{name: "CmdNativeHomeV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdNativeHomeV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdNativeHomeV1(v.(wire.CmdNativeHomeV1)); return p[:] },
			expected: func() any {
				value := wire.CmdNativeHomeV1{}
				value.AxisMask = 84148994
				return value
			}()},
		{name: "CmdServiceSdoWriteV1", size: 16, decode: func(p []byte) (any, error) { return wire.DecodeCmdServiceSdoWriteV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdServiceSdoWriteV1(v.(wire.CmdServiceSdoWriteV1)); return p[:] },
			expected: func() any {
				value := wire.CmdServiceSdoWriteV1{}
				value.AxisMask = 84148994
				value.Index = 1798
				value.Subindex = 8
				value.ValueType = 9
				value.ValueU32 = 218893066
				return value
			}()},
		{name: "CmdSetModeV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdSetModeV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdSetModeV1(v.(wire.CmdSetModeV1)); return p[:] },
			expected: func() any {
				value := wire.CmdSetModeV1{}
				value.AxisMask = 84148994
				value.Mode = 151521030
				return value
			}()},
		{name: "CmdSoftLimitsV1", size: 264, decode: func(p []byte) (any, error) { return wire.DecodeCmdSoftLimitsV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdSoftLimitsV1(v.(wire.CmdSoftLimitsV1)); return p[:] },
			expected: func() any {
				value := wire.CmdSoftLimitsV1{}
				value.AxisMask = 84148994
				value.MinQ[0] = 9.25
				value.MinQ[1] = 17.25
				value.MinQ[2] = 25.25
				value.MinQ[3] = 33.25
				value.MinQ[4] = 41.25
				value.MinQ[5] = 49.25
				value.MinQ[6] = 57.25
				value.MinQ[7] = 65.25
				value.MinQ[8] = 73.25
				value.MinQ[9] = 81.25
				value.MinQ[10] = 89.25
				value.MinQ[11] = 97.25
				value.MinQ[12] = 105.25
				value.MinQ[13] = 113.25
				value.MinQ[14] = 121.25
				value.MinQ[15] = 129.25
				value.MaxQ[0] = 137.25
				value.MaxQ[1] = 145.25
				value.MaxQ[2] = 153.25
				value.MaxQ[3] = 161.25
				value.MaxQ[4] = 169.25
				value.MaxQ[5] = 177.25
				value.MaxQ[6] = 185.25
				value.MaxQ[7] = 193.25
				value.MaxQ[8] = 201.25
				value.MaxQ[9] = 209.25
				value.MaxQ[10] = 217.25
				value.MaxQ[11] = 225.25
				value.MaxQ[12] = 233.25
				value.MaxQ[13] = 241.25
				value.MaxQ[14] = 249.25
				value.MaxQ[15] = 257.25
				return value
			}()},
		{name: "CmdRequestBundleV1", size: 8, decode: func(p []byte) (any, error) { return wire.DecodeCmdRequestBundleV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdRequestBundleV1(v.(wire.CmdRequestBundleV1)); return p[:] },
			expected: func() any {
				value := wire.CmdRequestBundleV1{}
				value.Reason = 84148994
				return value
			}()},
		{name: "CmdIoWriteV1", size: 32, decode: func(p []byte) (any, error) { return wire.DecodeCmdIoWriteV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdIoWriteV1(v.(wire.CmdIoWriteV1)); return p[:] },
			expected: func() any {
				value := wire.CmdIoWriteV1{}
				value.IoId = 84148994
				value.DoMask = 1229499251294997258
				value.DoValue = 1808220633999610642
				value.ApplyTimeNs = 2386942016704224026
				return value
			}()},
		{name: "CmdTrajectoryBeginV1", size: 24, decode: func(p []byte) (any, error) { return wire.DecodeCmdTrajectoryBeginV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdTrajectoryBeginV1(v.(wire.CmdTrajectoryBeginV1)); return p[:] },
			expected: func() any {
				value := wire.CmdTrajectoryBeginV1{}
				value.TrajId = 650777868590383874
				value.AxisMask = 218893066
				value.ExpectedPoints = 353637138
				return value
			}()},
		{name: "TrajectoryPointV1", size: 320, decode: func(p []byte) (any, error) { return wire.DecodeTrajectoryPointV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeTrajectoryPointV1(v.(wire.TrajectoryPointV1)); return p[:] },
			expected: func() any {
				value := wire.TrajectoryPointV1{}
				value.TrajId = 650777868590383874
				value.PointIndex = 218893066
				value.Flags = 3
				value.TFromStartNs = 1808220633999610642
				value.Q[0] = 25.25
				value.Q[1] = 33.25
				value.Q[2] = 41.25
				value.Q[3] = 49.25
				value.Q[4] = 57.25
				value.Q[5] = 65.25
				value.Q[6] = 73.25
				value.Q[7] = 81.25
				value.Q[8] = 89.25
				value.Q[9] = 97.25
				value.Q[10] = 105.25
				value.Q[11] = 113.25
				value.Q[12] = 121.25
				value.Q[13] = 129.25
				value.Q[14] = 137.25
				value.Q[15] = 145.25
				value.Qd[0] = 153.25
				value.Qd[1] = 161.25
				value.Qd[2] = 169.25
				value.Qd[3] = 177.25
				value.Qd[4] = 185.25
				value.Qd[5] = 193.25
				value.Qd[6] = 201.25
				value.Qd[7] = 209.25
				value.Qd[8] = 217.25
				value.Qd[9] = 225.25
				value.Qd[10] = 233.25
				value.Qd[11] = 241.25
				value.Qd[12] = 249.25
				value.Qd[13] = 257.25
				value.Qd[14] = 265.25
				value.Qd[15] = 273.25
				value.AxisMask = 522067228
				value.WaitTimeoutNs = 12345678
				value.WaitStableNs = 1234
				value.WaitInput = 13
				value.WaitValue = 1
				value.MonitorMask = 0x2040
				value.MonitorValues = 0x2000
				return value
			}()},
		{name: "CmdTrajectoryControlV1", size: 16, decode: func(p []byte) (any, error) { return wire.DecodeCmdTrajectoryControlV1Frame(p) },
			encode: func(v any) []byte {
				p := wire.EncodeCmdTrajectoryControlV1(v.(wire.CmdTrajectoryControlV1))
				return p[:]
			},
			expected: func() any {
				value := wire.CmdTrajectoryControlV1{}
				value.TrajId = 650777868590383874
				return value
			}()},
		{name: "CmdJogV1", size: 144, decode: func(p []byte) (any, error) { return wire.DecodeCmdJogV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeCmdJogV1(v.(wire.CmdJogV1)); return p[:] },
			expected: func() any {
				value := wire.CmdJogV1{}
				value.AxisMask = 84148994
				value.Flags = 7
				value.TimeoutNs = 1229499251294997258
				value.Qd[0] = 17.25
				value.Qd[1] = 25.25
				value.Qd[2] = 33.25
				value.Qd[3] = 41.25
				value.Qd[4] = 49.25
				value.Qd[5] = 57.25
				value.Qd[6] = 65.25
				value.Qd[7] = 73.25
				value.Qd[8] = 81.25
				value.Qd[9] = 89.25
				value.Qd[10] = 97.25
				value.Qd[11] = 105.25
				value.Qd[12] = 113.25
				value.Qd[13] = 121.25
				value.Qd[14] = 129.25
				value.Qd[15] = 137.25
				return value
			}()},
		{name: "IoDevStatusV1", size: 32, decode: func(p []byte) (any, error) { return wire.DecodeIoDevStatusV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoDevStatusV1(v.(wire.IoDevStatusV1)); return p[:] },
			expected: func() any {
				value := wire.IoDevStatusV1{}
				value.IoId = 84148994
				value.WidthBits = 151521030
				value.DiBits = 1229499251294997258
				value.DoBits = 1808220633999610642
				value.FaultFlags = 488381210
				return value
			}()},
		{name: "IoSnapshotV1", size: 264, decode: func(p []byte) (any, error) { return wire.DecodeIoSnapshotV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoSnapshotV1(v.(wire.IoSnapshotV1)); return p[:] },
			expected: func() any {
				value := wire.IoSnapshotV1{}
				value.NumDevices = 84148994
				value.Dev[0].IoId = 218893066
				value.Dev[0].WidthBits = 286265102
				value.Dev[0].DiBits = 1808220633999610642
				value.Dev[0].DoBits = 2386942016704224026
				value.Dev[0].FaultFlags = 623125282
				value.Dev[1].IoId = 757869354
				value.Dev[1].WidthBits = 825241390
				value.Dev[1].DiBits = 4123106164818064178
				value.Dev[1].DoBits = 4701827547522677562
				value.Dev[1].FaultFlags = 1162101570
				value.Dev[2].IoId = 1296845642
				value.Dev[2].WidthBits = 1364217678
				value.Dev[2].DiBits = 6437991695636517714
				value.Dev[2].DoBits = 7016713078341131098
				value.Dev[2].FaultFlags = 1701077858
				value.Dev[3].IoId = 1835821930
				value.Dev[3].WidthBits = 1903193966
				value.Dev[3].DiBits = 8752877226454971250
				value.Dev[3].DoBits = 144536844300483450
				value.Dev[3].FaultFlags = 100992003
				value.Dev[4].IoId = 235736075
				value.Dev[4].WidthBits = 303108111
				value.Dev[4].DiBits = 1880560806837687315
				value.Dev[4].DoBits = 2459282189542300699
				value.Dev[4].FaultFlags = 639968291
				value.Dev[5].IoId = 774712363
				value.Dev[5].WidthBits = 842084399
				value.Dev[5].DiBits = 4195446337656140851
				value.Dev[5].DoBits = 4774167720360754235
				value.Dev[5].FaultFlags = 1178944579
				value.Dev[6].IoId = 1313688651
				value.Dev[6].WidthBits = 1381060687
				value.Dev[6].DiBits = 6510331868474594387
				value.Dev[6].DoBits = 7089053251179207771
				value.Dev[6].FaultFlags = 1717920867
				value.Dev[7].IoId = 1852664939
				value.Dev[7].WidthBits = 1920036975
				value.Dev[7].DiBits = 8825217399293047923
				value.Dev[7].DoBits = 216737379161832571
				value.Dev[7].FaultFlags = 117835012
				return value
			}()},
	}...)
}
