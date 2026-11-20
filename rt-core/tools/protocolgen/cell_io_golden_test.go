package main

import wire "rosieos/rt-core/ipcclient"

// Independently packed little-endian fixtures pin the new I/O records without
// regenerating the retained drive bytes or conflating command with readback.
func cellIOGoldenCases() []goldenCase {
	input := wire.IoInputV1{ObservedNs: 1000000, Value: 1, Valid: 1, Bit: 0, Fast: 1}
	copy(input.Name[:], "consent")
	output := wire.IoOutputV1{ExpiryNs: 5000000, ObservedNs: 2000000, Intent: 1, Commanded: 0, Readback: 1, Valid: 1, ChangedNs: 1000000, MarkerSampleNs: 1000000, MarkerCycle: 2}
	copy(output.Name[:], "gas")
	state := wire.IoStateV1{TimeNs: 2000000, Cycle: 2, Armed: 1, InputCount: 1, OutputCount: 1}
	state.Inputs[0], state.Outputs[0] = input, output
	return []goldenCase{
		{name: "IoTest", size: 40, expected: wire.IoTest{NativeGeneration: 2, ApplicationGeneration: 3, SourceSequence: 4, DeadlineNs: 5000000, Index: 13, Value: 3},
			decode: func(p []byte) (any, error) { return wire.DecodeIoTestFrame(p) }, encode: func(v any) []byte { p := wire.EncodeIoTest(v.(wire.IoTest)); return p[:] }},
		{name: "IoRequest", size: 16, expected: wire.IoRequest{NativeGeneration: 2, ApplicationGeneration: 3},
			decode: func(p []byte) (any, error) { return wire.DecodeIoRequestFrame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoRequest(v.(wire.IoRequest)); return p[:] }},
		{name: "IoInputV1", size: 56, expected: input,
			decode: func(p []byte) (any, error) { return wire.DecodeIoInputV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoInputV1(v.(wire.IoInputV1)); return p[:] }},
		{name: "IoOutputV1", size: 96, expected: output,
			decode: func(p []byte) (any, error) { return wire.DecodeIoOutputV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoOutputV1(v.(wire.IoOutputV1)); return p[:] }},
		{name: "IoStateV1", size: 2464, expected: state,
			decode: func(p []byte) (any, error) { return wire.DecodeIoStateV1Frame(p) },
			encode: func(v any) []byte { p := wire.EncodeIoStateV1(v.(wire.IoStateV1)); return p[:] }},
	}
}
