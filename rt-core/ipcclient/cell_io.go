package ipcclient

// IoState is a native-cycle observation; a command or intent never establishes
// physical feedback. Invalid inputs are unavailable and fast contacts fail closed.
type IoState struct {
	TimeNS  uint64     `json:"time_ns"`
	Cycle   uint64     `json:"cycle"`
	Armed   bool       `json:"armed"`
	Reason  uint32     `json:"reason"`
	Inputs  []IoInput  `json:"inputs"`
	Outputs []IoOutput `json:"outputs"`
}
type IoInput struct {
	Name       string `json:"name"`
	Value      bool   `json:"value"`
	Valid      bool   `json:"valid"`
	ObservedNS uint64 `json:"observed_ns"`
	Bit        uint32 `json:"bit"`
	Fast       bool   `json:"fast"`
}
type IoOutput struct {
	Name           string `json:"name"`
	Intent         bool   `json:"intent"`
	Commanded      bool   `json:"commanded"`
	Readback       bool   `json:"readback"`
	Valid          bool   `json:"valid"`
	ExpiryNS       uint64 `json:"expiry_ns"`
	ObservedNS     uint64 `json:"observed_ns"`
	ChangedNS      uint64 `json:"changed_ns"`
	MarkerSampleNS uint64 `json:"marker_sample_ns"`
	MarkerCycle    uint64 `json:"marker_cycle"`
	Bit            uint32 `json:"bit"`
	Torch          bool   `json:"torch"`
}
