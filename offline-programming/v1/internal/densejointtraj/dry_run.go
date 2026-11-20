package densejointtraj

import (
	"crypto/sha256"
	"encoding/binary"
	"encoding/hex"
	"encoding/json"
)

// WithoutProcessOutputs validates the original artifact before deriving an
// execution copy. Every motion byte, segment clock and identity field is kept;
// only torch bits, process markers and the resulting content hashes change.
// Already motion-only content is idempotent. Header-only process markers are
// outside the v1 trajectory digest, so removing only those retains that digest.
func WithoutProcessOutputs(blob []byte) ([]byte, *Plan, error) {
	plan, err := Decode(blob)
	if err != nil {
		return nil, nil, err
	}
	headerLen := int(binary.BigEndian.Uint32(blob[:4]))
	var header map[string]json.RawMessage
	if err := json.Unmarshal(blob[4:4+headerLen], &header); err != nil {
		return nil, nil, err
	}
	delete(header, "process_markers")
	body := append([]byte(nil), blob[4+headerLen:]...)
	var segments []map[string]json.RawMessage
	if err := json.Unmarshal(header["segments"], &segments); err != nil {
		return nil, nil, err
	}
	for i := range plan.Segments {
		meta := &plan.Header.Segments[i]
		block := body[meta.Block.ByteOffset : meta.Block.ByteOffset+meta.Block.ByteLength]
		for j := range plan.Segments[i].Samples {
			plan.Segments[i].Samples[j].TorchOn = false
			block[(j+1)*RecordBytes-1] = 0
		}
		sum := sha256.Sum256(block)
		meta.Block.SHA256 = "sha256:" + hex.EncodeToString(sum[:])
		var blockHeader map[string]json.RawMessage
		if err := json.Unmarshal(segments[i]["block"], &blockHeader); err != nil {
			return nil, nil, err
		}
		blockHeader["sha256"], _ = json.Marshal(meta.Block.SHA256)
		segments[i]["block"], _ = json.Marshal(blockHeader)
	}
	plan.Header.TrajectoryDigest = TrajectoryDigest(plan.Header.Axes.AxisMask,
		plan.Header.Sampling.DtS, plan.Header.Limits.MaxAbsQdRadS, plan.Segments)
	header["trajectory_digest"], _ = json.Marshal(plan.Header.TrajectoryDigest)
	header["segments"], _ = json.Marshal(segments)
	raw, err := json.Marshal(header)
	if err != nil {
		return nil, nil, err
	}
	out := make([]byte, 4, 4+len(raw)+len(body))
	binary.BigEndian.PutUint32(out, uint32(len(raw)))
	out = append(out, raw...)
	out = append(out, body...)
	return out, plan, nil
}
