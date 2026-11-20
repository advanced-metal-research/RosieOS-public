// Self-contained runtime test for dense_joint_trajectory_format.hpp, run by
// `make test`. Asserts against the reference fixture in ../testdata -- the
// cross-language arbiter the Python encoder wrote and the Go twin also reads.
// If this binary and the fixture disagree, this code is wrong (or the format
// changed and the fixture was regenerated, in which case all three languages
// fail together -- the alarm working).

#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "dense_joint_trajectory_format.hpp"

namespace {

int failures = 0;

void expect(bool condition, const std::string& what) {
  if (!condition) {
    std::fprintf(stderr, "FAIL: %s\n", what.c_str());
    ++failures;
  }
}

std::vector<unsigned char> read_file(const std::string& path) {
  std::ifstream input(path, std::ios::binary);
  if (!input) {
    std::fprintf(stderr, "FAIL: cannot open %s\n", path.c_str());
    ++failures;
    return {};
  }
  std::ostringstream buffer;
  buffer << input.rdbuf();
  const std::string& data = buffer.str();
  return std::vector<unsigned char>(data.begin(), data.end());
}

}  // namespace

int main() {
  const std::string testdata = DENSE_JOINT_TRAJECTORY_TESTDATA_DIR;
  const std::vector<unsigned char> blob =
      read_file(testdata + "/dense_joint_trajectory_reference.rdt");
  const std::vector<unsigned char> expected_raw =
      read_file(testdata + "/dense_joint_trajectory_reference.expected.json");
  if (failures != 0) return 1;

  dense_joint_trajectory::JsonValue expected;
  {
    dense_joint_trajectory::JsonParser parser(
        reinterpret_cast<const char*>(expected_raw.data()), expected_raw.size());
    expect(parser.parse(&expected), "expected sidecar parses as JSON");
  }
  std::string want_digest;
  std::uint64_t want_total = 0;
  dense_joint_trajectory::json_str(expected.find("trajectory_digest"), &want_digest);
  dense_joint_trajectory::json_u64(expected.find("total_sample_count"), &want_total);
  expect(!want_digest.empty(), "expected sidecar names a digest");

  // Decode + digest agreement with the reference fixture.
  dense_joint_trajectory::Plan plan;
  {
    const dense_joint_trajectory::Rejection result =
        dense_joint_trajectory::decode(blob.data(), blob.size(), &plan);
    expect(result.ok, "reference fixture decodes (" + result.reason + ": " +
                          result.detail + ")");
  }
  expect(plan.header.trajectory_digest == want_digest,
         "header digest matches expected sidecar");
  const std::string recomputed = dense_joint_trajectory::trajectory_digest(
      plan.header.axis_mask, plan.header.dt_s, plan.header.max_abs_qd_rad_s,
      plan.segments);
  expect(recomputed == want_digest,
         "recomputed digest " + recomputed + " matches " + want_digest);
  std::size_t total = 0;
  for (const auto& segment : plan.segments) total += segment.samples.size();
  expect(total == want_total, "total sample count matches expected sidecar");
  expect(plan.segments.size() == 2, "fixture has 2 segments");

  // Re-encoded blocks must be byte-identical to the stored ones: decode is
  // lossless and encode is canonical.
  {
    const std::size_t header_len =
        (std::size_t(blob[0]) << 24u) | (std::size_t(blob[1]) << 16u) |
        (std::size_t(blob[2]) << 8u) | std::size_t(blob[3]);
    std::size_t offset = 4 + header_len;
    for (std::size_t i = 0; i < plan.segments.size(); ++i) {
      const std::vector<unsigned char> reencoded =
          dense_joint_trajectory::encode_block(plan.segments[i].samples);
      expect(std::equal(reencoded.begin(), reencoded.end(), blob.begin() + offset),
             "segment " + std::to_string(i) + " re-encodes byte-identically");
      offset += reencoded.size();
    }
    expect(offset == blob.size(), "blocks cover the whole blob body");
  }

  // Torch bits live only in the weld segment; the reference weld has 4.
  {
    std::size_t torch_samples = 0;
    for (const auto& segment : plan.segments) {
      for (const auto& sample : segment.samples) {
        if (sample.torch_on) {
          expect(segment.kind == "weld", "torch bit only in weld segments");
          ++torch_samples;
        }
      }
    }
    expect(torch_samples == 4, "reference weld carries 4 torch-on samples");
  }

  // One flipped bit in the LAST block must be refused by that segment's
  // hash, naming segment 1 -- corruption is localized, not just detected.
  {
    std::vector<unsigned char> corrupted = blob;
    corrupted[corrupted.size() - 10] ^= 0x01;
    dense_joint_trajectory::Plan scratch;
    const dense_joint_trajectory::Rejection result =
        dense_joint_trajectory::decode(corrupted.data(), corrupted.size(), &scratch);
    expect(!result.ok, "bit-flipped blob is refused");
    expect(result.reason == "block_sha256_mismatch" && result.segment_index == 1,
           "bit flip is reported as block_sha256_mismatch on segment 1, got " +
               result.reason + " on segment " +
               std::to_string(result.segment_index));
  }

  // Truncation is a length mismatch, before any hash confusion.
  {
    dense_joint_trajectory::Plan scratch;
    const dense_joint_trajectory::Rejection result = dense_joint_trajectory::decode(
        blob.data(), blob.size() - dense_joint_trajectory::kRecordBytes, &scratch);
    expect(!result.ok && result.reason == "blob_length_mismatch",
           "truncated blob is refused as blob_length_mismatch, got " +
               result.reason);
  }

  // Motion validation rejections, each an absolute-unit violation.
  {
    const auto mutated = [&](void (*mutate)(dense_joint_trajectory::Plan*)) {
      dense_joint_trajectory::Plan copy = plan;
      mutate(&copy);
      return dense_joint_trajectory::validate_motion(copy);
    };
    // 1e-5 rad/s boundary qd: an order of magnitude over the 1e-6 rest gate.
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[0].samples[0].qd_rad_s[3] = 1e-5;
           }).reason == "boundary_qd_nonzero",
           "boundary_qd_nonzero fires at 1e-5 rad/s");
    // 0.002 rad handover gap: twice the 1e-3 rad tolerance.
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[1].samples[0].q_rad[0] += 0.002;
           }).reason == "boundary_q_discontinuity",
           "boundary_q_discontinuity fires at 0.002 rad");
    // 3.5 rad/s against the 3.0 rad/s ceiling.
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[0].samples[2].qd_rad_s[0] = 3.5;
           }).reason == "qd_limit_exceeded",
           "qd_limit_exceeded fires at 3.5 rad/s");
    // 0.05 rad in one 0.01 s step: over 1.25 * 3.0 * 0.01 = 0.0375 rad.
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[0].samples[2].q_rad[0] =
                 p->segments[0].samples[1].q_rad[0] + 0.05;
           }).reason == "q_step_exceeded",
           "q_step_exceeded fires at 0.05 rad per step");
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[0].samples[2].torch_on = true;
           }).reason == "torch_outside_weld",
           "torch_outside_weld fires in a freespace segment");
    // A 1 ms grid slip: 100x the 1e-9 s tolerance.
    expect(mutated([](dense_joint_trajectory::Plan* p) {
             p->segments[0].samples[2].time_s += 0.001;
           }).reason == "time_grid_invalid",
           "time_grid_invalid fires on a 1 ms slip");
  }

  if (failures == 0) {
    std::printf("dense_joint_trajectory_format_test: all assertions passed\n");
    return 0;
  }
  std::fprintf(stderr, "dense_joint_trajectory_format_test: %d failure(s)\n", failures);
  return 1;
}
