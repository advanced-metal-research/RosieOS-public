#pragma once

// Reject repeated producer snapshots without extending their receipt lifetime.
// Callers serialize access and decide whether invalid input revokes authority.
#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace robot_v4 {
enum class StatusSequenceResult { Accepted, Legacy, Ignored, Invalid };
class StatusSequence {
 public:
  StatusSequenceResult observe(const std::string& boot, std::uint64_t sequence) {
    if (boot.empty() && sequence == 0)
      return boot_.empty() ? StatusSequenceResult::Legacy : StatusSequenceResult::Invalid;
    if (boot.empty() || boot.size() > 96 || sequence == 0)
      return StatusSequenceResult::Invalid;
    if (boot == boot_) {
      if (sequence <= sequence_) return StatusSequenceResult::Ignored;
    } else {
      if (std::find(retired_.begin(), retired_.end(), boot) != retired_.end())
        return StatusSequenceResult::Ignored;
      if (retired_.size() >= 64) return StatusSequenceResult::Invalid;
      if (!boot_.empty()) retired_.push_back(boot_);
      boot_ = boot;
      ++generation_;
    }
    sequence_ = sequence;
    return StatusSequenceResult::Accepted;
  }
  std::uint64_t generation() const { return generation_; }
 private:
  std::string boot_;
  std::uint64_t sequence_ = 0;
  std::uint64_t generation_ = 0;
  std::vector<std::string> retired_;
};
}  // namespace robot_v4
