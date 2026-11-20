#pragma once
#include <cstddef>
#include <cstdint>
#include <vector>

// Write-on-change for startup drive configuration. A drive that stores SDO
// writes (A6-EC with C13.10 = 1 writes EEPROM, rated about 1e5..1e6 cycles,
// ALF1.3 "too frequent writing of EEPROM") must not be rewritten with the
// value it already holds at every start: the prior value is read in PREOP and
// an identical object is not queued, so a steady-state start writes nothing.
// What is verified does not change: every descriptor is still read back after
// startup and gates config_verified_mask whether it was written or skipped.

// The object as read over the mailbox before scheduling. An unread object is
// written, exactly as before write-on-change: a failed read is never a match.
struct StartupSdoPrior {
  bool read = false;
  int64_t value = 0;
};

// One queued startup download, in the order the master applies them.
// StorageOff/StorageOn bracket the RAM-only trial writes; they write the
// storage_mode descriptor's object with its storage_off and store values.
enum class StartupSdoStepKind : uint8_t { kStored, kRamOnly, kStorageOff, kStorageOn };

struct StartupSdoStep {
  size_t descriptor = 0;
  int64_t value = 0;
  StartupSdoStepKind kind = StartupSdoStepKind::kStored;
};

enum class StartupSdoAction : uint8_t { kNone, kSkippedIdentical, kStored, kRamOnly };

// written = descriptors queued (persisted + ram_only), skipped = identical,
// unread = written because the prior read failed. A RAM-only trial without a
// storage_mode descriptor on the axis is refused, never written stored.
struct StartupSdoAxisPlan {
  std::vector<StartupSdoStep> steps;
  std::vector<StartupSdoAction> action;  // per descriptor; kNone off this axis
  uint32_t written = 0, skipped = 0, persisted = 0, ram_only = 0, unread = 0;
  bool storage_window = false, refused = false;
};

// Order: the storage-mode object first (so every stored write after it is
// stored), then stored descriptors in configuration order, then, only when a
// trial value differs, storage off, the trial writes, storage on.
template <class Descriptors>
StartupSdoAxisPlan plan_axis_startup_sdos(const Descriptors& startup, uint32_t axis,
                                          const std::vector<StartupSdoPrior>& prior) {
  StartupSdoAxisPlan plan;
  plan.action.assign(startup.size(), StartupSdoAction::kNone);
  const uint32_t bit = 1u << axis;
  constexpr size_t kNoStorage = ~size_t{0};
  size_t storage = kNoStorage;
  for (size_t d = 0; d < startup.size(); ++d)
    if ((startup[d].axis_mask & bit) && startup[d].storage_mode) storage = d;
  auto differs = [&](size_t d) {
    if (d >= prior.size() || !prior[d].read) {
      ++plan.unread;
      return true;
    }
    if (prior[d].value == startup[d].values[axis]) {
      plan.action[d] = StartupSdoAction::kSkippedIdentical;
      ++plan.skipped;
      return false;
    }
    return true;
  };
  auto store = [&](size_t d) {
    plan.steps.push_back({d, startup[d].values[axis], StartupSdoStepKind::kStored});
    plan.action[d] = StartupSdoAction::kStored;
    ++plan.written;
    ++plan.persisted;
  };
  if (storage != kNoStorage && differs(storage)) store(storage);
  for (size_t d = 0; d < startup.size(); ++d)
    if ((startup[d].axis_mask & bit) && !startup[d].storage_mode && !startup[d].ram_only && differs(d)) store(d);
  std::vector<size_t> trials;
  for (size_t d = 0; d < startup.size(); ++d)
    if ((startup[d].axis_mask & bit) && startup[d].ram_only && !startup[d].storage_mode && differs(d)) trials.push_back(d);
  if (trials.empty()) return plan;
  if (storage == kNoStorage) {
    plan.refused = true;
    return plan;
  }
  plan.storage_window = true;
  plan.steps.push_back({storage, startup[storage].storage_off, StartupSdoStepKind::kStorageOff});
  for (size_t d : trials) {
    plan.steps.push_back({d, startup[d].values[axis], StartupSdoStepKind::kRamOnly});
    plan.action[d] = StartupSdoAction::kRamOnly;
    ++plan.written;
    ++plan.ram_only;
  }
  plan.steps.push_back({storage, startup[storage].values[axis], StartupSdoStepKind::kStorageOn});
  return plan;
}
