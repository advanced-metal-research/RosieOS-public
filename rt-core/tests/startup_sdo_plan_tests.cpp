// Write-on-change contract for startup drive configuration, in absolute
// values: what the drive held decides what is queued, never the other way.
#include "startup_sdo_plan.hpp"
#include <array>
#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

namespace {
// Only the fields the planner reads; StartupSdoConfig carries the same names.
struct Descriptor {
  std::string key;
  uint32_t axis_mask = 1;
  std::array<int64_t, 16> values{};
  bool storage_mode = false, ram_only = false;
  int64_t storage_off = 0;
};

Descriptor at(const char* key, uint32_t axis, int64_t value) {
  Descriptor d;
  d.key = key;
  d.axis_mask = 1u << axis;
  d.values[axis] = value;
  return d;
}

// A6-EC shaped fixture on axis 1: C13.10 storage (store 1, RAM-only 0),
// tracking mode 4, feedforward 500, and C00.06 800 as a trial.
std::vector<Descriptor> machine() {
  std::vector<Descriptor> d{at("tracking_mode", 0, 4), at("storage", 1, 1), at("tracking_mode", 1, 4),
                            at("feedforward", 1, 500), at("inertia", 1, 800)};
  d[1].storage_mode = true;
  d[1].storage_off = 0;
  d[4].ram_only = true;
  return d;
}

std::vector<StartupSdoPrior> held(int64_t storage, int64_t tracking, int64_t feedforward, int64_t inertia) {
  return {{true, 9}, {true, storage}, {true, tracking}, {true, feedforward}, {true, inertia}};
}

using K = StartupSdoStepKind;
void expect_steps(const StartupSdoAxisPlan& plan, std::vector<std::array<int64_t, 3>> want) {
  assert(plan.steps.size() == want.size());
  for (size_t i = 0; i < want.size(); ++i) {
    assert(int64_t(plan.steps[i].descriptor) == want[i][0]);
    assert(plan.steps[i].value == want[i][1]);
    assert(int64_t(plan.steps[i].kind) == want[i][2]);
  }
}
}  // namespace

int main() {
  const auto d = machine();
  // Steady state: the drive holds every configured value. Nothing is written.
  {
    const auto plan = plan_axis_startup_sdos(d, 1, held(1, 4, 500, 800));
    assert(plan.steps.empty() && !plan.refused && !plan.storage_window);
    assert(plan.written == 0 && plan.skipped == 4 && plan.persisted == 0 && plan.ram_only == 0 && plan.unread == 0);
    for (size_t i = 1; i < d.size(); ++i) assert(plan.action[i] == StartupSdoAction::kSkippedIdentical);
    assert(plan.action[0] == StartupSdoAction::kNone);  // axis 0 is not this axis
  }
  std::puts("PASS: TestStartupIdenticalWritesNothing");
  // One stored value differs (feedforward 450 held, 500 configured): exactly that write.
  {
    const auto plan = plan_axis_startup_sdos(d, 1, held(1, 4, 450, 800));
    expect_steps(plan, {{3, 500, int64_t(K::kStored)}});
    assert(plan.written == 1 && plan.persisted == 1 && plan.skipped == 3 && plan.ram_only == 0);
    assert(plan.action[3] == StartupSdoAction::kStored);
  }
  std::puts("PASS: TestStartupDifferentWritesOnlyThatObject");
  // An unreadable object is written, as before write-on-change.
  {
    auto prior = held(1, 4, 500, 800);
    prior[2] = {};
    const auto plan = plan_axis_startup_sdos(d, 1, prior);
    expect_steps(plan, {{2, 4, int64_t(K::kStored)}});
    assert(plan.unread == 1 && plan.written == 1 && plan.skipped == 3);
  }
  std::puts("PASS: TestStartupUnreadIsWritten");
  // A trial that differs (1143 held, 800 configured) is written with storage
  // off: C13.10 = 0, C00.06 = 800, C13.10 = 1. Nothing is stored.
  {
    const auto plan = plan_axis_startup_sdos(d, 1, held(1, 4, 500, 1143));
    expect_steps(plan, {{1, 0, int64_t(K::kStorageOff)}, {4, 800, int64_t(K::kRamOnly)}, {1, 1, int64_t(K::kStorageOn)}});
    assert(plan.storage_window && plan.written == 1 && plan.ram_only == 1 && plan.persisted == 0 && plan.skipped == 3);
    assert(plan.action[4] == StartupSdoAction::kRamOnly && plan.action[1] == StartupSdoAction::kSkippedIdentical);
  }
  std::puts("PASS: TestStartupTrialWrittenInsideRamOnlyWindow");
  // A trial the drive already holds in RAM (rt-core restarted, drive not
  // power-cycled): no window, no storage-mode writes.
  {
    const auto plan = plan_axis_startup_sdos(d, 1, held(1, 4, 500, 800));
    assert(plan.steps.empty() && !plan.storage_window);
  }
  std::puts("PASS: TestStartupHeldTrialOpensNoWindow");
  // Storage found off (C13.10 = 0): store mode is restored first, so the
  // stored write after it is stored, and the trial window follows.
  {
    const auto plan = plan_axis_startup_sdos(d, 1, held(0, 4, 450, 1143));
    expect_steps(plan, {{1, 1, int64_t(K::kStored)}, {3, 500, int64_t(K::kStored)}, {1, 0, int64_t(K::kStorageOff)},
                        {4, 800, int64_t(K::kRamOnly)}, {1, 1, int64_t(K::kStorageOn)}});
    assert(plan.written == 3 && plan.persisted == 2 && plan.ram_only == 1 && plan.skipped == 1);
  }
  std::puts("PASS: TestStartupStorageRestoredBeforeStoredWrites");
  // Configuration order is kept among stored writes; trials follow them.
  {
    auto e = d;
    e[3].ram_only = true;  // feedforward also a trial
    e[4].ram_only = false;
    const auto plan = plan_axis_startup_sdos(e, 1, held(1, 3, 450, 1143));
    expect_steps(plan, {{2, 4, int64_t(K::kStored)}, {4, 800, int64_t(K::kStored)}, {1, 0, int64_t(K::kStorageOff)},
                        {3, 500, int64_t(K::kRamOnly)}, {1, 1, int64_t(K::kStorageOn)}});
  }
  std::puts("PASS: TestStartupStoredWritesKeepOrderTrialsLast");
  // A differing trial with no storage object is refused, never stored.
  {
    auto e = d;
    e[1].storage_mode = false;
    const auto plan = plan_axis_startup_sdos(e, 1, held(1, 4, 500, 1143));
    assert(plan.refused);
    for (const auto& step : plan.steps) assert(step.kind != K::kRamOnly && step.descriptor != 4);
  }
  std::puts("PASS: TestStartupTrialWithoutStorageRefused");
  // Without any storage policy (the 1430 cell today) the plan is plain write-on-change.
  {
    std::vector<Descriptor> plain{at("tracking_mode", 0, 4), at("feedforward", 0, 500)};
    const auto skip = plan_axis_startup_sdos(plain, 0, {{true, 4}, {true, 500}});
    assert(skip.steps.empty() && skip.skipped == 2);
    const auto write = plan_axis_startup_sdos(plain, 0, {{true, 4}, {true, 0}});
    expect_steps(write, {{1, 500, int64_t(K::kStored)}});
  }
  std::puts("PASS: TestStartupPlainWriteOnChange");
  return 0;
}
