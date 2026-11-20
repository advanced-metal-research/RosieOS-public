#include "drive_alarm_reset.hpp"
#include <cassert>
#include <string>

using rosie::rtcore::classify_drive_alarm_reset;
using rosie::rtcore::DriveAlarmReset;
using rosie::rtcore::DriveAlarmResetConfig;
using rosie::rtcore::DriveAlarmResetEvidence;
using rosie::rtcore::DriveAlarmResetObservation;
using rosie::rtcore::DriveAlarmResetPulse;
using rosie::rtcore::parse_drive_alarm_reset_spec;
using rosie::rtcore::render_drive_alarm_reset_spec;

// CiA402 statuswords (masked patterns from ds402.hpp).
constexpr uint16_t kFault = 0x0218;          // Fault, voltage, remote
constexpr uint16_t kFaultReaction = 0x021f;  // Fault Reaction Active
constexpr uint16_t kSwitchOnDisabled = 0x0250;
constexpr uint16_t kOperationEnabled = 0x0237;

static void test_spec_round_trip_and_refusals() {
  DriveAlarmResetConfig config{};
  assert(parse_drive_alarm_reset_spec("200|1000|0x7121:0x0411;0x3230:0x0410", &config));
  assert(config.leave_fault_timeout_ns == 200'000'000 && config.min_interval_ns == 1'000'000'000);
  assert(config.count == 2 && config.codes[0].error_code == 0x7121 && config.codes[0].aux_code == 0x0411);
  assert(render_drive_alarm_reset_spec(config) == "200|1000|0x7121:0x0411;0x3230:0x0410");
  DriveAlarmResetConfig again{};
  assert(parse_drive_alarm_reset_spec(render_drive_alarm_reset_spec(config), &again));
  assert(render_drive_alarm_reset_spec(again) == render_drive_alarm_reset_spec(config));
  // Every malformed form refuses the whole line and leaves nothing classified.
  for (const char* bad : {"", "200|1000|", "0|1000|0x7121:0x0411", "501|1000|0x7121:0x0411",
                          "200|0|0x7121:0x0411", "200|3600001|0x7121:0x0411", "200|1000|0x0000:0x0411",
                          "200|1000|0x7121", "200|1000|0x7121:0x0411;0x7121:0x0411", "200|1000|0x7121:0x10000",
                          "200|1000|0x7121:0x0411;", "200|1000|0xg121:0x0411", "200|1000|0x7121:0x0411|1",
                          "-1|1000|0x7121:0x0411", "200|1000|7121:0x0411x"}) {
    DriveAlarmResetConfig refused{};
    refused.count = 9;
    assert(!parse_drive_alarm_reset_spec(bad, &refused));
    assert(!refused.configured() && refused.count == 0);
  }
  std::string many = "200|1000|";
  for (unsigned i = 1; i <= rosie::rtcore::kMaxResettableDriveAlarms + 1; ++i)
    many += (i == 1 ? "" : ";") + std::string("0x7121:") + std::to_string(i);
  assert(!parse_drive_alarm_reset_spec(many, &config));
}

static void test_exact_pair_only() {
  DriveAlarmResetConfig config{};
  assert(parse_drive_alarm_reset_spec("200|1000|0x7121:0x0411", &config));
  assert(config.resettable(0x7121, 0x0411, true));
  // The upper 16 bits of the 0x203F receipt are ignored, as in the decode table.
  assert(config.resettable(0x7121, 0xbeef0411, true));
  assert(!config.resettable(0x7121, 0x0411, false)); // Unread detail is unknown.
  assert(!config.resettable(0x7121, 0x0412, true));
  assert(!config.resettable(0x3230, 0x0411, true));
  assert(!config.resettable(0, 0x0411, true));
  assert(!DriveAlarmResetConfig{}.resettable(0x7121, 0x0411, true));
}

static void test_classification_order() {
  DriveAlarmResetConfig config{};
  assert(parse_drive_alarm_reset_spec("200|1000|0x7121:0x0411", &config));
  DriveAlarmResetObservation in{};
  in.statusword = kFault; in.error_code = 0x7121; in.aux_code = 0x0411; in.aux_valid = true;
  in.fresh = true; in.inhibited = true; in.now_ns = 10'000'000'000;
  assert(classify_drive_alarm_reset(config, in) == DriveAlarmReset::Possible);
  // A drive not in CiA402 Fault needs no reset, whatever 0x603F says.
  for (uint16_t sw : {kSwitchOnDisabled, kOperationEnabled, kFaultReaction}) {
    auto other = in; other.statusword = sw;
    assert(classify_drive_alarm_reset(config, other) == DriveAlarmReset::None);
  }
  assert(classify_drive_alarm_reset(DriveAlarmResetConfig{}, in) == DriveAlarmReset::NotConfigured);
  auto stale = in; stale.fresh = false;
  assert(classify_drive_alarm_reset(config, stale) == DriveAlarmReset::FeedbackStale);
  auto unread = in; unread.aux_valid = false;
  assert(classify_drive_alarm_reset(config, unread) == DriveAlarmReset::AlarmUnread);
  auto other_alarm = in; other_alarm.error_code = 0x7305; other_alarm.aux_code = 0x0201;
  assert(classify_drive_alarm_reset(config, other_alarm) == DriveAlarmReset::NotResettable);
  auto armed = in; armed.inhibited = false;
  assert(classify_drive_alarm_reset(config, armed) == DriveAlarmReset::NotInhibited);
  // Not resettable outranks armed: the status never suggests disarming to
  // reset an alarm that Reset would refuse anyway.
  auto armed_other = other_alarm; armed_other.inhibited = false;
  assert(classify_drive_alarm_reset(config, armed_other) == DriveAlarmReset::NotResettable);
  auto recent = in; recent.last_attempt_ns = in.now_ns - 999'999'999;
  assert(classify_drive_alarm_reset(config, recent) == DriveAlarmReset::RateLimited);
  assert(rosie::rtcore::drive_alarm_reset_retry_after_ns(config, recent.last_attempt_ns, in.now_ns) == 1);
  recent.last_attempt_ns = in.now_ns - 1'000'000'000;
  assert(classify_drive_alarm_reset(config, recent) == DriveAlarmReset::Possible);
  assert(rosie::rtcore::drive_alarm_reset_retry_after_ns(config, recent.last_attempt_ns, in.now_ns) == 0);
}

static void test_pulse_is_one_cia402_edge() {
  // Held while in Fault for at most the pulse length, then low: one edge.
  DriveAlarmResetPulse pulse{};
  pulse.begin(20);
  unsigned high = 0;
  for (int cycle = 0; cycle < 50; ++cycle) high += pulse.step(kFault) == 0x80;
  assert(high == 20 && !pulse.left_fault && !pulse.out_of_fault(kFault));
  // Fault Reaction Active ignores bit 7: nothing is raised until Fault, so
  // the first high cycle in Fault is the rising edge.
  pulse.begin(20);
  for (int cycle = 0; cycle < 5; ++cycle) assert(pulse.step(kFaultReaction) == 0);
  assert(pulse.cycles_left == 20 && pulse.step(kFault) == 0x80);
  // The bit falls the cycle the drive leaves Fault.
  assert(pulse.step(kFault) == 0x80 && pulse.step(kSwitchOnDisabled) == 0);
  assert(pulse.left_fault && !pulse.refaulted && pulse.out_of_fault(kSwitchOnDisabled));
  // Fault again after leaving: judged re-faulted, and no second edge.
  assert(pulse.step(kFault) == 0 && pulse.refaulted && !pulse.out_of_fault(kSwitchOnDisabled));
  for (int cycle = 0; cycle < 30; ++cycle) assert(pulse.step(kFault) == 0);
  pulse.begin(20);
  assert(pulse.step(kSwitchOnDisabled) == 0 && pulse.step(kFaultReaction) == 0 && pulse.refaulted);
}

static void test_operator_text() {
  DriveAlarmResetEvidence e{};
  e.error_code = 0x7121; e.aux_code = 0x0411; e.aux_valid = 1;
  e.result = DriveAlarmReset::PersistsAfterReset;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 4, "Er41.1", "Motor over-temperature due to locked- rotor") ==
         "J5 drive alarm Er41.1 persists after reset: Motor over-temperature due to locked- rotor");
  e.result = DriveAlarmReset::Refaulted;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 4, "Er41.1", "x") == "J5 drive alarm Er41.1 returned after reset: x");
  e.result = DriveAlarmReset::NotResettable;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 0, "Er20.1", "Encoder internal fault") ==
         "J1 drive alarm Er20.1 is not resettable from rt-core (drive profile): Encoder internal fault");
  e.result = DriveAlarmReset::RateLimited; e.retry_after_ns = 1'500'000'001;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 1, "Er41.1", "") == "J2 drive alarm Er41.1 reset refused: retry in 1501 ms");
  // A drive without a decode table is named by its codes.
  e.result = DriveAlarmReset::PersistsAfterReset;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 2, "", "") == "J3 drive fault 0x7121/0x0411 persists after reset");
  e.aux_valid = 0; e.result = DriveAlarmReset::AlarmUnread;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 2, "", "").rfind("J3 drive fault 0x7121 not reset", 0) == 0);
  e.result = DriveAlarmReset::None;
  assert(rosie::rtcore::drive_alarm_reset_message(e, 2, "", "").empty());
  assert(rosie::rtcore::drive_alarm_reset_failed(DriveAlarmReset::Interrupted));
  assert(!rosie::rtcore::drive_alarm_reset_failed(DriveAlarmReset::Cleared));
  assert(!rosie::rtcore::drive_alarm_reset_failed(DriveAlarmReset::None));
}

int main() {
  test_spec_round_trip_and_refusals();
  test_exact_pair_only();
  test_classification_order();
  test_pulse_is_one_cia402_edge();
  test_operator_text();
  return 0;
}
