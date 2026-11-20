#define ROSIE_RT_SIMULATION 1
#include "engine/cycle_machine.hpp"

#include <cassert>

namespace {
enum class MappingTestFault { None, Refused, WideLate, LateSequence };
struct MappingTestUpload {
  uint16_t position, index;
  uint8_t subindex;
};
MappingTestFault mapping_test_fault = MappingTestFault::None;
// Measured bench topology in the supplied upload: nine slave positions, 0..8.
std::array<bool, 9> mapping_test_injected{};
std::vector<MappingTestUpload> mapping_test_uploads;
std::vector<SdoReadIdentity> mapping_test_identities;
std::optional<SdoReadReply> mapping_test_late;
unsigned mapping_test_identity_field = 0;
bool mapping_test_change_epoch = false;
MachineState* mapping_test_machine = nullptr;
void (*mapping_test_after_upload)() = nullptr;

// Unverified fixture: an OP bus continuously ready since 500000000 ns before
// the injected observation. Tests of other policies start after admission.
void settle_mailbox(MachineState& machine, ec_master_t& master, uint64_t now) {
  master.active = true;
  machine.bus_ready.store(true);
  machine.latest_feedback.startup_ready = 1;
  machine.latest_feedback.domain_wc_state = EC_WC_COMPLETE;
  machine.update_mailbox_readiness(true, now - 500000000);
  machine.master_diag_state = {machine.opt.num_axes, 8, 1};
  for (uint32_t i = 0; i < machine.opt.num_axes; ++i) machine.sc_state[i] = {1, 1, 8};
}

// Inject the bench's refused 0x1702:03 and a hypothesized cached 32-bit payload
// at later axes' count reads. This models an untrusted upload boundary, not an
// assertion that IgH's per-slave CoE state machine can shift between slaves.
int mapping_test_upload(ec_master_t* master, uint16_t position, uint16_t index,
                        uint8_t subindex, uint8_t* data, size_t capacity,
                        size_t* size, uint32_t* abort) {
  mapping_test_uploads.push_back({position, index, subindex});
  if (mapping_test_fault != MappingTestFault::None && position == 0 &&
      index == 0x1702 && subindex == 3 && !mapping_test_injected[0]) {
    mapping_test_injected[0] = true;
    *size = 0;
    *abort = 0;
    return -EIO;
  }
  if (mapping_test_fault == MappingTestFault::WideLate && position > 0 &&
      index == 0x1c12 && subindex == 0 && !mapping_test_injected[position]) {
    mapping_test_injected[position] = true;
    assert(capacity >= 4);
    EC_WRITE_U32(data, 0x60ff0020);
    *size = 4;
    *abort = 0;
    return 0;
  }
  return ecrt_master_sdo_upload(master, position, index, subindex, data, capacity, size, abort);
}

// Return the old receipt with its old identity, including when the same object
// is retried. A caller must reject it even when its size and payload look valid.
SdoReadReply mapping_test_read(ec_master_t* master, const SdoReadIdentity& identity) {
  mapping_test_identities.push_back(identity);
  SdoReadReply reply;
  reply.identity = identity;
  reply.error = mapping_test_upload(master, identity.position, identity.index, identity.subindex,
      reply.bytes.data(), reply.bytes.size(), &reply.size, &reply.abort);
  if (mapping_test_fault == MappingTestFault::LateSequence && identity.axis == 0 &&
      identity.index == 0x1702 && identity.subindex == 3) {
    if (reply.error != 0) {
      auto late = upload_sdo_read(master, identity);
      assert(late.error == 0 && late.size == 4);
      mapping_test_late = late;
    } else if (mapping_test_late) {
      reply = *mapping_test_late;
      mapping_test_late.reset();
    }
  }
  switch (mapping_test_identity_field) {
    case 1: ++reply.identity.axis; break;
    case 2: ++reply.identity.position; break;
    case 3: ++reply.identity.index; break;
    case 4: ++reply.identity.subindex; break;
    case 5: --reply.identity.sequence; break;
    default: break;
  }
  if (mapping_test_change_epoch && identity.index == 0x1b04 && identity.subindex == 10)
    mapping_test_machine->configuration_epoch.fetch_add(1);
  if (mapping_test_after_upload) mapping_test_after_upload();
  return reply;
}

// The 2026-09-14 bench upload supplies these exact seven Rx and ten Tx entries.
// Each axis owns its simulated slave dictionary; one refusal cannot mutate it.
struct MappingTestFixture {
  Options options;
  OptionsParseResult configuration;
  ec_master_t master;
  std::unique_ptr<MachineState> storage;
  MappingTestFixture() {
    options.num_axes = 9;
    ec_pdo_entry_info_t rx[] = {{0x6040, 0, 16}, {0x607a, 0, 32}, {0x60ff, 0, 32},
        {0x6071, 0, 16}, {0x6060, 0, 8}, {0x60b8, 0, 16}, {0x607f, 0, 32}};
    ec_pdo_entry_info_t tx[] = {{0x603f, 0, 16}, {0x6041, 0, 16}, {0x6064, 0, 32},
        {0x6077, 0, 16}, {0x6061, 0, 8}, {0x60f4, 0, 32}, {0x60b9, 0, 16},
        {0x60ba, 0, 32}, {0x60bc, 0, 32}, {0x606c, 0, 32}};
    ec_pdo_info_t pdos[] = {{0x1702, 7, rx}, {0x1b04, 10, tx}};
    ec_sync_info_t syncs[] = {{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
        {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE},
        {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
    for (unsigned axis = 0; axis < 9; ++axis) {
      options.slave_position[axis] = axis;
      auto& pdo = configuration.axis_profiles[axis].pdo;
      pdo.rx_pdo = 0x1702; pdo.tx_pdo = 0x1b04;
      pdo.rx_sync_index = 2; pdo.tx_sync_index = 3;
      assert(parse_pdo_layout_spec("cw|0x6040|0|16;target_pos|0x607a|0|32;target_vel|0x60ff|0|32;target_torque|0x6071|0|16;mode|0x6060|0|8;tp_func|0x60b8|0|16;max_profile_vel|0x607f|0|32", &pdo.rx_entries));
      assert(parse_pdo_layout_spec("err|0x603f|0|16;sw|0x6041|0|16;pos|0x6064|0|32;torque|0x6077|0|16;mode_disp|0x6061|0|8;following_error|0x60f4|0|32;tp_status|0x60b9|0|16;tp_pos1|0x60ba|0|32;tp_pos2|0x60bc|0|32;velocity_actual|0x606c|0|32", &pdo.tx_entries));
      auto* slave = ecrt_master_slave_config(&master, 0, axis, 4194304, 1813);
      assert(ecrt_slave_config_pdos(slave, EC_END, syncs) == 0);
    }
    storage = std::make_unique<MachineState>(options, configuration);
    storage->shared_master.store(&master);
    settle_mailbox(*storage, master, now_monotonic_ns());
    mapping_test_fault = MappingTestFault::None;
    mapping_test_injected = {};
    mapping_test_uploads.clear();
    mapping_test_identities.clear();
    mapping_test_late.reset();
    mapping_test_identity_field = 0;
    mapping_test_change_epoch = false;
    mapping_test_after_upload = nullptr;
    mapping_test_machine = storage.get();
  }
  ~MappingTestFixture() {
    mapping_test_fault = MappingTestFault::None;
    mapping_test_change_epoch = false;
    mapping_test_machine = nullptr;
  }
};

// Positive assertions use complete upload chains and exact attempt counts;
// refused reads never latch a mismatch on this or any other axis.
void mapping_test_retry(MappingTestFault fault) {
  MappingTestFixture fixture;
  mapping_test_fault = fault;
  std::array<MappingVerification, 9> progress{};
  for (unsigned axis = 0; axis < 9; ++axis) {
    progress[axis].observe(fixture.storage->verify_axis_mapping(axis));
    assert(!progress[axis].faulted);
    assert(progress[axis].attempts == 1);
    assert(progress[axis].verified == (fault == MappingTestFault::Refused && axis > 0));
  }
  for (unsigned axis = 0; axis < 9; ++axis) {
    if (progress[axis].pending()) progress[axis].observe(fixture.storage->verify_axis_mapping(axis));
    assert(progress[axis].verified && !progress[axis].faulted);
    assert(progress[axis].attempts == (fault == MappingTestFault::Refused && axis > 0 ? 1u : 2u));
  }
  assert(mapping_test_uploads.size() == (fault == MappingTestFault::Refused ? 230 : 414));
  for (size_t i = 1; i < mapping_test_identities.size(); ++i)
    assert(mapping_test_identities[i].sequence > mapping_test_identities[i - 1].sequence);
}

// A difference followed by an unavailable element is an incomplete proof,
// even if the first element would previously have caused an immediate latch.
void mapping_test_incomplete_mismatch() {
  MappingTestFixture fixture;
  auto& sdos = fixture.master.slaves[0]->sdos;
  sdos[0x1c1200][0] = 2;
  sdos.erase(0x1b040a);
  assert(fixture.storage->verify_axis_mapping(0) == MappingReadback::Unavailable);
  assert(mapping_test_uploads.size() == 23);
}

// The old sequence for the same axis/object is unavailable, not a new proof.
// Unrelated axes finish once; axis zero succeeds on the third bounded attempt.
void mapping_test_late_sequence() {
  MappingTestFixture fixture;
  mapping_test_fault = MappingTestFault::LateSequence;
  std::array<MappingVerification, 9> progress{};
  for (unsigned pass = 1; pass <= 3; ++pass) {
    for (unsigned axis = 0; axis < 9; ++axis) {
      if (progress[axis].pending()) progress[axis].observe(fixture.storage->verify_axis_mapping(axis));
      assert(!progress[axis].faulted);
      assert(progress[axis].attempts == (axis == 0 ? pass : 1));
      assert(progress[axis].verified == (axis > 0 || pass == 3));
    }
  }
  assert(mapping_test_uploads.size() == 253);
}

// Exact width and all identity fields are mandatory for every scalar type.
// Failed reads preserve the caller's sentinel instead of exposing stale bytes.
void mapping_test_scalar_identity_and_width() {
  MappingTestFixture fixture;
  auto& machine = *fixture.storage;
  int32_t value = 12345;
  uint32_t abort = 123;
  for (unsigned field = 1; field <= 5; ++field) {
    mapping_test_identity_field = field;
    assert(!machine.read_sdo_scalar_axis(0, 0x1702, 3, SdoScalarType::kU32, &value, &abort));
    assert(value == 12345 && abort == 0);
  }
  mapping_test_identity_field = 0;
  for (auto type : {SdoScalarType::kU8, SdoScalarType::kI8, SdoScalarType::kU16,
                    SdoScalarType::kI16, SdoScalarType::kU32, SdoScalarType::kI32}) {
    const size_t width = type == SdoScalarType::kU8 || type == SdoScalarType::kI8 ? 1 :
        type == SdoScalarType::kU16 || type == SdoScalarType::kI16 ? 2 : 4;
    for (size_t size = 0; size <= 4; ++size) {
      fixture.master.slaves[0]->sdos[0x200000] = std::vector<uint8_t>(size, 1);
      value = 12345;
      const bool ok = machine.read_sdo_scalar_axis(0, 0x2000, 0, type, &value, &abort);
      assert(ok == (size == width));
      if (!ok) assert(value == 12345);
    }
  }
}

// A missing mapping remains inhibited after exactly three complete attempts;
// terminal results cannot be overwritten by an unsolicited fourth observation.
void mapping_test_retry_bound() {
  MappingTestFixture fixture;
  fixture.master.slaves[0]->sdos.erase(0x170203);
  MappingVerification progress;
  for (unsigned attempt = 1; attempt <= 3; ++attempt) {
    progress.observe(fixture.storage->verify_axis_mapping(0));
    assert(progress.attempts == attempt && !progress.verified);
    assert(progress.faulted == (attempt == 3));
  }
  assert(mapping_test_uploads.size() == 69);
  progress.observe(MappingReadback::Matched);
  assert(progress.attempts == 3 && progress.faulted && !progress.verified);
}

// A genuine changed descriptor names the first difference only after all 23
// responses arrive; publishing that proof latches the native mapping fault.
void mapping_test_named_mismatch() {
  MappingTestFixture fixture;
  auto& machine = *fixture.storage;
  fixture.master.slaves[0]->sdos[0x1b040a][0] = 16;
  FILE* log = std::fopen("build/mapping-mismatch.log", "w+");
  assert(log);
  const int saved = dup(STDERR_FILENO);
  assert(saved >= 0 && dup2(fileno(log), STDERR_FILENO) >= 0);
  MappingVerification progress;
  progress.observe(machine.verify_axis_mapping(0));
  std::fflush(stderr);
  assert(dup2(saved, STDERR_FILENO) >= 0);
  close(saved);
  std::rewind(log);
  // Unverified capture bound: 4096 bytes for this single diagnostic.
  std::array<char, 4096> bytes{};
  const auto size = std::fread(bytes.data(), 1, bytes.size(), log);
  std::fclose(log);
  const std::string output(bytes.data(), size);
  assert(output.find("result=pdo_mapping_mismatch reason=7 index=0x1b04 sub=0x0a expected=0x606c0020 observed=0x606c0010") != std::string::npos);
  assert(progress.faulted && !progress.verified && progress.attempts == 1);
  assert(mapping_test_uploads.size() == 23);
  MachineState::cycle_owner = &machine;
  machine.configuration_readbacks.publish({1, 0, 1, machine.verification_request.load()});
  machine.consume_control_requests();
  assert(machine.control_policy.config_verified_mask.load() == 0);
  assert(machine.execution_fault_reasons.load() == rosie::rtcore::FaultPdoMappingMismatch);
  assert(machine.safety_fault_mask.load() == 1);
  MachineState::cycle_owner = nullptr;

  // An epoch change on the final response invalidates even a complete mismatch.
  mapping_test_change_epoch = true;
  assert(machine.verify_axis_mapping(0) == MappingReadback::Unavailable);
}

// Unverified deterministic observations: no wall clock waiting establishes
// success; worker termination is bounded by 100 admission observations.
uint64_t mailbox_test_now = 1000000000;
unsigned mailbox_test_observations = 0;
uint64_t mailbox_test_clock() { return mailbox_test_now; }
uint64_t mailbox_worker_clock() {
  mailbox_test_now += 600000000;
  if (++mailbox_test_observations == 100) mapping_test_machine->commissioning_running.store(false);
  mapping_test_machine->readbacks_requested.store(true);
  return mailbox_test_now;
}

// Reproduce the reviewer's activated PREOP mapping/startup/diagnostic probes
// and freshly-ready rotation probe. Every path must issue zero uploads.
void runtime_mailbox_gates() {
  for (unsigned mode = 0; mode < 4; ++mode) {
    MappingTestFixture f;
    auto& m = *f.storage;
    g_stop.store(false);
    mailbox_test_now = 1000000000; mailbox_test_observations = 0;
    m.mailbox_ready_since_ns.store(500000000);
    m.mailbox_clock = mailbox_worker_clock;
    m.diagnostic_handles_ready.store(true);
    m.commissioning_running.store(true);
    if (mode == 0) { m.bus_ready.store(false); m.latest_feedback.startup_ready = 0; }
    if (mode < 3) f.master.slaves[8]->al_state_override.store(2);
    if (mode == 1) {
      StartupSdoConfig descriptor{};
      descriptor.valid = true; descriptor.axis_mask = 1; descriptor.key = "mailbox_gate";
      descriptor.type = StartupSdoValueType::kU16; descriptor.index = 0x2010; descriptor.subindex = 0x1a;
      descriptor.values[0] = 1; m.startup_sdos.push_back(descriptor);
      f.master.slaves[0]->sdos[0x20101a] = {1, 0};
    }
    if (mode == 2) {
      assert(m.metrics_absolute_feedback_poll_enabled);
      auto& absolute = m.axis_profiles[0].absolute;
      absolute.valid = true; absolute.field_count = 1;
      absolute.fields[0] = {true, "gate", {true, 0x2040, 0x21, SdoScalarType::kI32}};
      f.master.slaves[0]->sdos[0x204021] = {1, 0, 0, 0};
    }
    if (mode == 3) {
      m.mailbox_clock = mailbox_test_clock;
      m.mailbox_ready_since_ns.store(1000000000);
      assert(!m.sample_absolute_axis(0, mailbox_test_clock));
    } else m.run_commissioning_worker();
    assert(mapping_test_uploads.size() == 0);
  }
}

// Requests start at exactly 500000000 ns of continuous readiness. A drop or
// epoch change during a request discards its bytes and closes admission again.
void runtime_mailbox_boundary_and_inflight() {
  MappingTestFixture f;
  auto& m = *f.storage;
  g_stop.store(false);
  m.mailbox_clock = mailbox_test_clock;
  mailbox_test_now = 1499999999;
  m.mailbox_ready_since_ns.store(1000000000);
  int32_t value = 12345; uint32_t abort = 0;
  assert(!m.read_sdo_scalar_axis(0, 0x1702, 3, SdoScalarType::kU32, &value, &abort));
  assert(!m.write_sdo_scalar_axis(0, 0x607c, 0, SdoScalarType::kI32, 1, &abort));
  assert(mapping_test_uploads.size() == 0 && value == 12345);
  mailbox_test_now = 1500000000;
  assert(m.read_sdo_scalar_axis(0, 0x1702, 3, SdoScalarType::kU32, &value, &abort));
  assert(mapping_test_uploads.size() == 1);
  for (bool epoch : {false, true}) {
    mapping_test_after_upload = epoch ? +[] { mapping_test_machine->configuration_epoch.fetch_add(1); } : +[] {
      mapping_test_machine->update_mailbox_readiness(false, 1500000000);
      mapping_test_machine->update_mailbox_readiness(true, 1500000001);
    };
    value = 12345;
    assert(!m.read_sdo_scalar_axis(0, 0x1702, 3, SdoScalarType::kU32, &value, &abort));
    assert(value == 12345);
    mapping_test_after_upload = nullptr;
    m.update_mailbox_readiness(true, 1500000001);
    mailbox_test_now = 2000000000;
    assert(!m.mailbox_admitted(0, m.configuration_epoch.load(), m.mailbox_ready_since_ns.load()));
    mailbox_test_now = 2000000001;
    assert(m.mailbox_admitted(0, m.configuration_epoch.load(), m.mailbox_ready_since_ns.load()));
  }
  m.latest_feedback.domain_wc_state = EC_WC_INCOMPLETE;
  assert(!m.read_sdo_scalar_axis(0, 0x1702, 3, SdoScalarType::kU32, &value, &abort));
}

void runtime_startup_requires_op();
// The battery-derived mailbox source must not mix axes or consult rotation
// registers in mode 3. Refused, stale and out-of-range reads supply no anchor.
void battery_anchor_source() {
  using rosie::rtcore::SampledAbsolutePosition;
  using rosie::rtcore::SampleInvalidity;
  MappingTestFixture f;
  auto& m = *f.storage;
  g_stop.store(false);
  // Unverified deterministic mailbox observation; no scheduler timing is used.
  mailbox_test_now = 1500000000;
  m.mailbox_clock = mailbox_test_clock;
  m.update_mailbox_readiness(false, mailbox_test_now);
  settle_mailbox(m, f.master, mailbox_test_now);
  m.rt_cycle_counter.store(42);
  m.verified_configuration_epoch.store(1);
  m.control_policy.config_verified_mask.store(3);
  for (unsigned axis : {0u, 1u}) {
    auto& p = f.configuration.axis_profiles[axis];
    p.encoder_battery_absent = axis == 0;
    p.anchor_source = {0x2040, 0x21, 0x23, "a6ec_raw_multi_turn", "raw_multi_turn_encoder", 3};
    m.sampled_consumption[axis].publish({});
    f.options.axis[axis].counts_per_unit = 131072 / 6.28318530717958647692;
    const auto source = p.absolute_anchor_source();
    assert(source.index == 0x2040);
    assert(source.low_subindex == 0x21);
    assert(source.high_subindex == 0x23);
    assert(source.name == "a6ec_raw_multi_turn");
    const auto identity_before = p.coordinate_identity;
    auto& dictionary = f.master.slaves[axis]->sdos;
    for (const auto entry : {std::make_pair(0x20401du, int32_t{131071}),
                            std::make_pair(0x20402bu, int32_t{65536}),
                            std::make_pair(0x20402du, int32_t{0})}) {
      dictionary[entry.first].resize(4);
      EC_WRITE_S32(dictionary[entry.first].data(), entry.second);
    }
    mapping_test_uploads.clear();
    assert(m.sample_absolute_axis(axis, mailbox_test_clock));
    SampledAbsolutePosition sample{};
    assert(m.sampled_absolute[axis].read(sample) && sample.valid);
    assert(sample.cycle == 42 && sample.configuration_epoch == 1 && sample.home_epoch == 0);
    assert(sample.encoder_counts == (axis == 0 ? 131071 : 65536));
    assert(sample.started_ns == 1500000000 && sample.completed_ns == 1500000000);
    assert(mapping_test_uploads.size() == (axis == 0 ? 1u : 3u));
    for (size_t n = 0; n < mapping_test_uploads.size(); ++n) {
      const auto& upload = mapping_test_uploads[n];
      assert(upload.position == axis && upload.index == 0x2040);
      assert(upload.subindex == (axis == 0 ? 0x1d : n == 1 ? 0x2b : 0x2d));
    }
    assert(p.coordinate_identity == identity_before);
  }
  auto& data = f.master.slaves[0]->sdos[0x20401d];
  for (int32_t count : {-1, 0, 131071, 131072}) {
    EC_WRITE_S32(data.data(), count);
    const bool valid = count == 0 || count == 131071;
    assert(m.sample_absolute_axis(0, mailbox_test_clock) == valid);
    SampledAbsolutePosition sample{};
    assert(m.sampled_absolute[0].read(sample));
    assert(sample.reason == uint32_t(valid ? SampleInvalidity::None : SampleInvalidity::Range));
    assert(sample.encoder_counts == count);
  }
  EC_WRITE_S32(data.data(), 65536);
  f.options.axis[0].counts_per_unit = 262144 / 6.28318530717958647692;
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  f.options.axis[0].counts_per_unit = 131072 / 6.28318530717958647692;
  data.resize(2);
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  data.resize(4);
  mapping_test_uploads.clear();
  m.control_policy.config_verified_mask.store(2);
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  assert(mapping_test_uploads.empty());
  m.control_policy.config_verified_mask.store(3);
  mapping_test_after_upload = +[] { mapping_test_machine->absolute_frame_epoch[0].fetch_add(1); };
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  SampledAbsolutePosition sample{};
  assert(m.sampled_absolute[0].read(sample) && sample.reason == uint32_t(SampleInvalidity::Epoch));
  mapping_test_after_upload = +[] { mapping_test_machine->configuration_epoch.fetch_add(1); };
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  assert(m.sampled_absolute[0].read(sample) && sample.reason == uint32_t(SampleInvalidity::Unavailable));
  mapping_test_after_upload = nullptr;
}

// A diagnostic edge waits for the exact runtime admission boundary, including
// every slave and complete exchanges. Both declared and observed reply widths
// preserve raw bytes, and no successful or failed edge can create retries.
void drive_alarm_admission_edges_and_widths() {
  MappingTestFixture f;
  auto& m = *f.storage;
  auto& alarm = m.drive_alarms[0];
  g_stop.store(false);
  m.mailbox_clock = mailbox_test_clock;
  mailbox_test_now = 1499999999;
  m.mailbox_ready_since_ns.store(1000000000);
  f.master.slaves[0]->sdos[0x203f00] = {0x10, 0x0f};
  alarm.observe(0x6320);
  assert(m.alarm_readback_pending());
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.empty() && m.alarm_readback_pending());
  mailbox_test_now = 1500000000;
  m.latest_feedback.domain_wc_state = EC_WC_INCOMPLETE;
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.empty());
  m.latest_feedback.domain_wc_state = EC_WC_COMPLETE;
  f.master.slaves[8]->al_state_override.store(2);
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.empty());
  f.master.slaves[8]->al_state_override.store(0);
  m.update_mailbox_readiness(true, 1500000000);
  mailbox_test_now = 2000000000;
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.size() == 1 && !m.alarm_readback_pending());
  assert(mapping_test_uploads[0].index == 0x203f && mapping_test_uploads[0].subindex == 0);
  auto result = alarm.snapshot(alarm.event.load(), 1, 0x6320);
  assert(result.alarm_aux_valid && result.alarm_aux_code == 0x0f10);
  for (unsigned i = 0; i < 100; ++i) { alarm.observe(0x6320); m.refresh_drive_alarm(); }
  assert(mapping_test_uploads.size() == 1);
  alarm.observe(0);
  result = alarm.snapshot(alarm.event.load(), 1, 0);
  assert(!result.alarm_aux_valid && result.alarm_aux_code == 0);
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.size() == 1);
  alarm.observe(0x6320);
  f.master.slaves[0]->sdos[0x203f00] = {0x12, 0x0f, 0xcd, 0xab};
  m.refresh_drive_alarm();
  result = alarm.snapshot(alarm.event.load(), 1, 0x6320);
  assert(mapping_test_uploads.size() == 2 && result.alarm_aux_valid && result.alarm_aux_code == 0xabcd0f12);
  alarm.observe(0x5530);
  assert(!alarm.snapshot(alarm.event.load(), 1, 0x5530).alarm_aux_valid);
  f.master.slaves[0]->sdos[0x203f00] = {0x11, 0x0f};
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.size() == 3);
  assert(alarm.snapshot(alarm.event.load(), 1, 0x5530).alarm_aux_code == 0x0f11);
}

// Drive-profile monitor objects (docs/decisions/0001): one due read per
// checkpoint, round-robin over the axes, each again only after its interval;
// none before mailbox admission; a failed read waits its interval too and
// publishes nothing.
void drive_monitor_reads_due_objects() {
  MappingTestFixture f;
  auto& m = *f.storage;
  g_stop.store(false);
  m.mailbox_clock = mailbox_test_clock;
  m.bus_ready.store(true);
  std::vector<MonitorConfig> monitors;
  assert(parse_monitor_spec("heatsink_temperature|8256|49|i16|1|degC|5000", &monitors));
  m.axis_profiles[0].monitor = monitors;
  m.axis_profiles[1].monitor = monitors;
  f.master.slaves[0]->sdos[0x204031] = {0x59, 0x01};  // 345: 34.5 degC
  f.master.slaves[1]->sdos[0x204031] = {0xf6, 0xff};  // -10: -1.0 degC
  mailbox_test_now = 1499999999;
  m.mailbox_ready_since_ns.store(1000000000);
  m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.empty());  // not admitted yet
  m.update_mailbox_readiness(true, 1500000000);
  mailbox_test_now = 2000000000;
  int32_t raw = 0;
  uint64_t at = 0;
  m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 1 && mapping_test_uploads[0].index == 0x2040 && mapping_test_uploads[0].subindex == 0x31);
  assert(m.drive_monitors[0].slots[0].snapshot(&raw, &at) && raw == 345 && at == 2000000000);
  assert(!m.drive_monitors[1].slots[0].snapshot(&raw, &at));
  m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 2 && m.drive_monitors[1].slots[0].snapshot(&raw, &at) && raw == -10);
  for (int i = 0; i < 20; ++i) m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 2);  // neither is due again
  mailbox_test_now = 7000000000;
  f.master.slaves[0]->sdos.erase(0x204031);  // the next read of axis 0 fails
  m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 3);
  assert(m.drive_monitors[0].slots[0].snapshot(&raw, &at) && raw == 345 && at == 2000000000);  // kept, ageing
  m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 4);  // axis 1 is due; axis 0 waits its interval
  for (int i = 0; i < 20; ++i) m.refresh_monitor(mailbox_test_clock);
  assert(mapping_test_uploads.size() == 4);
  // The loader refuses what the compiler would refuse.
  for (const char* bad : {"", "k|8256|49|i16|1|degC|999", "k|8256|49|i16|7|degC|5000", "k|8256|49|f32|1|degC|5000",
                          "k|8256|49|i16|1|deg C|5000", "k|8256|49|i16|1|degC|5000;k|8256|50|i16|1|degC|5000",
                          "k|8256|49|i16|1|degC|5000;j|8256|49|i16|1|degC|5000", "k|0|49|i16|1|degC|5000"})
    assert(!parse_monitor_spec(bad, &monitors));
}

// Reject every unsupported width and receipt identity. Failed uploads consume
// exactly one attempt; an epoch change or clear/reassert cannot reuse the reply.
void drive_alarm_failed_and_stale_reads() {
  for (unsigned failure = 0; failure < 12; ++failure) {
    MappingTestFixture f;
    auto& m = *f.storage;
    auto& alarm = m.drive_alarms[0];
    g_stop.store(false);
    alarm.observe(0x6320);
    f.master.slaves[0]->sdos[0x203f00] = {0x10, 0x0f};
    if (failure < 3) f.master.slaves[0]->sdos[0x203f00] = std::vector<uint8_t>(failure == 2 ? 3 : failure, 0);
    else if (failure < 8) mapping_test_identity_field = failure - 2;
    else if (failure == 8) f.master.slaves[0]->sdos.erase(0x203f00);
    else if (failure == 9) mapping_test_after_upload = +[] { mapping_test_machine->configuration_epoch.fetch_add(1); };
    else if (failure == 10) mapping_test_after_upload = +[] { mapping_test_machine->drive_alarms[0].observe(0); };
    else mapping_test_after_upload = +[] {
      mapping_test_machine->drive_alarms[0].observe(0);
      mapping_test_machine->drive_alarms[0].observe(0x6320);
    };
    m.refresh_drive_alarm();
    assert(mapping_test_uploads.size() == 1);
    assert(!alarm.snapshot(alarm.event.load(), m.configuration_epoch.load(), 0x6320).alarm_aux_valid);
    mapping_test_after_upload = nullptr;
    mapping_test_identity_field = 0;
    if (failure == 11) {
      m.refresh_drive_alarm();
      assert(mapping_test_uploads.size() == 2);
      assert(alarm.snapshot(alarm.event.load(), 1, 0x6320).alarm_aux_valid);
    } else {
      for (unsigned i = 0; i < 100; ++i) m.refresh_drive_alarm();
      assert(mapping_test_uploads.size() == 1);
    }
  }
}

// Background diagnostics must run even with a connected controller and no
// metrics readback request; one checkpoint services only one axis.
void drive_alarm_worker_connected() {
  MappingTestFixture f;
  auto& m = *f.storage;
  g_stop.store(false);
  m.control_policy.connected.store(true);
  m.readbacks_requested.store(false);
  m.diagnostic_handles_ready.store(false);
  m.commissioning_running.store(true);
  m.drive_alarms[0].observe(0x6320);
  m.drive_alarms[1].observe(0x6320);
  f.master.slaves[0]->sdos[0x203f00] = {0x10, 0x0f};
  f.master.slaves[1]->sdos[0x203f00] = {0x12, 0x0f};
  mapping_test_after_upload = +[] { mapping_test_machine->commissioning_running.store(false); };
  m.run_commissioning_worker();
  assert(mapping_test_uploads.size() == 1 && mapping_test_uploads[0].position == 0);
  assert(m.drive_alarms[0].snapshot(m.drive_alarms[0].event.load(), 1, 0x6320).alarm_aux_valid);
  assert(m.alarm_readback_pending());
  mapping_test_after_upload = nullptr;
  m.refresh_drive_alarm();
  assert(mapping_test_uploads.size() == 2 && mapping_test_uploads[1].position == 1);
  assert(!m.alarm_readback_pending());
}

// The measured alarm must be unique only with the auxiliary low half. Unknown
// evidence exposes candidates, and unknown/clear bus codes cannot invent names.
void drive_alarm_decode() {
  using rosie::rtcore::decode_a6ec_alarm;
  const auto named = decode_a6ec_alarm(0x6320, 0xbeef0f10, true);
  assert(named.code == "ALF1.0");
  assert(named.text == "Re-power-on required for parameter settings to take effect");
  const auto candidates = decode_a6ec_alarm(0x6320, 0, false);
  assert(candidates.code.find("ALF1.0 | ALF1.2 | ALF4.0") == 0);
  assert(candidates.code.find("Er03.") != std::string::npos);
  assert(candidates.code == decode_a6ec_alarm(0x6320, 0xffff, true).code);
  assert(decode_a6ec_alarm(0, 0x0f10, true).code.empty());
  assert(decode_a6ec_alarm(0xffff, 0x0f10, true).text.empty());
  for (const auto& row : rosie::rtcore::a6ec_alarm_codes) {
    if (!row.bus_valid) continue;
    const auto decoded = decode_a6ec_alarm(row.bus_code, row.aux_code, true);
    assert(decoded.code.find(row.display_code) != std::string::npos);
    assert(decoded.text.find(row.meaning) != std::string::npos);
  }
}

// Review source-wrap reproducer, extended in both directions. Unverified
// fixture: W=131072 counts, 1000 counts/s, 10000000 ns age and 1-count error.
// Wrapped uploads must retain PDO continuity; a continuous source displaced
// by a genuine 131072 counts must still invalidate the established frame.
void battery_anchor_source_wrap() {
  using namespace rosie::rtcore;
  for (bool absent : {false, true}) for (int direction : {-1, 1}) {
    MappingTestFixture f;
    auto& m = *f.storage;
    g_stop.store(false);
    mailbox_test_now = 1500000000;
    m.mailbox_clock = mailbox_test_clock;
    m.update_mailbox_readiness(false, mailbox_test_now);
    settle_mailbox(m, f.master, mailbox_test_now);
    m.verified_configuration_epoch.store(1);
    m.control_policy.config_verified_mask.store(1);
    auto& p = f.configuration.axis_profiles[0];
    p.encoder_battery_absent = absent;
    m.sampled_consumption[0].publish({});
    f.options.axis[0].counts_per_unit = 131072 / 6.28318530717958647692;
    CoordinatePolicy policy{131072, 1000, 10000000, 1, 1, true};
    SampledPdoHistory history;
    history.configure(10000000, 1000000);
    SampledAbsolutePosition retained{};
    auto& dictionary = f.master.slaves[0]->sdos;
    for (const auto key : {0x20401du, 0x20402bu, 0x20402du}) dictionary[key].resize(4);
    for (unsigned cycle : {1u, 2u, 3u}) {
      m.rt_cycle_counter.store(cycle);
      const int32_t pdo = direction < 0 ? (cycle == 1 ? 0 : -1) : (cycle == 1 ? 131071 : 131072);
      const int32_t wrapped = (pdo % 131072 + 131072) % 131072;
      // Mode 3 cannot supply whole turns: out-of-range data is a source
      // failure. Mode 4 can, and its whole-turn disagreement is incoherent.
      const int32_t encoder = pdo + (cycle == 3 ? 131072 : 0);
      EC_WRITE_S32(dictionary[0x20401d].data(), wrapped + (cycle == 3 ? 131072 : 0));
      EC_WRITE_S32(dictionary[0x20402b].data(), encoder);
      EC_WRITE_S32(dictionary[0x20402d].data(), encoder < 0 ? -1 : 0);
      history.observe(1, 0, cycle, mailbox_test_now, pdo, true, policy);
      assert(m.sample_absolute_axis(0, mailbox_test_clock) == !(absent && cycle == 3));
      SampledAbsolutePosition sample{};
      assert(m.sampled_absolute[0].read(sample));
      const bool valid = history.consume(sample, retained, policy, mailbox_test_now, false);
      if (cycle < 3) {
        assert(valid && !history.failed && history.metrics.startup_verified == 1);
        assert(history.metrics.coherence_error_counts == 0 && history.command_counts == pdo);
      } else {
        assert(!valid && history.failed);
        assert(history.metrics.reason == uint32_t(absent ? SampleInvalidity::Range : SampleInvalidity::Incoherent));
        if (!absent) assert(history.metrics.coherence_error_counts == 131072);
      }
      mailbox_test_now += 1000000;
    }
  }
}

// CoE revision must come from the admitted bus upload, never the profile's SII
// expectation. Missing/malformed uploads and SII mismatch cannot verify axis 0.
void simulated_revision_readback() {
  for (unsigned fault = 0; fault < 4; ++fault) {
    MappingTestFixture f;
    auto& m = *f.storage;
    g_stop.store(false);
    // Measured bench identities: SII 12024 and CoE firmware revision 22033.
    f.configuration.axis_profiles[0].revision_no = 12024;
    f.master.slaves[0]->sii_revision = fault == 3 ? 22034 : 12024;
    auto& dictionary = f.master.slaves[0]->sdos;
    if (fault != 1) {
      auto& revision = dictionary[0x101803];
      revision.resize(4);
      EC_WRITE_U32(revision.data(), 22033);
      if (fault == 2) revision.resize(2);
    }
    m.diagnostic_handles_ready.store(true);
    m.commissioning_running.store(true);
    // Complete exactly one worker pass; uploads and publication prove success.
    mapping_test_after_upload = +[] { mapping_test_machine->commissioning_running.store(false); };
    m.run_commissioning_worker();
    mapping_test_after_upload = nullptr;
    assert(std::count_if(mapping_test_uploads.begin(), mapping_test_uploads.end(),
        [](const auto& read) { return read.position == 0 && read.index == 0x1018 && read.subindex == 3; }) == 1);
    assert(m.observed_revision_valid_mask.load() == (fault == 1 || fault == 2 ? 0u : 1u));
    assert(m.observed_revision[0].load() == (fault == 1 || fault == 2 ? 0u : 22033u));
    MachineState::ConfigurationReadback published{};
    assert(m.configuration_readbacks.read(published));
    assert(published.epoch == 1 && published.verified_mask == (fault == 0 ? 511u : 510u));
    assert(published.mapping_mismatch_mask == 0);
  }
}

// Observe-only drive tuning reads (owner decision 2026-10-07): each entry is
// uploaded once through the admitted mailbox and decoded with its own width
// and sign; a failing object is retried within the bound and then skipped.
// Nothing is written, and neither the startup feedback nor the published
// configuration verification changes. A stop or a closed mailbox admits no
// upload and consumes no attempt.
void startup_observe_reads() {
  MappingTestFixture f;
  auto& m = *f.storage;
  g_stop.store(false);
  m.axis_profiles[0].startup_observe = {{"inertia", StartupSdoValueType::kU16, 0x2000, 0x07},
      {"lowpass", StartupSdoValueType::kI16, 0x2002, 0x3b}, {"wide", StartupSdoValueType::kU32, 0x2002, 0x10}};
  m.axis_profiles[2].startup_observe = {{"absent", StartupSdoValueType::kU16, 0x2001, 0x41}};
  auto& dictionary = f.master.slaves[0]->sdos;
  dictionary[0x200007] = {0xfa, 0x00};
  dictionary[0x20023b] = {0x85, 0xff};
  dictionary[0x200210] = {0xf0, 0xff, 0xff, 0xff};
  StartupSdoConfig descriptor{};
  descriptor.valid = true; descriptor.axis_mask = 1; descriptor.key = "written";
  descriptor.type = StartupSdoValueType::kU16; descriptor.index = 0x2001; descriptor.subindex = 0x14;
  descriptor.values[0] = 1;
  m.startup_sdos.push_back(descriptor);
  m.latest_feedback.startup_drive_config_feedback[0].readback_valid[0] = 1;
  m.latest_feedback.startup_drive_config_feedback[0].readback[0] = 1;
  m.latest_feedback.startup_drive_config_feedback[0].verified[0] = 1;
  m.configuration_readbacks.publish({1, 1, 0, m.verification_request.load()});
  const auto dictionary_before = dictionary;

  g_stop.store(true);
  assert(!m.startup_observe_step() && mapping_test_uploads.empty() && m.startup_observe_progress.attempts == 0);
  g_stop.store(false);
  m.latest_feedback.domain_wc_state = EC_WC_INCOMPLETE;
  assert(!m.startup_observe_step() && mapping_test_uploads.empty() && m.startup_observe_progress.entry == 0);
  m.latest_feedback.domain_wc_state = EC_WC_COMPLETE;

  const int64_t expected[] = {250, -123, 4294967280LL};
  for (const auto value : expected) {
    assert(!m.startup_observe_step());
    assert(m.startup_observe_progress.last_value == value);
  }
  for (unsigned attempt = 1; attempt <= 3; ++attempt) assert(!m.startup_observe_step());
  assert(m.startup_observe_step() && m.startup_observe_step());
  const auto& progress = m.startup_observe_progress;
  assert(progress.complete && progress.reads == 3 && progress.failed == 1);
  assert(mapping_test_uploads.size() == 6);
  for (size_t i = 0; i < 3; ++i) assert(mapping_test_uploads[i].position == 0);
  for (size_t i = 3; i < 6; ++i)
    assert(mapping_test_uploads[i].position == 2 && mapping_test_uploads[i].index == 0x2001 && mapping_test_uploads[i].subindex == 0x41);
  assert(dictionary == dictionary_before);
  const auto& feedback = m.latest_feedback.startup_drive_config_feedback[0];
  assert(feedback.verified[0] == 1 && feedback.readback[0] == 1 && feedback.readback_valid[0] == 1);
  MachineState::ConfigurationReadback published{};
  assert(m.configuration_readbacks.read(published) && published.epoch == 1 && published.verified_mask == 1);
}

int run_mapping_readback_tests(const std::string& filter) {
  unsigned ran = 0;
  auto run = [&](const char* name, auto test) {
    if (!filter.empty() && filter != name) return;
    ++ran;
    std::fprintf(stderr, "RUN %s\n", name);
    test();
    std::fprintf(stderr, "PASS %s\n", name);
  };
  run("TestSimulationFixedMappingRefusedMidChain", [] { mapping_test_retry(MappingTestFault::Refused); });
  run("TestSimulationFixedMappingLateWideResponse", [] { mapping_test_retry(MappingTestFault::WideLate); });
  run("TestSimulationFixedMappingIncompleteMismatch", mapping_test_incomplete_mismatch);
  run("TestSimulationFixedMappingLateResponse", mapping_test_late_sequence);
  run("TestSimulationFixedMappingScalarIdentityAndWidth", mapping_test_scalar_identity_and_width);
  run("TestSimulationFixedMappingRetryBound", mapping_test_retry_bound);
  run("TestSimulationFixedMappingNamedMismatch", mapping_test_named_mismatch);
  run("TestRuntimeMailboxReviewGates", runtime_mailbox_gates);
  run("TestRuntimeMailboxSettleAndInflight", runtime_mailbox_boundary_and_inflight);
  run("TestRuntimeStartupRequiresOP", runtime_startup_requires_op);
  run("TestDriveAlarmAdmissionEdgesAndWidths", drive_alarm_admission_edges_and_widths);
  run("TestDriveMonitorReadsDueObjects", drive_monitor_reads_due_objects);
  run("TestDriveAlarmFailedAndStaleReads", drive_alarm_failed_and_stale_reads);
  run("TestDriveAlarmWorkerConnected", drive_alarm_worker_connected);
  run("TestDriveAlarmDecode", drive_alarm_decode);
  run("TestSimulationRevisionReadback", simulated_revision_readback);
  run("TestStartupObserveReadsWithoutVerification", startup_observe_reads);
  return ran ? 0 : 1;
}

// Ordered record of the worker's drive-facing actions while a test installs it:
// cyclic service requests and every SDO download, in submission order.
std::vector<std::string>* worker_drive_events = nullptr;
int recorded_sdo_download(ec_master_t* master, uint16_t position, uint16_t index, uint8_t subindex,
                          const uint8_t* data, size_t size, uint32_t* abort) {
  if (worker_drive_events) {
    const int64_t value = size == 1 ? int64_t(int8_t(data[0])) :
        size == 2 ? int64_t(int16_t(data[0] | (data[1] << 8))) : int64_t(EC_READ_S32(data));
    char text[64];
    std::snprintf(text, sizeof(text), "sdo 0x%04x:%u=%lld", unsigned(index), unsigned(subindex),
                  static_cast<long long>(value));
    worker_drive_events->push_back(text);
  }
  return ecrt_master_sdo_download(master, position, index, subindex, data, size, abort);
}
}

#define ecrt_master_sdo_upload mapping_test_upload
#define upload_sdo_read mapping_test_read
#define ecrt_master_sdo_download recorded_sdo_download
#include "engine/commissioning_worker.hpp"
#undef ecrt_master_sdo_upload
#undef upload_sdo_read
#undef ecrt_master_sdo_download
#include <cassert>

namespace {
// Atomic purpose and scoped release keep background readback distinguishable
// from an operator job, including early failure and an unread receipt queue.
void commissioning_claim_release() {
  Options options;
  OptionsParseResult configuration;
  auto storage = std::make_unique<MachineState>(options, configuration);
  auto& machine = *storage;
  using Claim = MachineState::CommissioningClaim;
  Claim held;
  assert(machine.try_claim_commissioning(Claim::Readback));
  assert(!machine.try_claim_commissioning(Claim::Command, &held) && held == Claim::Readback);
  {
    MachineState::CommissioningClaimRelease release{&machine.commissioning_claim};
    release.release();
    assert(machine.try_claim_commissioning(Claim::Command));
  }
  assert(!machine.try_claim_commissioning(Claim::Command, &held) && held == Claim::Command);
  {
    MachineState::CommissioningClaimRelease release{&machine.commissioning_claim};
    // The queue's declared 64 receipts remain unread, like a vanished consumer.
    for (unsigned receipt = 0; receipt < 64; ++receipt)
      assert(machine.commissioning_job_receipts.push({}));
    assert(!machine.commissioning_job_receipts.push({}));
  }
  assert(machine.try_claim_commissioning(Claim::Readback));
  {
    MachineState::CommissioningClaimRelease release{&machine.commissioning_claim};
  }
  assert(machine.try_claim_commissioning(Claim::Command));
}

// Fixed feedback and a recorded process image make the one-cycle inhibition
// assertion independent of thread scheduling and mailbox completion.
struct CommissioningTestBus {
  std::array<uint8_t, 64 * rosie::protocol::max_axes> image{};
  uint32_t feedback_mask = 1;
  rosie::rtcore::CycleBusResults bus_results{};
  ec_domain_state_t state{2, EC_WC_COMPLETE};
  uint32_t submissions = 0;
  bool begin_cycle(uint64_t) { return true; }
  uint32_t feedback_axis_mask() const { return feedback_mask; }
  const auto& results() const { return bus_results; }
  const auto& domain_state() const { return state; }
  rosie::rtcore::PdoView axis(uint32_t axis) { return rosie::rtcore::PdoView(image.data() + 64 * axis); }
  void sync_clocks(uint64_t) {}
  rosie::rtcore::BusResult query_state() { return {rosie::rtcore::BusOperation::DomainState, 0}; }
  rosie::rtcore::BusResult queue(uint32_t) { return {rosie::rtcore::BusOperation::Queue, 0}; }
  rosie::rtcore::BusSubmission submit(uint32_t mask) {
    ++submissions;
    return {{rosie::rtcore::BusOperation::Send, 64}, mask, rosie::rtcore::SubmissionSkip::None};
  }
};
struct CommissioningTestHandles {
  rosie::rtcore::CellIOWire cell_io_wire;
  CommissioningTestBus cycle_backend;
  std::array<BusHandles::AxisOffsets, rosie::protocol::max_axes> off{};
  std::array<ec_slave_config_t*, rosie::protocol::max_axes> sc{};
  std::array<unsigned int*, rosie::protocol::max_axes> brake_signal_offsets{};
  bool ecrt_ok = true;
  uint32_t expected_wkc = 2;
  uint64_t startup_begin_ns = 0, startup_cycle_counter = 0;
  void log_phase_summary(const char*, uint64_t, uint64_t) {}
};
uint64_t commissioning_test_now = 1'000'000'000;
uint64_t commissioning_test_clock() { return commissioning_test_now; }

// Exercise actual off-cycle uploads and native verification publication. The
// slave fixture changes one descriptor without changing the registered layout.
void fixed_mapping_readback() {
  Options options;
  options.num_axes = 1;
  options.axis[0].counts_per_unit = 13107200 / 6.28318530717958647692;
  OptionsParseResult configuration;
  auto& pdo = configuration.axis_profiles[0].pdo;
  pdo.rx_pdo = 0x1702; pdo.tx_pdo = 0x1b02;
  pdo.rx_sync_index = 2; pdo.tx_sync_index = 3;
  assert(parse_pdo_layout_spec("cw|0x6040|0|16;target_pos|0x607a|0|32;mode|0x6060|0|8", &pdo.rx_entries));
  assert(parse_pdo_layout_spec("sw|0x6041|0|16;pos|0x6064|0|32;mode_disp|0x6061|0|8", &pdo.tx_entries));
  auto& profile = configuration.axis_profiles[0];
  profile.home.valid = true;
  profile.home.truth_source = {true, 0x607c, 0, SdoScalarType::kI32};
  profile.home_truth_sign = -1;
  profile.coordinate_evidence = {true, 5000000, 3000000000, 16, 16, 0, 0, true, 1500000000};
  profile.anchor_source = {0x2040, 0x21, 0x23, "a6ec_raw_multi_turn", "raw_multi_turn_encoder", 3};
  profile.absolute.valid = true;
  profile.absolute.field_count = 5;
  profile.absolute.fields[0] = {true, "multi_turn_low", {true, 0x2040, 0x21, SdoScalarType::kI32}};
  profile.absolute.fields[1] = {true, "multi_turn_high", {true, 0x2040, 0x23, SdoScalarType::kI32}};
  profile.absolute.fields[2] = {true, "rotation_reference", {true, 0x2040, 0x29, SdoScalarType::kI32}};
  profile.absolute.fields[3] = {true, "linear_mode_encoder_low", {true, 0x2040, 0x25, SdoScalarType::kI32}};
  profile.absolute.fields[4] = {true, "linear_mode_encoder_high", {true, 0x2040, 0x27, SdoScalarType::kI32}};
  auto storage = std::make_unique<MachineState>(options, configuration);
  auto& machine = *storage;
  assert(machine.metrics_absolute_feedback_poll_enabled);
  ec_master_t master;
  auto* slave = ecrt_master_slave_config(&master, 0, 0, 4194304, 1813);
  ec_pdo_entry_info_t rx[] = {{0x6040, 0, 16}, {0x607a, 0, 32}, {0x6060, 0, 8}};
  ec_pdo_entry_info_t tx[] = {{0x6041, 0, 16}, {0x6064, 0, 32}, {0x6061, 0, 8}};
  ec_pdo_info_t pdos[] = {{0x1702, 3, rx}, {0x1b02, 3, tx}};
  ec_sync_info_t syncs[] = {{2, EC_DIR_OUTPUT, 1, &pdos[0], EC_WD_ENABLE},
      {3, EC_DIR_INPUT, 1, &pdos[1], EC_WD_DISABLE}, {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}};
  assert(ecrt_slave_config_pdos(slave, EC_END, syncs) == 0);
  machine.shared_master.store(&master);
  settle_mailbox(machine, master, now_monotonic_ns());
  assert(machine.verify_axis_mapping(0) == MappingReadback::Matched);
  const auto mapping_key = (0x1c12u << 8);
  auto saved_assignment = slave->sdos[mapping_key];
  slave->sdos.erase(mapping_key);
  MappingVerification transient, persistent;
  transient.observe(machine.verify_axis_mapping(0));
  assert(transient.attempts == 1 && transient.pending() && !transient.faulted);
  for (unsigned attempt = 1; attempt <= 3; ++attempt) {
    persistent.observe(machine.verify_axis_mapping(0));
    assert(persistent.attempts == attempt);
    assert(persistent.faulted == (attempt == 3));
  }
  slave->sdos[mapping_key] = saved_assignment;
  transient.observe(machine.verify_axis_mapping(0));
  assert(transient.attempts == 2 && transient.verified && !transient.faulted);
  slave->sii_revision = 12024;
  auto& coe_revision = slave->sdos[(0x1018u << 8) | 3];
  coe_revision.resize(4); EC_WRITE_U32(coe_revision.data(), 22033);
  assert(verify_sii_revision(&master, 0, 12024));
  assert(!verify_sii_revision(&master, 0, 22033));
  slave->sii_revision = 22034;
  EC_WRITE_U32(coe_revision.data(), 12024);
  assert(!verify_sii_revision(&master, 0, 12024));
  assert(!verify_sii_revision(nullptr, 0, 12024));
  MachineState::cycle_owner = &machine;
  machine.configuration_readbacks.publish({1, 1, 0, machine.verification_request.load()});
  machine.consume_control_requests();
  assert(machine.control_policy.config_verified_mask.load() == 1);
  assert(machine.execution_fault_reasons.load() == 0);
  // Matching status offsets cannot disguise a mismatched mapped width.
  slave->sdos[(0x1b02u << 8) | 2][0] = 16;
  assert(machine.verify_axis_mapping(0) == MappingReadback::Mismatch);
  machine.configuration_readbacks.publish({1, 1, 1, machine.verification_request.load()});
  machine.consume_control_requests();
  assert(machine.control_policy.config_verified_mask.load() == 0);
  assert(machine.execution_fault_reasons.load() == rosie::rtcore::FaultPdoMappingMismatch);
  assert(machine.safety_fault_mask.load() == 1);
  slave->sdos[(0x1b02u << 8) | 2][0] = 32;
  slave->sdos[(0x1c13u << 8) | 1][0] = 3;
  assert(machine.verify_axis_mapping(0) == MappingReadback::Mismatch);

  auto sdo = [&](uint8_t sub, int32_t value) {
    auto& bytes = slave->sdos[(0x2040u << 8) | sub];
    bytes.resize(4); EC_WRITE_S32(bytes.data(), value);
  };
  sdo(0x21, 930051); sdo(0x23, 0); sdo(0x29, 6509175);
  sdo(0x2b, 6509175); sdo(0x2d, 0);
  machine.bus_ready.store(true);
  machine.rt_cycle_counter.store(42);
  assert(machine.sample_absolute_axis(0));
  rosie::rtcore::SampledAbsolutePosition sample;
  assert(machine.sampled_absolute[0].read(sample));
  assert(sample.valid && sample.cycle == 42 && sample.encoder_counts == 930051);
  assert(sample.started_ns > 0 && sample.completed_ns >= sample.started_ns);
  assert(machine.latest_feedback.absolute_feedback[0].fields[0].value == 930051);
  assert(machine.latest_feedback.absolute_feedback[0].fields[1].value == 0);
  assert(machine.latest_feedback.absolute_feedback[0].fields[0].valid == sample.valid);
  assert(machine.latest_feedback.absolute_feedback[0].fields[1].valid == sample.valid);
  // bench-servo 2026-09-13 readback: Home offset 6553600 and encoder 930051
  // are independent. Interleaved truth/diagnostic uploads preserve the sample.
  auto& home_offset = slave->sdos[0x607cu << 8];
  home_offset.resize(4); EC_WRITE_S32(home_offset.data(), 6553600);
  int32_t truth = 0;
  assert(machine.read_native_home_truth_axis(0, &truth) && truth == -6553600);
  AbsoluteFeedbackFieldSample diagnostic;
  assert(machine.read_absolute_feedback_field_axis(0, 0x2040, 0x21, SdoScalarType::kI32, &diagnostic));
  assert(diagnostic.valid == 1 && diagnostic.value == 930051);
  assert(machine.sampled_absolute[0].read(sample) && sample.encoder_counts == 930051 && sample.home_epoch == 0);
  sdo(0x21, -1); sdo(0x23, -1);
  assert(machine.sample_absolute_axis(0));
  assert(machine.sampled_absolute[0].read(sample) && sample.encoder_counts == -1);
  assert(machine.latest_feedback.absolute_feedback[0].fields[0].value == -1);
  assert(machine.latest_feedback.absolute_feedback[0].fields[1].value == -1);
  assert(machine.latest_feedback.absolute_feedback[0].fields[0].valid == sample.valid);
  assert(machine.latest_feedback.absolute_feedback[0].fields[1].valid == sample.valid);
  sdo(0x21, 0); sdo(0x23, 1);
  assert(machine.sample_absolute_axis(0));
  assert(machine.sampled_absolute[0].read(sample) && sample.encoder_counts == 4294967296LL);
  for (int64_t counts : {-4294967299LL, -4294967297LL, -4294967296LL, -4294967295LL, -3LL, -1LL, 0LL, 1LL}) {
    const int32_t low = static_cast<int32_t>(static_cast<uint32_t>(counts));
    const int32_t high = int32_t((counts - static_cast<uint32_t>(low)) / 4294967296LL);
    sdo(0x21, low); sdo(0x23, high);
    assert(machine.sample_absolute_axis(0));
    assert(machine.sampled_absolute[0].read(sample) && sample.encoder_counts == counts);
    assert(machine.latest_feedback.absolute_feedback[0].fields[0].value == low);
    assert(machine.latest_feedback.absolute_feedback[0].fields[1].value == high);
    assert(machine.latest_feedback.absolute_feedback[0].fields[0].valid == sample.valid);
    assert(machine.latest_feedback.absolute_feedback[0].fields[1].valid == sample.valid);
  }
  slave->sdos.erase((0x2040u << 8) | 0x23);
  assert(!machine.sample_absolute_axis(0));
  assert(machine.sampled_absolute[0].read(sample) && !sample.valid);
  assert(machine.latest_feedback.absolute_feedback[0].fields[0].valid == sample.valid);
  assert(machine.latest_feedback.absolute_feedback[0].fields[1].valid == sample.valid);
  machine.shared_master.store(nullptr);
  MachineState::cycle_owner = nullptr;
}

// Drive a fixed-mapping cycle with no continuous PDO offsets. A fresh mailbox
// pair establishes coordinates; motion preserves validity and incoherence latches loss.
void sampled_fixed_mapping_cycle() {
  Options options;
  options.num_axes = 1;
  options.cycle_ns = 1000000;
  options.axis[0].counts_per_unit = 13107200 / 6.28318530717958647692;
  OptionsParseResult configuration;
  auto& profile = configuration.axis_profiles[0];
  profile.coordinate_evidence = {true, 5000000, 2000000000, 16, 16, 0, 0, true, 1500000000};
  profile.limits.valid = true;
  profile.limits.min_q = -8; profile.limits.max_q = 10;
  auto storage = std::make_unique<MachineState>(options, configuration);
  auto& machine = *storage;
  CommissioningTestHandles handles;
  handles.off[0].cw = 0; handles.off[0].target_pos = 4; handles.off[0].target_vel = 8;
  handles.off[0].sw = 16; handles.off[0].pos = 20; handles.off[0].mode_disp = 26; handles.off[0].err = 28;
  auto image = handles.cycle_backend.axis(0);
  image.write_u16(16, 0x40); image.write_s32(20, 6509174); image.write_u8(26, 8);
  ec_master_t master;
  master.active = true;
  master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
  handles.sc[0] = master.slaves[0].get();
  machine.shared_master.store(&master);
  machine.bus_ready.store(true);
  machine.startup_ready = true;
  settle_mailbox(machine, master, 1000000000);
  machine.axis_coordinate_epoch[0] = 1;
  machine.verified_configuration_epoch.store(1);
  machine.control_policy.config_verified_mask.store(1);
  // PDO history starts before the first mailbox sample. Move at the bench
  // velocity cap: 5000 counts per 1000000 ns, through successive acquisitions.
  for (uint64_t cycle = 1; cycle <= 2200; ++cycle) {
    const int32_t position = 6509174 + int32_t(cycle - 1) * 5000;
    image.write_s32(20, position);
    commissioning_test_now = 1000000000 + (cycle - 1) * 1000000;
    if (cycle == 2 || cycle == 1502) {
      machine.sampled_absolute[0].publish({1, 0, cycle - 1,
          commissioning_test_now - 1000000, commissioning_test_now - 1,
          position - 5000, 1});
    }
    machine.next_ns = commissioning_test_now;
    machine.previous_cycle_ns = commissioning_test_now - 1000000;
    const auto result = machine.step(CycleInputs<CommissioningTestHandles>{handles, nullptr, commissioning_test_clock});
    assert(result.submitted_mask == 1 && result.motion_permitted_mask == 0);
    if (cycle > 1) {
      assert(machine.latest_feedback.coordinate_source_valid[0] == 1);
      assert(machine.latest_feedback.coordinate_valid[0] == 1);
      assert(machine.latest_feedback.coordinate_counts[0] == position);
      assert(machine.execution_fault_reasons.load() == 0);
    }
  }
  // Acquisition is 1000000 ns: 5000 physical counts + 16 count error.
  // An offset of 5017 counts is incoherent even though PDOs remain fresh.
  machine.sampled_absolute[0].publish({1, 0, 2200, commissioning_test_now,
      commissioning_test_now + 1000000, 6509174 + 2199 * 5000 + 5017, 1});
  commissioning_test_now += 1000000;
  image.write_s32(20, 6509174 + 2200 * 5000);
  machine.next_ns = commissioning_test_now;
  machine.previous_cycle_ns = commissioning_test_now - 1000000;
  machine.step(CycleInputs<CommissioningTestHandles>{handles, nullptr, commissioning_test_clock});
  assert(machine.latest_feedback.coordinate_source_valid[0] == 0);
  assert(machine.latest_feedback.coordinate_valid[0] == 0);
  assert((machine.execution_fault_reasons.load() & rosie::rtcore::FaultCoordinateReference) == 64);
  assert(machine.sampled_bindings[0].metrics.coherence_error_counts == 5017);
  machine.shared_master.store(nullptr);
  commissioning_test_now = 1000000000;
}

// Unverified deterministic mailbox/cycle fixture: 1000000 ns PDO receipts,
// W=1310720 counts, three uploads per sample, no scheduler sleeps as assertions.
struct SampledCycleFixture {
  // MachineState keeps references to both inputs for every later cycle.
  // Fixture ownership prevents dangling configuration after construction.
  Options options;
  OptionsParseResult configuration;
  std::unique_ptr<MachineState> storage;
  CommissioningTestHandles handles;
  ec_master_t master;
  uint32_t count, mask, refresh_mask = 0;
  bool enabled = false;
  CycleOutputs last_outputs{};
  inline static SampledCycleFixture* active = nullptr;
  explicit SampledCycleFixture(uint32_t axes = 1, int32_t position = -1, bool single = false, bool bind = true, bool independent = false) : count(axes), mask((1u << axes) - 1) {
    options.num_axes = axes; options.cycle_ns = 1000000;
    for (uint32_t i = 0; i < axes; ++i) {
      options.slave_position[i] = i;
      options.axis[i].counts_per_unit = 1310720 / 6.28318530717958647692;
      options.axis[i].max_profile_vel_counts_per_s = 5000000;
      configuration.axis_profiles[i].coordinate_evidence = {true, 5000000, 3000000000, 16, 16, 0, 0, true, 1500000000};
      (void)single;
      (void)independent;
      configuration.axis_profiles[i].vendor_id = 4194304;
      configuration.axis_profiles[i].product_code = 1813;
      configuration.axis_profiles[i].home_truth_sign = -1;
      configuration.axis_profiles[i].anchor_source =
          {0x2040, 0x21, 0x23, "a6ec_raw_multi_turn", "raw_multi_turn_encoder", 3};
      auto& limits = configuration.axis_profiles[i].limits;
      // Unverified fixture travel: single-turn span 6 rad is strictly below 2*pi.
      limits.valid = true; limits.min_q = single ? -3 : -100; limits.max_q = single ? 3 : 100;
      limits.max_qd = 25; limits.jog_acceleration = 25;
      limits.max_target_lead = 1; limits.following_error = 2;
      limits.following_error_timeout_ns = 1000000000;
      handles.off[i].cw = 0; handles.off[i].target_pos = 4; handles.off[i].target_vel = 8;
      handles.off[i].sw = 16; handles.off[i].pos = 20; handles.off[i].mode_disp = 26; handles.off[i].err = 28;
      auto* slave = ecrt_master_slave_config(&master, 0, i, 4194304, 1813);
      slave->raw_multi_turn_encoder_source = true; slave->raw_multi_turn_encoder_counts.store(position);
      slave->sdos[0x204021].resize(4); EC_WRITE_S32(slave->sdos[0x204021].data(), position);
      slave->sdos[0x204023].resize(4); EC_WRITE_S32(slave->sdos[0x204023].data(), position < 0 ? -1 : 0);
      handles.sc[i] = slave;
      auto pd = handles.cycle_backend.axis(i);
      pd.write_u16(16, 0x27); pd.write_s32(20, position); pd.write_u8(26, 8);
    }
    handles.expected_wkc = axes * 2;
    handles.cycle_backend.state.working_counter = axes * 2;
    handles.cycle_backend.feedback_mask = mask;
    master.active = true;
    storage = std::make_unique<MachineState>(options, configuration);
    auto& m = *storage;
    m.shared_master.store(&master); m.bus_ready.store(true); m.startup_ready = true;
    m.mailbox_clock = commissioning_test_clock;
    settle_mailbox(m, master, 1000000000);
    m.verified_configuration_epoch.store(1); m.control_policy.config_verified_mask.store(mask);
    for (uint32_t i = 0; i < axes; ++i) m.axis_coordinate_epoch[i] = 1;
    commissioning_test_now = 1000000000;
    active = this;
    master.sample_advance_ns = [](uint64_t ns) { active->advance(commissioning_test_now + ns); };
    step();
    if (!bind) return;
    for (uint32_t i = 0; i < axes; ++i) {
      assert(m.sample_absolute_axis(i, commissioning_test_clock));
      rosie::rtcore::SampledAbsolutePosition sample{};
      assert(m.sampled_absolute[i].read(sample));
      assert(m.sampled_bindings[i].bind_independent_origin(sample, m.sampled_at_receipt[i],
          rosie::rtcore::CoordinatePolicy{5000000, 3000000000, 16, 16, true, 5000000}, commissioning_test_now, position, false));
      m.calibration_valid_mask.fetch_or(1u << i);
    }
    advance(1001000000);
    for (uint32_t i = 0; i < axes; ++i) assert(m.latest_feedback.coordinate_valid[i] == 1);
    m.control_policy.home_valid_mask.store(mask);
  }
  ~SampledCycleFixture() {
    storage->shared_master.store(nullptr); MachineState::cycle_owner = nullptr;
    active = nullptr; commissioning_test_now = 1000000000;
  }
  void step() {
    auto& m = *storage;
    m.next_ns = commissioning_test_now; m.previous_cycle_ns = commissioning_test_now - 1000000;
    if (enabled) m.control_policy.heartbeat_ns.store(commissioning_test_now);
    for (uint32_t i = 0; i < count; ++i)
      if ((refresh_mask & (1u << i)) && commissioning_test_now - m.absolute_last_poll_ns[i] >= 1500000000)
        assert(m.sample_absolute_axis(i, commissioning_test_clock));
    const auto result = m.step(CycleInputs<CommissioningTestHandles>{handles, nullptr, commissioning_test_clock});
    last_outputs = result;
    assert(result.submitted_mask == mask);
    if (!enabled) assert(result.motion_permitted_mask == 0);
  }
  void advance(uint64_t target) {
    while (commissioning_test_now < target) {
      commissioning_test_now += std::min(uint64_t{1000000}, target - commissioning_test_now);
      step();
    }
  }
};


void independent_raw_source_bracket() {
  using namespace rosie::rtcore;
  MappingTestFixture f;
  auto& m = *f.storage;
  auto& profile = m.axis_profiles[0];
  profile.vendor_id = 4194304; profile.product_code = 1813;
  profile.coordinate_evidence.sdo_sampled = true;
  profile.coordinate_evidence.sample_interval_ns = 1500000000;
  profile.anchor_source = {0x2040, 0x21, 0x23,
      "a6ec_raw_multi_turn", "raw_multi_turn_encoder", 3};
  auto& low = f.master.slaves[0]->sdos[0x204021]; low.resize(4); EC_WRITE_S32(low.data(), -473893);
  auto& high = f.master.slaves[0]->sdos[0x204023]; high.resize(4); EC_WRITE_S32(high.data(), -1);
  m.rt_cycle_counter.store(1);
  const auto source = profile.absolute_anchor_source();
    assert(source.version == 3 && std::string(source.identity_name) == "raw_multi_turn_encoder");
  assert(m.sample_absolute_axis(0, mailbox_test_clock));
  SampledAbsolutePosition proof{};
  assert(m.sampled_absolute[0].read(proof) && proof.encoder_counts == -473893 && proof.independent_origin);
  mapping_test_after_upload = +[] {
    if (mapping_test_uploads.back().subindex == 0x21) {
      auto* master = mapping_test_machine->shared_master.load();
      EC_WRITE_S32(master->slaves[0]->sdos[0x204023].data(), 0);
    }
  };
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  assert(m.sampled_absolute[0].read(proof) && proof.reason == uint32_t(SampleInvalidity::Torn));
  mapping_test_after_upload = +[] { mapping_test_machine->absolute_frame_epoch[0].fetch_add(1); };
  assert(!m.sample_absolute_axis(0, mailbox_test_clock));
  assert(m.sampled_absolute[0].read(proof) && proof.reason == uint32_t(SampleInvalidity::Epoch));
  mapping_test_after_upload = nullptr;
}

// Actual mailbox source and cyclic commissioning boundaries, without a drive.
// Raw encoder origin differs from command origin; both are saved independently.
void independent_raw_home_restore_and_recovery() {
  using namespace rosie::rtcore;
  for (bool restoring : {false, true}) {
    constexpr int32_t period = 13107200, command = period - 3, raw = 6643436;
    SampledCycleFixture f(1, command, false, false);
    auto& m = *f.storage;
    auto& profile = m.axis_profiles[0];
    profile.vendor_id = 4194304; profile.product_code = 1813; profile.home_truth_sign = 1;
    f.options.axis[0].counts_per_unit = period / 6.28318530717958647692;
    m.control_policy.connected.store(true);
    m.control_policy.heartbeat_ns.store(commissioning_test_now);
    auto pd = f.handles.cycle_backend.axis(0);
    pd.write_u16(16, 0x40);
    auto set_raw = [&](int64_t value) {
      f.master.slaves[0]->raw_multi_turn_encoder_counts.store(value);
      auto& low = f.master.slaves[0]->sdos[0x204021]; low.resize(4); EC_WRITE_U32(low.data(), uint32_t(value));
      auto& high = f.master.slaves[0]->sdos[0x204023]; high.resize(4); EC_WRITE_U32(high.data(), uint32_t(uint64_t(value) >> 32));
    };
    set_raw(raw);
    m.absolute_frame_epoch[0].store(1);
    f.advance(1001000000);
    mapping_test_uploads.clear();
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    assert(mapping_test_uploads.size() == 3);
    assert(mapping_test_uploads[0].subindex == 0x23 && mapping_test_uploads[1].subindex == 0x21 && mapping_test_uploads[2].subindex == 0x23);
    SampledAbsolutePosition sample{};
    assert(m.sampled_absolute[0].read(sample) && sample.independent_origin && sample.encoder_counts == raw);
    f.advance(1002000000);
    assert(!m.latest_feedback.coordinate_valid[0] && !m.sampled_bindings[0].bound);
    // A later CompleteHome qualification failure must discard the tentative
    // independent origin. Fixing that condition cannot silently finish Home.
    {
      const CommissioningIdentity identity{5, m.control_policy.generation.load(), 1, m.home_epoch.load()};
      m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
      MachineState::CommissioningEnvelope rejected{{5, identity, CommissioningOperation::CompleteHome, 1, 0, 0, command}};
      rejected.restoring = restoring;
      rejected.anchor.counts = command; rejected.anchor.turns = raw; rejected.anchor.tolerance_counts = 16;
      assert(m.sampled_absolute[0].read(rejected.home_sample));
      profile.limits.valid = false;
      assert(m.commissioning_requests.push(rejected));
      f.advance(commissioning_test_now + 1000000);
      assert(m.commissioning_state.completed_mask == 0 && !m.latest_feedback.coordinate_valid[0]);
      assert(!m.sampled_bindings[0].bound && m.sampled_bindings[0].failed);
      profile.limits.valid = true;
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
      f.advance(commissioning_test_now + 1000000);
      assert(!m.latest_feedback.coordinate_valid[0] && m.control_policy.home_valid_mask.load() == 0);
      m.absolute_frame_epoch[0].fetch_add(1);
      f.advance(commissioning_test_now + 1000000);
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
    }
    auto complete = [&](bool restore, uint64_t job, int32_t home_command, int32_t expected_command) {
      m.control_policy.heartbeat_ns.store(commissioning_test_now);
      const CommissioningIdentity identity{job, m.control_policy.generation.load(), 1, m.home_epoch.load()};
      m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
      MachineState::CommissioningEnvelope envelope{{job, identity, CommissioningOperation::CompleteHome, 1, 0, 0, home_command}};
      envelope.restoring = restore;
      assert(m.sampled_absolute[0].read(envelope.home_sample));
      envelope.anchor.counts = home_command; envelope.anchor.turns = int32_t(envelope.home_sample.encoder_counts);
      envelope.anchor.tolerance_counts = 16;
      assert(m.commissioning_requests.push(envelope));
      f.advance(commissioning_test_now + 1000000);
      assert(m.commissioning_state.completed_mask == 1);
      assert(m.latest_feedback.coordinate_valid[0] == 1);
      if (m.sampled_bindings[0].command_counts != expected_command)
        std::fprintf(stderr, "independent restore=%u command_counts=%lld want=%d home_command=%d pdo=%d raw=%lld\n",
            unsigned(restore), static_cast<long long>(m.sampled_bindings[0].command_counts),
            expected_command, home_command, m.latest_feedback.coordinate_counts[0],
            static_cast<long long>(envelope.home_sample.encoder_counts));
      assert(m.sampled_bindings[0].command_counts == expected_command);
      assert(m.commissioning_requests.push({{job + 1, identity, CommissioningOperation::Finish}}));
      f.advance(commissioning_test_now + 1000000);
      assert(m.control_policy.home_valid_mask.load() == 1);
    };
    complete(restoring, 10, restoring ? command : 0, command);
    const auto fixed_offset = m.sampled_bindings[0].command_offset;
    pd.write_s32(20, period + 4); set_raw(raw + 7);
    f.advance(commissioning_test_now + 1000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(commissioning_test_now + 1000000);
    assert(m.latest_feedback.coordinate_valid[0] && m.latest_feedback.coordinate_counts[0] == period + 4);
    assert(m.sampled_bindings[0].command_offset == fixed_offset);
    assert(m.execution_fault_reasons.load() == 0 && m.control_policy.home_valid_mask.load() == 1);

    // Full-period reset preserves PDO phase, but must revoke Home and motion.
    set_raw(raw + 7 - period);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(commissioning_test_now + 1000000);
    assert(m.sampled_bindings[0].failed && !m.latest_feedback.coordinate_valid[0]);
    assert(m.execution_fault_reasons.load() == FaultCoordinateReference && m.control_policy.home_valid_mask.load() == 0);
    assert(!m.armed.load() && !m.axis_enable_mask.load());
    // Missing, torn, old-frame/configuration and expired acquisitions cannot
    // make fault recovery available, even with fresh PDO feedback.
    SampledAbsolutePosition reset_proof{};
    assert(m.sampled_absolute[0].read(reset_proof));
    for (unsigned invalid = 0; invalid < 5; ++invalid) {
      auto bad = reset_proof;
      const auto age = profile.coordinate_evidence.maximum_acquisition_age_ns;
      if (invalid == 0) bad.started_ns = 0;
      if (invalid == 1) { bad.valid = 0; bad.reason = uint32_t(SampleInvalidity::Torn); }
      if (invalid == 2) ++bad.configuration_epoch;
      if (invalid == 3) ++bad.home_epoch;
      if (invalid == 4) { bad.started_ns = bad.completed_ns = 1; profile.coordinate_evidence.maximum_acquisition_age_ns = 1; }
      m.sampled_absolute[0].publish(bad);
      assert(m.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 90 + invalid}));
      f.advance(commissioning_test_now + 1000000);
      assert(m.execution_fault_reasons.load() == FaultCoordinateReference);
      assert(!m.latest_feedback.coordinate_valid[0] && m.control_policy.home_valid_mask.load() == 0);
      profile.coordinate_evidence.maximum_acquisition_age_ns = age;
    }
    // Reset only clears the fault when fresh raw evidence is available. It
    // cannot revive the failed binding or grant Home/Enable/Arm.
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    assert(m.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 99}));
    f.advance(commissioning_test_now + 1000000);
    assert(m.execution_fault_reasons.load() == 0 && m.safety_fault_mask.load() == 0);
    assert(m.sampled_bindings[0].failed && !m.latest_feedback.coordinate_valid[0]);
    assert(m.control_policy.home_valid_mask.load() == 0 && !m.armed.load() && !m.axis_enable_mask.load());
    auto enable = MachineState::ControlRequest{MachineState::ControlOperation::Enable, 1};
    enable.generation = m.control_policy.generation.load(); enable.configuration = 1; enable.home = m.home_epoch.load();
    assert(!m.apply_control(enable).accepted);
    // Explicit new Home changes the frame. No automatic re-fit on Reset.
    m.absolute_frame_epoch[0].fetch_add(1);
    pd.write_s32(20, 4);
    f.advance(commissioning_test_now + 1000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    complete(false, 20, 0, 4);
    assert(!m.sampled_bindings[0].failed && m.latest_feedback.coordinate_counts[0] == 4);
  }
}

// Linear HM35 truth is zero, while its disabled settled PDO is 22 counts below
// it. Completing Home must retain that physical displacement;
// the configured 16-count stationary proof bounds movement during acquisition,
// not the distance of the current position from the logical-zero offset.
void independent_home_preserves_actual_position() {
  using namespace rosie::rtcore;
  constexpr int32_t period = 13107200, home_truth = 0;
  constexpr int32_t observed = -22, raw = 50345;
  SampledCycleFixture f(1, observed, false, false);
  auto& m = *f.storage;
  auto& profile = m.axis_profiles[0];
  profile.vendor_id = 4194304; profile.product_code = 1813; profile.home_truth_sign = -1;
  f.options.axis[0].counts_per_unit = period / 6.28318530717958647692;
  m.control_policy.connected.store(true);
  m.control_policy.heartbeat_ns.store(commissioning_test_now);
  f.handles.cycle_backend.axis(0).write_u16(16, 0x40);
  f.master.slaves[0]->raw_multi_turn_encoder_counts.store(raw);
  auto& low = f.master.slaves[0]->sdos[0x204021]; low.resize(4); EC_WRITE_S32(low.data(), raw);
  auto& high = f.master.slaves[0]->sdos[0x204023]; high.resize(4); EC_WRITE_S32(high.data(), 0);
  m.absolute_frame_epoch[0].store(1);
  f.advance(commissioning_test_now + 1000000);
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  f.advance(commissioning_test_now + 1000000);
  assert(!m.latest_feedback.coordinate_valid[0]);
  const CommissioningIdentity identity{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
  m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
  MachineState::CommissioningEnvelope envelope{{42, identity, CommissioningOperation::CompleteHome,
      1, 0, 0, -home_truth}};
  assert(m.sampled_absolute[0].read(envelope.home_sample));
  assert(m.commissioning_requests.push(envelope));
  f.advance(commissioning_test_now + 1000000);
  assert(m.commissioning_state.completed_mask == 1);
  MachineState::CommissioningReceipt receipt{};
  assert(m.commissioning_receipts.pop(receipt) && receipt.accepted);
  assert(receipt.home_diagnostic.axis == UINT32_MAX);
  assert(m.latest_feedback.coordinate_valid[0] && m.latest_feedback.coordinate_counts[0] == observed);
  assert(m.latest_feedback.native_home_position_offset[0] == -home_truth);
  const auto logical = m.coordinate_model(0, -home_truth).command_to_logical(m.latest_feedback.coordinate_counts[0]);
  assert(logical.valid && std::abs(logical.logical + 22.0 / f.options.axis[0].counts_per_unit) < 1e-12);
  assert(m.control_policy.home_valid_mask.load() == 1);
  assert(m.commissioning_requests.push({{43, identity, CommissioningOperation::Finish}}));
  f.advance(commissioning_test_now + 1000000);
  assert(m.control_policy.home_valid_mask.load() == 1 && m.sampled_bindings[0].bound);
  assert(m.sampled_bindings[0].command_counts == observed);
  assert(!m.armed.load() && m.axis_enable_mask.load() == 0 && !m.execution_fault_reasons.load());
}

void retained_home_recovers_moved_position() {
  // Recorded-like J5: normal movement since Home with exact continuous
  // command/encoder displacement agreement. Startup recovers without Home.
  // Case 10 is the the 1430 cell cabinet power cycle: A6 firmware cleared bit 15/12
  // (statusword 0x1650) while encoder, PDO and 607C still agree.
  using Refusal = rosie::rtcore::RetainedHomeRefusal;
  for (unsigned refusal = 0; refusal < 17; ++refusal) {
    const bool accepted = refusal == 0 || refusal == 10;
    constexpr int32_t period = 13107200, observed = 6570411, raw = 106625;
    SampledCycleFixture f(1, observed, false, false);
    auto& m = *f.storage;
    auto& profile = m.axis_profiles[0];
    profile.vendor_id = 4194304; profile.product_code = 1813; profile.home_truth_sign = -1;
    // Case 1: a drive profile declaring bit-15 retention still requires it.
    profile.retained_home_requires_homing_attained = refusal == 1;
    if (refusal == 16) { profile.limits.min_q = -100; profile.limits.max_q = -50; }
    f.options.axis[0].counts_per_unit = period / 6.28318530717958647692;
    f.handles.cycle_backend.axis(0).write_u16(16, refusal == 1 || refusal == 10 ? 0x1650 : 0x9650);
    f.master.slaves[0]->raw_multi_turn_encoder_counts.store(raw);
    auto& low = f.master.slaves[0]->sdos[0x204021]; low.resize(4); EC_WRITE_S32(low.data(), raw);
    auto& high = f.master.slaves[0]->sdos[0x204023]; high.resize(4); EC_WRITE_S32(high.data(), 0);
    m.absolute_frame_epoch[0].store(1);
    f.advance(commissioning_test_now + 1000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(commissioning_test_now + 1000000);
    MachineState::HomeTruthReadback r{m.control_policy.generation.load(),
        m.configuration_epoch.load(), m.home_epoch.load(), 55, 0, 0};
    r.audit_generation = m.audit_generation.load();
    r.retained_anchor_valid = true;
    r.anchor.version = 3; r.anchor.home_epoch = 2; r.anchor.counts = 6553601; r.anchor.turns = 89815;
    r.anchor.offset = 0; r.anchor.tolerance_counts = 16;
    r.alarm_objects_read = true;
    assert(m.sampled_absolute[0].read(r.sample));
    if (refusal == 11) r.alarm_objects_read = false; // failed U41.05/C10.15 upload
    if (refusal == 12) r.encoder_fault_state = 0x4000; // U41.05 bit 14 battery fault
    if (refusal == 13) r.multi_turn_overflow = 1;      // C10.15
    if (refusal == 14) r.encoder_fault_state = 0x2000; // U41.05 bit 13 multi-turn fault
    if (refusal == 15) r.anchor.counts += 200;         // sub-revolution PDO disagreement
    if (refusal == 2) r.offset++;
    if (refusal == 3) r.anchor.turns += 1000;
    if (refusal == 4) r.configuration_epoch++;
    if (refusal == 5) r.retained_anchor_valid = false; // drive identity still loading
    if (refusal == 6) { r.sample.valid = false; r.sample.reason = uint32_t(rosie::rtcore::SampleInvalidity::Torn); }
    if (refusal == 7) --r.audit_generation;
    if (refusal == 9) { r.retained_anchor_valid = false; r.calibration_reason = 2; }
    if (refusal == 8) {
      f.handles.cycle_backend.axis(0).write_s32(20, observed + 51);
      f.advance(commissioning_test_now + 1000000);
      f.handles.cycle_backend.axis(0).write_s32(20, observed);
    }
    assert(m.home_truth_readbacks.push(r));
    f.advance(commissioning_test_now + 1000000);
    if (m.control_policy.home_valid_mask.load() != (accepted ? 1u : 0u))
      std::fprintf(stderr, "retained refusal=%u reason=%s bind=%u processed=%llu bound=%u failed=%u fault=%u\n",
          refusal, rosie::rtcore::retained_home_refusal_name(Refusal(m.retained_home_refusal[0].load())),
          unsigned(m.sampled_bindings[0].last_bind_refusal),
          (unsigned long long)m.home_truth_processed[0].load(), unsigned(m.sampled_bindings[0].bound),
          unsigned(m.sampled_bindings[0].failed), m.execution_fault_reasons.load());
    assert(m.control_policy.home_valid_mask.load() == (accepted ? 1u : 0u));
    const auto reason = Refusal(m.retained_home_refusal[0].load());
    const std::array<std::pair<unsigned, Refusal>, 10> reasons{{{1, Refusal::HomingAttained},
        {2, Refusal::OffsetChanged}, {3, Refusal::Disagreement}, {8, Refusal::Unstationary},
        {11, Refusal::AlarmUnread}, {12, Refusal::EncoderAlarm}, {13, Refusal::MultiTurnOverflow},
        {14, Refusal::EncoderAlarm}, {15, Refusal::Disagreement}, {16, Refusal::HardLimit}}};
    for (const auto& [index, expected] : reasons) if (index == refusal) assert(reason == expected);
    if (accepted) assert(reason == Refusal::None);
    if (!accepted) assert(!m.sampled_bindings[0].bound && !m.restored_anchor_home_epoch[0].load());
    if (refusal == 2 || refusal == 9) assert(!m.calibration_valid_mask.load());
    if (refusal == 2) assert(m.recovery_position_reason[0] == rosie::rtcore::CoordinateReason::OffsetChanged);
    if (accepted) {
      assert(m.sampled_bindings[0].bound);
      assert(m.sampled_bindings[0].command_counts == observed);
      assert(m.latest_feedback.native_home_position_offset[0] == 0);
      assert(m.restored_anchor_home_epoch[0].load() == 2);
      assert(m.calibration_valid_mask.load() == 1);
      const auto original_home_epoch = m.home_epoch.load();
      const auto original_position_epoch = m.axis_coordinate_epoch[0];
      auto pd = f.handles.cycle_backend.axis(0);
      pd.write_u16(28, 0x7305); pd.write_u16(16, 0x9658);
      f.advance(commissioning_test_now + 1000000);
      assert(m.calibration_valid_mask.load() == 1 && !m.control_policy.home_valid_mask.load());
      assert(m.execution_fault_reasons.load() & rosie::rtcore::FaultCoordinateReference);
      assert(m.axis_coordinate_epoch[0] > original_position_epoch);
      assert(m.home_epoch.load() == original_home_epoch);
      pd.write_u16(28, 0); pd.write_u16(16, 0x9650);
      f.advance(commissioning_test_now + 1000000);
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
      f.advance(commissioning_test_now + 1000000);
      r.ticket++; r.generation = m.control_policy.generation.load();
      r.configuration_epoch = m.configuration_epoch.load(); r.home_epoch = m.home_epoch.load();
      r.audit_generation = m.audit_generation.load();
      assert(m.sampled_absolute[0].read(r.sample));
      assert(m.home_truth_readbacks.push(r));
      f.advance(commissioning_test_now + 1000000);
      assert(m.control_policy.home_valid_mask.load() == 1 && m.latest_feedback.coordinate_valid[0]);
      assert(m.home_epoch.load() == original_home_epoch);
      assert(m.execution_fault_reasons.load() & rosie::rtcore::FaultCoordinateReference);
      assert(!m.armed.load() && !m.axis_enable_mask.load() && !m.active_trajectory);
    }
    assert(!m.armed.load() && !m.axis_enable_mask.load());
    assert(m.native_home_active_axis_mask.load() == 0);
    if (refusal == 5) {
      assert(m.home_truth_applied[0].load() != r.ticket);
      r.ticket++; r.retained_anchor_valid = true;
      assert(m.home_truth_readbacks.push(r));
      f.advance(commissioning_test_now + 1000000);
      assert(m.control_policy.home_valid_mask.load() == 1);
    }
  }
}

// A rosie-anchor-v5 file is a removed current-pose zero capture. Startup,
// RestoreAnchor and verified-delta restore all load through load_restore_anchor,
// which must refuse it as a source failure naming the reason, leave the file
// untouched, and never reach an enable, Home or drive write.
void current_reference_anchor_file_refused() {
  const auto directory = std::filesystem::temp_directory_path() /
      ("rosie-v5-anchor-" + std::to_string(::getpid()));
  std::filesystem::create_directories(directory);
  const std::string payload = "rosie-anchor-v5 " + std::string(64, 'a') +
      " 0 4194304 1813 1 0 0 4 6553601 -6553600 89836 13107200 16 " + std::string(32, 'b') +
      " 100 616e63686f72 70616972 1 independent_absolute_current_pose_v1 91 3 4 5 1 6553601 89836"
      " -6553601 6553600 100 200 300\n";
  uint32_t crc = 0xffffffffu;
  for (unsigned char byte : payload) {
    crc ^= byte;
    for (unsigned bit = 0; bit < 8; ++bit) crc = (crc >> 1) ^ ((crc & 1) ? 0xedb88320u : 0u);
  }
  char trailer[16];
  std::snprintf(trailer, sizeof(trailer), "%08x\n", ~crc);
  const auto path = directory / "axis-0.anchor";
  { std::ofstream file(path, std::ios::binary); file << payload << trailer; }
  const auto original = std::filesystem::file_size(path);
  ::setenv("ROSIE_RT_ANCHOR_DIR", directory.c_str(), 1);
  SampledCycleFixture f(1, 6570411, false, false);
  auto& m = *f.storage;
  MachineState::RestoreAnchorEvidence evidence{};
  std::string field;
  assert(m.load_restore_anchor(0, evidence, &field) == 3);
  assert(field == "current_reference_anchor" && evidence.version == 0);
  assert(std::string(rosie::rtcore::retained_home_refusal_name(
      rosie::rtcore::RetainedHomeRefusal::CurrentReferenceAnchor)) == "current_reference_anchor_rejected");
  assert(std::filesystem::file_size(path) == original);
  assert(!m.control_policy.home_valid_mask.load() && !m.armed.load() && !m.axis_enable_mask.load());
  ::unsetenv("ROSIE_RT_ANCHOR_DIR");
  std::filesystem::remove_all(directory);
}

// Replay the 106 fresh PDO observations from J2's rejected Home acquisition.
// Its 18-count full span stays within 15 counts of the witnessed start anchor.
void independent_home_measured_j2_acquisition() {
  using namespace rosie::rtcore;
  constexpr int32_t period = 13107200, home = period / 2;
  const std::array<int32_t, 106> measured{6553634, 6553632, 6553634, 6553634, 6553634, 6553635, 6553635, 6553634, 6553635, 6553634, 6553635, 6553636, 6553634, 6553634, 6553635, 6553635, 6553635, 6553634, 6553635, 6553635, 6553635, 6553635, 6553635, 6553637, 6553636, 6553636, 6553635, 6553636, 6553635, 6553636, 6553635, 6553635, 6553637, 6553635, 6553636, 6553636, 6553636, 6553636, 6553637, 6553636, 6553636, 6553636, 6553635, 6553637, 6553636, 6553636, 6553636, 6553636, 6553636, 6553636, 6553636, 6553636, 6553636, 6553636, 6553634, 6553632, 6553625, 6553623, 6553619, 6553621, 6553623, 6553624, 6553623, 6553623, 6553623, 6553622, 6553623, 6553622, 6553622, 6553622, 6553622, 6553623, 6553622, 6553622, 6553622, 6553623, 6553622, 6553623, 6553623, 6553624, 6553622, 6553623, 6553623, 6553622, 6553622, 6553623, 6553622, 6553622, 6553622, 6553623, 6553622, 6553622, 6553622, 6553624, 6553622, 6553623, 6553623, 6553624, 6553621, 6553622, 6553623, 6553623, 6553622, 6553623, 6553622, 6553622};
  for (bool excessive : {false, true}) {
    SampledCycleFixture f(1, measured.front(), false, false);
    auto& m = *f.storage;
    auto& profile = m.axis_profiles[0];
    profile.vendor_id = 4194304; profile.product_code = 1813; profile.home_truth_sign = -1;
    f.options.axis[0].counts_per_unit = period / 6.28318530717958647692;
    m.control_policy.connected.store(true); m.control_policy.heartbeat_ns.store(1);
    m.control_policy.lease_ns = UINT64_MAX;
    auto pd = f.handles.cycle_backend.axis(0); pd.write_u16(16, 0x40);
    f.master.slaves[0]->raw_multi_turn_encoder_counts.store(66731);
    auto& low = f.master.slaves[0]->sdos[0x204021]; low.resize(4); EC_WRITE_S32(low.data(), 66731);
    auto& high = f.master.slaves[0]->sdos[0x204023]; high.resize(4); EC_WRITE_S32(high.data(), 0);
    m.absolute_frame_epoch[0].store(1);
    f.advance(commissioning_test_now + 1000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    const CommissioningIdentity identity{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
    m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
    MachineState::CommissioningEnvelope envelope{{42, identity, CommissioningOperation::CompleteHome, 1, 0, 0, -home}};
    assert(m.sampled_absolute[0].read(envelope.home_sample));
    for (size_t i = 1; i < measured.size(); ++i) {
      pd.write_s32(20, excessive && i == 58 ? measured.front() - 51 : measured[i]);
      f.advance(commissioning_test_now + 1000000);
    }
    envelope.home_sample.completed_ns = commissioning_test_now;
    envelope.home_sample.low_completed_ns = 0;
    envelope.home_sample.bracket_completed_ns = 0;
    envelope.home_sample.low_cycle = 0;
    assert(m.commissioning_requests.push(envelope));
    f.advance(commissioning_test_now + 1000000);
    MachineState::CommissioningReceipt receipt{};
    assert(m.commissioning_receipts.pop(receipt));
    assert(receipt.accepted == !excessive);
    assert(m.control_policy.home_valid_mask.load() == (excessive ? 0u : 1u));
    if (excessive) {
      assert(receipt.home_diagnostic.bind_refusal == IndependentBindRefusal::StationaryWindow);
      assert(receipt.home_diagnostic.interval_valid);
      assert(receipt.home_diagnostic.interval_min == measured.front() - 51);
      assert(receipt.home_diagnostic.anchor_counts == measured.front());
      assert(receipt.home_diagnostic.displacement_counts == 51);
      assert(receipt.home_diagnostic.displacement_limit == 50);
      assert(!m.sampled_bindings[0].bound);
    } else {
      assert(m.latest_feedback.coordinate_valid[0]);
      assert(m.latest_feedback.coordinate_counts[0] == measured.back());
      assert(m.latest_feedback.native_home_position_offset[0] == -home);
      assert(m.commissioning_requests.push({{43, identity, CommissioningOperation::Finish}}));
      f.advance(commissioning_test_now + 1000000);
      assert(m.control_policy.home_valid_mask.load() == 1);
    }
    assert(!m.armed.load() && m.axis_enable_mask.load() == 0);
  }
}

void home_capture_waits_for_disabled_settling() {
  using namespace rosie::rtcore;
  using namespace gradient::ipc::v1;
  // Measured the 1430 cell shape: torque drops after service release; the brake timer
  // reaches APPLIED and PDO position shifts113counts inside the old acquisition.
  // The pre-capture wait must observe a new quiet window after that shift.
  for (unsigned injection = 0; injection < 10; ++injection) {
    SampledCycleFixture f(1, 0, false, false);
    auto& m = *f.storage;
    m.commissioning_running.store(true);
    m.control_policy.connected.store(true);
    m.control_policy.heartbeat_ns.store(1);
    m.control_policy.lease_ns = UINT64_MAX;
    m.axis_profiles[0].brake.present = true;
    m.axis_profiles[0].brake.hold_displacement_tolerance_counts = 16;
    m.bus_ready.store(true);
    const CommissioningIdentity identity{42, m.control_policy.generation.load(),
        m.configuration_epoch.load(), m.home_epoch.load()};
    const uint64_t start = 1000000000;
    uint64_t now = start;
    unsigned tick = 0;
    auto publish = [&] {
      m.latest_feedback.seq.store(2 * (tick + 1));
      m.latest_feedback.pdo_observed_time_ns[0] = now;
      m.latest_feedback.pdo_fresh_axis_mask = 1;
      m.latest_feedback.statusword[0] = 0x40;
      m.latest_feedback.brake_state[0] = tick < 50 ? BRAKE_WAIT_HOLD_DELAY : BRAKE_APPLIED;
      m.latest_feedback.pos_counts[0] = 6553587 + (tick >= 53 ? 113 : 0) + (tick % 3);
    };
    publish();
    const bool accepted = wait_native_home_capture_settle(m, 0, identity, start + 500000000,
        [&] { return now; }, [&] {
          ++tick; now += 1000000;
          if (injection != 1) publish(); // frozen snapshot cannot become quiet
          if (injection == 9 && tick % 7 == 0) m.latest_feedback.seq.fetch_add(1);
          if (injection == 2) m.latest_feedback.pos_counts[0] = 6553600 + (tick % 2) * 51;
          if (tick >= 60) {
            if (injection == 3) m.control_policy.generation.fetch_add(1);
            if (injection == 4) m.control_policy.connected.store(false);
            if (injection == 5) m.armed.store(true);
            if (injection == 6) m.service_mode_axis_mask.store(1);
            if (injection == 7) m.absolute_frame_epoch[0].fetch_add(1);
            if (injection == 8) m.latest_feedback.pdo_fresh_axis_mask = 0;
          }
        });
    assert(accepted == (injection == 0 || injection == 9));
    if (accepted) assert(tick >= 153 && tick < 500);
    assert(m.control_policy.home_valid_mask.load() == 0);
  }
}

void home_and_restore_wait_for_post_acquisition_pdo() {
  using namespace rosie::rtcore;
  for (bool restoring : {false, true}) {
  for (bool guarded : {false, true}) {
    SampledCycleFixture f(1, 0, false, false);
    auto& m = *f.storage;
    m.axis_profiles[0].vendor_id = 4194304;
    m.axis_profiles[0].product_code = 1813;
    m.axis_profiles[0].home_truth_sign = 1;
    m.commissioning_running.store(true);
    m.control_policy.connected.store(true);
    m.control_policy.heartbeat_ns.store(1);
    // Clock-injected fixture only: keep transport authority through the test.
    m.control_policy.lease_ns = UINT64_MAX;
    f.handles.cycle_backend.axis(0).write_u16(16, 0x40);
    for (auto sub : {0x21, 0x23}) f.master.slaves[0]->sdos[(0x2040u << 8) | sub] = std::vector<uint8_t>(4);
    f.advance(1001000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    const CommissioningIdentity identity{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
    m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
    MachineState::CommissioningEnvelope envelope{{42, identity, CommissioningOperation::CompleteHome, 1}};
    envelope.restoring = restoring;
    envelope.anchor.tolerance_counts = 16;
    assert(m.sampled_absolute[0].read(envelope.home_sample));
    // Acquisition finishes after the next PDO receipt, before its command
    // queue consumption. The unguarded request must still reject future proof.
    envelope.home_sample.completed_ns = 1002500000;
    uint64_t worker_now = envelope.home_sample.completed_ns;
    unsigned pauses = 0;
    if (guarded) {
      assert(wait_home_sample_receipt(m, envelope.request, 0, envelope.home_sample,
          worker_now + 250000000, [&] { return worker_now; }, [&] {
            ++pauses;
            f.advance(1003000000);
            worker_now = 1003000000;
          }));
      assert(pauses == 1 && m.commissioning_state.completed_mask == 0);
    }
    assert(m.commissioning_requests.push(envelope));
    f.advance(guarded ? 1004000000 : 1002000000);
    assert(m.commissioning_state.completed_mask == (guarded ? 1u : 0u));
    assert(m.sampled_bindings[0].bound == guarded);
    assert(m.axis_enable_mask.load() == 0 && !m.armed.load());
    MachineState::CommissioningReceipt receipt{};
    assert(m.commissioning_receipts.pop(receipt) && receipt.accepted == guarded);
    if (!restoring && !guarded) {
      const auto& diagnostic = receipt.home_diagnostic;
      assert(diagnostic.axis == 0);
      assert(diagnostic.bind_refusal == IndependentBindRefusal::FutureSample);
      assert(diagnostic.sample_completed_ns == envelope.home_sample.completed_ns);
      assert(diagnostic.receipt_ns < diagnostic.sample_completed_ns);
      assert(!(diagnostic.qualification_mask & 1u));
    }
    if (!guarded) continue;
    // The worker wait remains bounded and cancellation cannot dispatch a Home.
    envelope.home_sample.completed_ns = worker_now + 10000000;
    assert(!wait_home_sample_receipt(m, envelope.request, 0, envelope.home_sample,
        worker_now + 2000000, [&] { return worker_now; }, [&] { worker_now += 1000000; }));
    // Configuration, coordinate frame and output permission may change while
    // waiting. None may let the old completion reach the cyclic owner.
    auto refused = [&] {
      assert(!wait_home_sample_receipt(m, envelope.request, 0, envelope.home_sample,
          worker_now + 250000000, [&] { return worker_now; }, [] { assert(false); }));
    };
    m.configuration_epoch.fetch_add(1); refused(); m.configuration_epoch.fetch_sub(1);
    m.absolute_frame_epoch[0].fetch_add(1); refused(); m.absolute_frame_epoch[0].fetch_sub(1);
    m.armed.store(true); refused(); m.armed.store(false);
    m.axis_enable_mask.store(1); refused(); m.axis_enable_mask.store(0);
    m.control_policy.generation.fetch_add(1); refused();
  }
  }
}

// Complete exchanges with one PREOP slave cannot establish startup_ready.
void runtime_startup_requires_op() {
  SampledCycleFixture f(9, 0, false, false);
  auto& m = *f.storage;
  m.sc_state[8] = {1, 0, 2};
  f.advance(1001000000);
  assert(!m.startup_ready && !m.bus_ready.load() && m.mailbox_ready_since_ns.load() == 0);
  m.sc_state[8] = {1, 1, 8};
  f.advance(1002000000);
  assert(m.startup_ready && m.mailbox_ready_since_ns.load() == 1002000000);
}

// Drive each formerly silent failure through the actual native Home method,
// then the cyclic failure channel. One cause must survive release/cancellation
// and identify the configured step without publishing successful Home.
void native_home_failure_diagnostics() {
  using namespace rosie::rtcore;
  using Operation = CommissioningOperation;
  struct Case { unsigned injection; uint32_t code; const char* location; const char* reason; };
  for (const auto& tc : {
      Case{0, 4043309058u, "step=0 op=refresh_truth", "frame epoch exhausted"},
      Case{1, 4043309059u, "step=0 op=refresh_truth", "unavailable"},
      Case{2, 4043309064u, "step=0 op=refresh_truth", "truth refresh failed"},
      Case{3, 4043309065u, "step=1 op=final_truth", "final truth refresh failed"},
      Case{4, 4043309060u, "step=0 op=refresh_truth", "permission lost"},
      Case{5, 4043309061u, "step=1 op=release_service_override", "cancelled"},
      Case{6, 4043309063u, "step=1 op=final_absolute_sample", "unavailable"},
      Case{7, 4043309062u, "step=1 op=CompleteHome", "not_ready"},
      Case{8, 4043309059u, "step=0 op=refresh_truth", "torn"},
      Case{9, 4043309068u, "step=0 op=begin", "not_ready"},
      Case{10, 4043309060u, "step=0 op=begin", "permission lost"}}) {
    SampledCycleFixture f(1, 0, false, false);
    auto& m = *f.storage;
    auto& home = m.axis_profiles[0].home;
    home.valid = home.truth_source.valid = true;
    home.truth_source = {true, 0x607c, 0, SdoScalarType::kI32};
    home.steady_state_mode = 8; home.commissioning_mode = 6;
    NativeHomeOp op{};
    op.kind = tc.injection == 3 ? NativeHomeOpKind::kRestoreMode : NativeHomeOpKind::kRefreshTruth;
    op.value_i32 = 8;
    home.transaction = {op};
    m.axis_profiles[0].home_truth_sign = tc.injection == 2 || tc.injection == 3 ? 0 : 1;
    auto& truth = f.master.slaves[0]->sdos[0x607cu << 8];
    truth.resize(4); EC_WRITE_S32(truth.data(), 65536);
    m.commissioning_running.store(true);
    m.control_policy.connected.store(tc.injection != 10);
    m.control_policy.heartbeat_ns.store(now_monotonic_ns());
    m.absolute_last_poll_ns[0] = now_monotonic_ns();
    if (tc.injection == 0) m.absolute_frame_epoch[0].store(UINT64_MAX);
    if (tc.injection == 1) f.master.faults.sample_unavailable_from_upload = 1;
    if (tc.injection == 8) f.master.faults.sample_zero_crossing_low_read = 1;
    const CommissioningIdentity identity{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
    m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
    uint32_t failures = 0, completions = 0;
    auto request = [&](CommissioningRequest req, CommissioningIdentity* = nullptr, bool* = nullptr,
                       CommissioningReason* reason = nullptr) {
      m.control_policy.heartbeat_ns.store(now_monotonic_ns());
      if (reason) *reason = CommissioningReason::None;
      if (req.operation == Operation::FailHome) {
        ++failures;
        assert(req.abort_code == tc.code && req.axis_mask == 1);
        req.ticket = 1;
        assert(m.commissioning_requests.push({req}));
        return true;
      }
      if (req.operation == Operation::Service) {
        if (tc.injection == 9) { *reason = CommissioningReason::NotReady; return false; }
        if (tc.injection == 4) m.control_policy.connected.store(false);
      }
      if (req.operation == Operation::Release) {
        if (tc.injection == 5) { *reason = CommissioningReason::Cancelled; return false; }
        if (tc.injection == 6) f.master.faults.sample_unavailable_from_upload = 1;
      }
      if (req.operation == Operation::CompleteHome) {
        ++completions;
        assert(tc.injection == 7);
        *reason = CommissioningReason::NotReady;
        return false;
      }
      return true;
    };
    FILE* log = std::fopen("build/native-home-failure.log", "w+");
    assert(log);
    std::fflush(stderr);
    const int saved_stderr = dup(STDERR_FILENO);
    assert(saved_stderr >= 0 && dup2(fileno(log), STDERR_FILENO) >= 0);
    assert(!m.native_home_axis(0, identity, request));
    std::fflush(stderr);
    assert(dup2(saved_stderr, STDERR_FILENO) >= 0); close(saved_stderr);
    std::rewind(log);
    std::string diagnostic;
    char line[1024];
    while (std::fgets(line, sizeof(line), log)) diagnostic += line;
    std::fclose(log);
    const auto failure = diagnostic.find("native_home failed axis=0");
    assert(failure != std::string::npos);
    assert(diagnostic.find("native_home failed axis=0", failure + 1) == std::string::npos);
    assert(diagnostic.find(tc.location) != std::string::npos);
    assert(diagnostic.find(std::string("reason=") + tc.reason) != std::string::npos);
    assert(failures == 1 && completions == (tc.injection == 7 ? 1u : 0u));
    m.control_policy.heartbeat_ns.store(commissioning_test_now);
    f.advance(1001000000);
    assert(m.latest_feedback.native_home_last_abort_code[0] == tc.code);
    assert(m.latest_feedback.native_home_state[0] == gradient::ipc::v1::NATIVE_HOME_STATE_FAILED);
    assert(m.control_policy.home_valid_mask.load() == 0 && m.service_mode_axis_mask.load() == 0);
    assert(m.commissioning_state.phase == CommissioningPhase::Draining);
  }
}

// GradientOS HM35 transaction for the A6-EC profile, exactly as rtctl compiles
// config/drives/a6ec.json on origin/main (014b12c2). Vendor-confirmed sequence;
// do not modify (see rt-core/AGENTS.md). tools/rtctl
// TestA6ecHomeTransactionMatchesVendorGolden pins a6ec.json to this string.
constexpr const char kVendorHm35Transaction[] =
    "steady_state_mode|8;commissioning_mode|6;truth_source|24700|0|i32;"
    "op|set_mode|6;op|write_sdo|24806|0|u8|0;op|write_sdo|24700|0|i32|0;"
    "op|write_sdo|24728|0|i8|35;op|controlword_sequence|6,7,15;op|wait_statusword|551|8264;"
    "op|controlword_sequence|31;op|wait_statusword|36864|8192;op|refresh_truth;"
    "op|restore_mode|8;op|release_service_override";

// The worker submits that transaction and nothing else: no steady-mode
// Shutdown dwell, CSP precondition, pretrigger dwell or extra object write.
// Begin publishes mode 6 with CW0 and waits 50 ms, as on origin/main.
void native_home_executes_vendor_hm35_transaction_exactly() {
  using namespace rosie::rtcore;
  using Operation = CommissioningOperation;
  using Clock = std::chrono::steady_clock;
  SampledCycleFixture f(1, 0, false, false);
  auto& m = *f.storage;
  auto& home = m.axis_profiles[0].home;
  assert(parse_native_home_config_spec(kVendorHm35Transaction, &home) && home.valid);
  assert(home.steady_state_mode == 8 && home.commissioning_mode == 6 && home.transaction.size() == 11);
  m.axis_profiles[0].home_truth_sign = 1;
  m.axis_profiles[0].anchor_source =
      {0x2040, 0x21, 0x23, "a6ec_raw_multi_turn", "raw_multi_turn_encoder", 3};
  for (const auto object : {0x204021u, 0x204023u})
    f.master.slaves[0]->sdos[object].assign(4, 0);
  m.commissioning_running.store(true);
  m.control_policy.connected.store(true);
  m.control_policy.heartbeat_ns.store(now_monotonic_ns());
  m.absolute_last_poll_ns[0] = now_monotonic_ns();
  const CommissioningIdentity identity{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
  m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
  std::vector<std::string> events;
  std::vector<Clock::time_point> stamps;
  const auto record = [&](std::string event) { events.push_back(std::move(event)); stamps.push_back(Clock::now()); };
  auto request = [&](CommissioningRequest req, CommissioningIdentity* = nullptr, bool* = nullptr,
                     CommissioningReason* reason = nullptr) {
    m.control_policy.heartbeat_ns.store(now_monotonic_ns());
    if (reason) *reason = CommissioningReason::None;
    assert(req.axis_mask == 1);
    switch (req.operation) {
      case Operation::Service:
        record("service mode=" + std::to_string(req.mode) + " cw=" + std::to_string(req.controlword));
        // Drive model: CW15 reaches Operation Enabled; CW31 attains Home.
        if (req.controlword == 15) m.latest_feedback.statusword[0] = 0x0237;
        if (req.controlword == 31) m.latest_feedback.statusword[0] = 0x9237;
        break;
      case Operation::Release: record("release"); break;
      case Operation::CompleteHome: record("complete_home"); break;
      case Operation::FailHome: record("fail_home"); break;
      default: record("operation=" + std::to_string(static_cast<uint32_t>(req.operation))); break;
    }
    return true;
  };
  worker_drive_events = &events;
  const bool homed = m.native_home_axis(0, identity, request);
  worker_drive_events = nullptr;
  const std::vector<std::string> golden = {
      "service mode=6 cw=0",  // begin: commissioning mode, CW0, then 50 ms
      "service mode=6 cw=0",  // set_mode 6
      "sdo 0x60e6:0=0",
      "sdo 0x607c:0=0",
      "sdo 0x6098:0=35",
      "service mode=6 cw=6", "service mode=6 cw=7", "service mode=6 cw=15",
      "service mode=6 cw=31",
      "service mode=8 cw=31",  // restore_mode 8
      "service mode=8 cw=0",   // release_service_override restores steady mode
      "release", "complete_home"};
  for (size_t i = 0; i < events.size(); ++i) std::fprintf(stderr, "hm35 event %zu: %s\n", i, events[i].c_str());
  assert(homed);
  assert(events == golden);
  // CW15 to CW31 is the 50 ms step settle plus the statusword match; a
  // pretrigger dwell (>=500 ms) would exceed this bound.
  const auto gap_ms = [&](size_t from, size_t to) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(stamps[to] - stamps[from]).count();
  };
  assert(gap_ms(0, 1) >= 50);
  assert(gap_ms(7, 8) >= 50 && gap_ms(7, 8) < 400);
  std::puts("PASS: native Home executes the vendor-confirmed HM35 transaction exactly");
}

// A Home frame change reaches the early cyclic brake guard before coordinate
// reconciliation. Movement inside the 100 ms energized hold window is only
// recorded; after torque off a cumulative 17 counts must still be detected,
// as a warning that latches no fault.
void native_home_cyclic_brake_anchor() {
  using namespace rosie::rtcore;
  SampledCycleFixture f(1, 65536, false, false);
  auto& m = *f.storage;
  m.axis_profiles[0].coordinate_evidence = {};
  m.axis_profiles[0].brake = {true, 0, 100000000, false, false, 16};
  auto pd = f.handles.cycle_backend.axis(0);
  f.advance(1001000000);
  pd.write_u16(16, 0x40);
  pd.write_s32(20, 100);
  m.absolute_frame_epoch[0].store(1);
  f.advance(1002000000);
  assert(m.execution_fault_reasons.load() == 0 && m.brake_guards[0].anchor == 100);
  m.service_mode_axis_mask.store(1);
  pd.write_s32(20, 65536);
  f.advance(1003000000);
  assert(m.execution_fault_reasons.load() == 0 && m.brake_guards[0].anchor == 65536);
  pd.write_s32(20, 65552);
  f.advance(1004000000);
  assert(m.execution_fault_reasons.load() == 0);
  BrakeHoldEvidence evidence{};
  assert(!m.brake_hold_evidence[0].read(evidence));
  pd.write_s32(20, 65553);
  f.advance(1005000000);
  assert(!m.brake_hold_evidence[0].read(evidence)); // Energized: not judged.
  f.advance(1103000000);
  assert(m.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence == 0);
  assert(evidence.torque_off_anchor_counts == 65553 && evidence.energized_peak_displacement_counts == 17);
  pd.write_s32(20, 65569);
  f.advance(1104000000);
  pd.write_s32(20, 65570);
  f.advance(1105000000);
  assert(m.execution_fault_reasons.load() == 0 && m.safety_fault_mask.load() == 0);
  assert(m.brake_hold_evidence[0].read(evidence) && evidence.warning_sequence == 1);
  assert(evidence.anchor_counts == 65553 && evidence.displacement_counts == 17);
  assert(evidence.torque_off_anchor_counts == 65553 && !evidence.slip && evidence.same_direction_count == 1);
}

// Qualified axes commit independently within a real cyclic HomeAll reservation.
// Failed qualification, explicit failure and authority cancellation cannot undo
// an earlier axis, nor may an obsolete CompleteHome resurrect an unfinished one.
void native_home_partial_success_survives_group_failure() {
  using namespace rosie::rtcore;
  constexpr int32_t period = 13107200, zero = 0;
  for (unsigned failure = 0; failure < 3; ++failure) {
    SampledCycleFixture f(2, zero + 22, false, false);
    auto& m = *f.storage;
    m.control_policy.connected.store(true); m.control_policy.heartbeat_ns.store(1);
    m.control_policy.lease_ns = UINT64_MAX;
    for (unsigned axis = 0; axis < 2; ++axis) {
      auto& profile = m.axis_profiles[axis];
      profile.vendor_id = 4194304; profile.product_code = 1813; profile.home_truth_sign = -1;
      f.options.axis[axis].counts_per_unit = period / 6.28318530717958647692;
      f.handles.cycle_backend.axis(axis).write_u16(16, 0x40);
      f.master.slaves[axis]->raw_multi_turn_encoder_counts.store(66731);
      auto& low = f.master.slaves[axis]->sdos[0x204021]; low.resize(4); EC_WRITE_S32(low.data(), 66731);
      auto& high = f.master.slaves[axis]->sdos[0x204023]; high.resize(4); EC_WRITE_S32(high.data(), 0);
      m.absolute_frame_epoch[axis].store(1);
    }
    uint64_t ticket = 1;
    auto transact = [&](MachineState::CommissioningEnvelope envelope) {
      envelope.request.ticket = ticket++;
      assert(m.commissioning_requests.push(envelope));
      f.advance(commissioning_test_now + 1000000);
      MachineState::CommissioningReceipt receipt{};
      assert(m.commissioning_receipts.pop(receipt));
      return receipt;
    };
    const CommissioningIdentity initial{42, m.control_policy.generation.load(), 1, m.home_epoch.load()};
    auto reserved = transact({{0, initial, CommissioningOperation::Reserve, 3}});
    assert(reserved.accepted && m.control_policy.home_valid_mask.load() == 0);
    const auto identity = reserved.identity;
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    MachineState::CommissioningEnvelope first{{0, identity, CommissioningOperation::CompleteHome, 1, 0, 0, -zero}};
    assert(m.sampled_absolute[0].read(first.home_sample));
    auto completed = transact(first);
    assert(completed.accepted && !completed.job_completed);
    assert(m.control_policy.home_valid_mask.load() == 1 && m.latest_feedback.coordinate_valid[0]);
    const auto first_coordinate = m.coordinate_reference[0];
    assert(m.sample_absolute_axis(1, commissioning_test_clock));
    MachineState::CommissioningEnvelope second{{0, identity, CommissioningOperation::CompleteHome, 2, 0, 0, -zero}};
    assert(m.sampled_absolute[1].read(second.home_sample));
    if (failure == 0) {
      auto invalid = second;
      invalid.home_sample.valid = 0;
      auto refused = transact(invalid);
      assert(!refused.accepted && refused.reason == CommissioningReason::NotReady);
    } else if (failure == 1) {
      auto failed = transact({{0, identity, CommissioningOperation::FailHome, 2, 0, 0, 0, 123}});
      assert(failed.accepted);
    } else {
      m.control_policy.generation.fetch_add(1);
      auto stale = transact(second);
      assert(!stale.accepted && stale.reason == CommissioningReason::Cancelled);
    }
    assert(m.control_policy.home_valid_mask.load() == 1);
    assert(m.coordinate_reference[0].valid && m.coordinate_reference[0].continuous_counts == first_coordinate.continuous_counts);
    auto finished = transact({{0, identity, CommissioningOperation::Finish}});
    assert(finished.accepted && !finished.job_completed);
    assert(m.commissioning_state.phase == CommissioningPhase::Idle);
    assert(m.control_policy.home_valid_mask.load() == 1 && m.sampled_bindings[0].bound);
    auto late = transact(second);
    assert(!late.accepted && late.reason == CommissioningReason::Cancelled);
    assert(m.control_policy.home_valid_mask.load() == 1 && !m.sampled_bindings[1].bound);
    assert(!m.armed.load() && !m.axis_enable_mask.load() && !m.service_mode_axis_mask.load());
  }
}

// Nine masked drives share one reservation: the first failed transaction drains
// all active overrides and refuses subsequent axes until Finish releases the job.
void native_home_group_failure_drain() {
  using namespace rosie::rtcore;
  CommissioningContext context{1, 1, 1, 511, 511, true, true, false, false, false};
  auto reserved = evaluate_commissioning({}, context,
      {1, {42, 1, 1, 1}, CommissioningOperation::Reserve, 511});
  assert(reserved.accepted && reserved.state.identity.home_epoch == 2);
  context.home_epoch = 2;
  auto state = reserved.state;
  for (uint32_t bit : {1u, 2u}) {
    auto service = evaluate_commissioning(state, context,
        {2, state.identity, CommissioningOperation::Service, bit, 6, 15});
    assert(service.accepted);
    state = service.state;
  }
  assert(state.service_mask == 3);
  auto failed = evaluate_commissioning(state, context,
      {3, state.identity, CommissioningOperation::FailHome, 1, 0, 0, 0, 4043309059u});
  assert(failed.accepted && failed.clear_service_mask == 3 && failed.fail_home_mask == 1);
  assert(failed.state.phase == CommissioningPhase::Draining && failed.state.service_mask == 0);
  auto later = evaluate_commissioning(failed.state, context,
      {4, state.identity, CommissioningOperation::Service, 4, 6, 15});
  assert(!later.accepted && later.reason == CommissioningReason::Cancelled);
  auto finish = evaluate_commissioning(failed.state, context,
      {5, state.identity, CommissioningOperation::Finish});
  assert(finish.accepted && !finish.job_completed && finish.state.phase == CommissioningPhase::Idle);
}

void sampled_zero_crossing_retains_evidence() {
  using namespace rosie::rtcore;
  SampledCycleFixture f;
  auto& m = *f.storage;
  f.master.faults.sample_zero_crossing_low_read = 2;
  assert(!m.sample_absolute_axis(0, commissioning_test_clock));
  SampledAbsolutePosition torn{};
  assert(m.sampled_absolute[0].read(torn));
  assert(torn.reason == uint32_t(SampleInvalidity::Torn) && torn.encoder_counts == 4294967295LL);
  f.advance(1002000000);
  assert(m.latest_feedback.coordinate_source_valid[0] == 1);
  assert(m.sampled_at_receipt[0].started_ns == 1000000000);
  assert(m.sampled_bindings[0].metrics.last_invalidity_reason == uint32_t(SampleInvalidity::Torn));
  assert(!m.sampled_bindings[0].failed && m.execution_fault_reasons.load() == 0);
  f.handles.cycle_backend.axis(0).write_s32(20, 0);
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  f.advance(1003000000);
  assert(m.latest_feedback.coordinate_source_valid[0] == 1 && m.latest_feedback.coordinate_counts[0] == 0);
  assert(m.sampled_at_receipt[0].encoder_counts == 0 && m.control_policy.home_valid_mask.load() == 1);
  for (auto reason : {SampleInvalidity::Unavailable, SampleInvalidity::Cadence, SampleInvalidity::Torn}) {
    SampledAbsolutePosition missing{1, 0, m.rt_cycle_counter.load(), commissioning_test_now,
        commissioning_test_now, 0, 0, 0, uint32_t(reason)};
    m.sampled_absolute[0].publish(missing);
    f.advance(commissioning_test_now + 1000000);
    assert(m.latest_feedback.coordinate_source_valid[0] == 1);
    assert(m.sampled_at_receipt[0].started_ns == 1002000000);
  }
  // Unverified stall fixture: 100000000 ns per upload remains inside the
  // configured acquisition interval. Continuous signed counts have no W/2
  // ambiguity, so a stationary coherent bracket replaces the retained audit.
  f.master.faults.sample_upload_delay_ns = 100000000;
  const auto delayed_started = commissioning_test_now;
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  f.advance(commissioning_test_now + 1000000);
  assert(m.latest_feedback.coordinate_source_valid[0] == 1);
  assert(m.sampled_at_receipt[0].started_ns == delayed_started);
  assert(m.sampled_bindings[0].metrics.reason == uint32_t(SampleInvalidity::None));
  assert(m.execution_fault_reasons.load() == 0 && m.control_policy.home_valid_mask.load() == 1);
  f.master.faults.sample_upload_delay_ns = 0;
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  f.advance(commissioning_test_now + 1000000);
  assert(m.latest_feedback.coordinate_source_valid[0] == 1 && m.sampled_bindings[0].metrics.reason == 0);
}

// A stationary audit cannot use an expired or armed-overlapping publication
// to revoke a trustworthy running position.
void sampled_late_audit_is_diagnostic_only() {
  using namespace rosie::rtcore;
  SampledCycleFixture f(1, 500000, false, true, true);
  auto& m = *f.storage;
  SampledAbsolutePosition sample{}; assert(m.sampled_absolute[0].read(sample));
  sample.started_ns++; sample.encoder_counts += 1310720;
  sample.completed_ns = sample.started_ns + 1;
  f.advance(4001000001);
  m.sampled_absolute[0].publish(sample);
  f.advance(4002000001);
  assert(m.control_policy.home_valid_mask.load() == 1 && m.calibration_valid_mask.load() == 1);
  assert(m.latest_feedback.coordinate_valid[0] && !m.sampled_bindings[0].failed);
  assert(m.execution_fault_reasons.load() == 0);
}

void sampled_expiry_preserves_running_position() {
  using namespace rosie::rtcore;
  SampledCycleFixture f(1, -1, false, true, true);
  auto& m = *f.storage;
  f.enabled = true;
  m.control_policy.connected.store(true);
  m.armed.store(true); m.axis_enable_mask.store(1);
  m.desired_mode_of_operation[0].store(8);
  f.advance(1002000000);
  assert(f.last_outputs.motion_permitted_mask == 1);
  SampledAbsolutePosition torn{}; assert(m.sampled_absolute[0].read(torn));
  torn.started_ns++; torn.completed_ns = torn.started_ns + 1;
  torn.valid = 0; torn.reason = uint32_t(SampleInvalidity::Torn);
  m.sampled_absolute[0].publish(torn);
  m.active_jog = true; m.active_jog_axis_mask = 1;
  m.active_jog_velocity_counts_per_s[0] = 0;
  m.active_jog_deadline_ns = 5000000000;
  f.advance(4000000001);
  assert(m.sampled_bindings[0].metrics.age_ns > 3000000000);
  assert(m.latest_feedback.coordinate_valid[0] && m.control_policy.home_valid_mask.load() == 1);
  assert(m.calibration_valid_mask.load() == 1 && m.latest_feedback.position_state[0] == 2);
  assert(m.execution_fault_reasons.load() == 0 && m.armed.load() && m.active_jog);
  assert(f.last_outputs.motion_permitted_mask == 1);
  // A following-error stop changes authority, not the calibration or healthy PDO history.
  MachineState::cycle_owner = &m;
  m.execution_fault(FaultFollowingError, 1);
  MachineState::cycle_owner = nullptr;
  f.advance(4001000001);
  assert(m.calibration_valid_mask.load() == 1 && m.latest_feedback.coordinate_valid[0]);
  assert(m.execution_fault_reasons.load() == FaultFollowingError && !m.armed.load());
}

void sampled_nine_axis_expiry_preserves_position() {
  // Every migrated A6 axis carries a qualified independent origin. Audit age
  // does not revoke continuously tracked PDO coordinates or Home.
  using namespace rosie::rtcore;
  SampledCycleFixture f(9);
  auto& m = *f.storage;
  f.advance(4000000000);
  assert(m.control_policy.home_valid_mask.load() == 511);
  f.advance(4000000001);
  assert(m.execution_fault_reasons.load() == 0);
  assert(m.control_policy.home_valid_mask.load() == 511);
  for (unsigned axis = 0; axis < 9; ++axis)
    assert(m.latest_feedback.coordinate_valid[axis]);
}

void sampled_native_anchor_command_refusal() {
  using namespace rosie::rtcore;
  SampledCycleFixture f;
  auto& m = *f.storage;
  m.control_policy.connected.store(true);
  m.control_policy.heartbeat_ns.store(commissioning_test_now);
  m.control_policy.home_valid_mask.store(0);
  m.home_epoch.store(2);
  const CommissioningIdentity identity{17, m.control_policy.generation.load(), 1, 2};
  m.commissioning_state = {identity, CommissioningPhase::Active, 1, 0, 0};
  MachineState::CommissioningEnvelope envelope{{1, identity, CommissioningOperation::CompleteHome, 1}};
  envelope.restoring = true;
  // Independent encoder -1 matches the saved turns. Only the command frame
  // disagrees: 16 - (-1) = 17 counts, beyond the 16-count native tolerance.
  envelope.anchor.counts = 16; envelope.anchor.turns = -1; envelope.anchor.tolerance_counts = 16;
  assert(m.commissioning_requests.push(envelope));
  f.advance(1002000000);
  assert(m.anchor_reason[0].load() == 4);
  assert(m.control_policy.home_valid_mask.load() == 0);
  assert(m.commissioning_state.completed_mask == 0);
  assert(m.latest_feedback.coordinate_source_valid[0] == 1);
}

// Single-turn absence never inhibits an established frame, including enabled
// hold beyond the standing policy's age and grace bounds. Decidable
// contradictions and encoder faults revoke the frame on their receiving cycle.
void single_turn_stale_readiness_and_drive_guards() {
  using namespace rosie::rtcore;
  for (unsigned guard = 0; guard < 4; ++guard) {
    SampledCycleFixture f(1, 500000, true);
    auto& m = *f.storage;
    f.enabled = true;
    m.control_policy.connected.store(true);
    m.armed.store(true); m.axis_enable_mask.store(1);
    m.desired_mode_of_operation[0].store(8);
    f.advance(9000000000);
    assert(m.sampled_bindings[0].metrics.age_ns == 8000000000);
    assert(m.latest_feedback.coordinate_valid[0] == 1 && m.latest_feedback.coordinate_source_valid[0] == 1);
    assert(m.control_policy.home_valid_mask.load() == 1 && m.axis_coordinate_epoch[0] == 1);
    assert(m.execution_fault_reasons.load() == 0);
    assert(f.last_outputs.motion_ready_mask == 1 && f.last_outputs.motion_permitted_mask == 1);
    f.master.faults.sample_unavailable_from_upload = 4;
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(9001000000);
    assert(m.latest_feedback.coordinate_valid[0] == 1 && f.last_outputs.motion_permitted_mask == 1);
    if (guard == 0) {
      f.master.faults.sample_unavailable_from_upload = 0;
      f.master.faults.sample_contradiction_from_upload = 1;
      f.master.faults.sample_contradiction_counts = 1310720;
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
    } else if (guard == 1) f.handles.cycle_backend.axis(0).write_u16(16, 0x8);
    else f.handles.cycle_backend.axis(0).write_u16(28, guard == 2 ? 1 : 0x7305);
    f.advance(9002000000);
    const bool invalid = guard == 0 || guard == 3;
    assert(m.execution_fault_reasons.load() == (invalid ? FaultCoordinateReference : guard == 1 ? FaultDriveReadiness : 0));
    assert(m.latest_feedback.coordinate_valid[0] == (invalid ? 0 : 1));
    assert(m.control_policy.home_valid_mask.load() == (invalid ? 0 : 1));
    assert(f.last_outputs.motion_permitted_mask == 0);
  }
}

// Actual simulated SDO uploads test finite startup failure and explicit Home
// recovery; healthy later samples cannot silently bypass the exhausted bound.
void single_turn_startup_failures_and_home() {
  using namespace rosie::rtcore;
  for (bool unavailable : {false, true}) {
    SampledCycleFixture f(1, 500000, true, false);
    auto& m = *f.storage;
    m.verified_configuration_epoch.store(0);
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    assert(f.master.slaves[0]->encoder_uploads == 0);
    m.verified_configuration_epoch.store(1);
    m.control_policy.config_verified_mask.store(0);
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    assert(f.master.slaves[0]->encoder_uploads == 0);
    m.control_policy.config_verified_mask.store(1);
    f.master.faults.sample_unavailable_from_upload = unavailable ? 1 : 0;
    f.master.faults.sample_contradiction_from_upload = unavailable ? 0 : 1;
    f.master.faults.sample_contradiction_counts = 5017;
    for (unsigned attempt = 1; attempt <= 3; ++attempt) {
      assert(m.sample_absolute_axis(0, commissioning_test_clock) != unavailable);
      f.advance(1000000000 + attempt * 1000000);
      assert(m.sampled_bindings[0].metrics.startup_attempts == attempt);
      assert(m.latest_feedback.coordinate_valid[0] == 0 && m.execution_fault_reasons.load() == 0);
    }
    assert(m.sampled_bindings[0].metrics.startup_reason == uint32_t(unavailable ? SampleInvalidity::Unavailable : SampleInvalidity::Incoherent));
    f.master.faults = {};
    const auto uploads = f.master.slaves[0]->encoder_uploads;
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    assert(f.master.slaves[0]->encoder_uploads == uploads);
    f.advance(1004000000);
    assert(m.latest_feedback.coordinate_valid[0] == 0 && m.execution_fault_reasons.load() == 0);
    m.absolute_frame_epoch[0].fetch_add(1);
    f.advance(1005000000);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(1006000000);
    assert(m.latest_feedback.coordinate_valid[0] == 1 && m.latest_feedback.coordinate_counts[0] == 500000);
    assert(m.sampled_bindings[0].metrics.startup_verified == 1);
  }
}

// Generic drive faults inhibit both modes without losing a sound coordinate or
// Home. Encoder error 0x7305 must revoke both even with statusword Fault clear.
void sampled_drive_fault_coordinate_policy() {
  using namespace rosie::rtcore;
  for (bool single : {false, true}) {
    for (unsigned guard = 0; guard < 3; ++guard) {
      SampledCycleFixture f(1, 500000, single);
      auto& m = *f.storage;
      f.enabled = true;
      m.control_policy.connected.store(true);
      m.armed.store(true); m.axis_enable_mask.store(1);
      m.desired_mode_of_operation[0].store(8);
      f.advance(1002000000);
      assert(f.last_outputs.motion_permitted_mask == 1);
      const auto frame = m.absolute_frame_epoch[0].load();
      const auto home = m.home_epoch.load();
      auto pd = f.handles.cycle_backend.axis(0);
      if (guard == 0) pd.write_u16(16, 0x8);
      else pd.write_u16(28, guard == 1 ? 1 : 0x7305);
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
      f.advance(1003000000);
      const bool encoder = guard == 2;
      assert(f.last_outputs.motion_ready_mask == 0 && f.last_outputs.motion_permitted_mask == 0);
      assert(m.latest_feedback.coordinate_valid[0] == (encoder ? 0 : 1));
      assert(m.control_policy.home_valid_mask.load() == (encoder ? 0 : 1));
      assert(m.execution_fault_reasons.load() == (encoder ? FaultCoordinateReference : guard == 0 ? FaultDriveReadiness : 0));
      assert(m.sampled_bindings[0].failed == encoder);
      assert(m.axis_coordinate_epoch[0] == (encoder ? 2 : 1));
      pd.write_u16(16, 0x27); pd.write_u16(28, 0);
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
      f.advance(1004000000);
      assert(m.absolute_frame_epoch[0].load() == frame && m.home_epoch.load() == home);
      if (!encoder) {
        assert(m.latest_feedback.coordinate_valid[0] == 1 && m.control_policy.home_valid_mask.load() == 1);
        if (guard == 0) {
          assert(m.helper_controls.requests.push({MachineState::ControlOperation::ResetFault, 1, 99}));
          f.advance(1005000000);
          assert(m.control_policy.home_valid_mask.load() == 1 && m.latest_feedback.coordinate_valid[0] == 1);
          assert(m.execution_fault_reasons.load() == 0 && !m.armed.load());
        } else {
          assert(m.execution_fault_reasons.load() == 0 && f.last_outputs.motion_permitted_mask == 1);
        }
      } else {
        assert(m.control_policy.home_valid_mask.load() == 0);
        assert(m.execution_fault_reasons.load() == FaultCoordinateReference);
      }
    }
  }
}

// Three faulted startup cadences leave the entire comparison budget available;
// reset must verify the next coherent sample in the same frame without Home.
void single_turn_faulted_startup_reset() {
  using namespace rosie::rtcore;
  for (unsigned guard = 0; guard < 3; ++guard) {
    SampledCycleFixture f(1, 500000, true, false);
    auto& m = *f.storage;
    const auto frame = m.absolute_frame_epoch[0].load();
    const auto home = m.home_epoch.load();
    auto pd = f.handles.cycle_backend.axis(0);
    if (guard == 0) pd.write_u16(16, 0x8);
    else pd.write_u16(28, guard == 1 ? 1 : 0x7305);
    for (unsigned cadence = 1; cadence <= 3; ++cadence) {
      assert(m.sample_absolute_axis(0, commissioning_test_clock));
      f.advance(1000000000 + cadence * 1500000000ULL);
      assert(m.sampled_bindings[0].metrics.startup_attempts == 0);
      assert(m.sampled_bindings[0].metrics.startup_reason == uint32_t(SampleInvalidity::DriveFault));
      assert(m.latest_feedback.coordinate_valid[0] == 0 && m.execution_fault_reasons.load() == 0);
    }
    pd.write_u16(16, 0x27); pd.write_u16(28, 0);
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(5501000000);
    assert(m.sampled_bindings[0].metrics.startup_attempts == 1);
    assert(m.sampled_bindings[0].metrics.startup_verified == 1);
    assert(m.latest_feedback.coordinate_valid[0] == 1 && m.latest_feedback.coordinate_counts[0] == 500000);
    assert(m.execution_fault_reasons.load() == 0 && m.control_policy.home_valid_mask.load() == 0);
    assert(m.absolute_frame_epoch[0].load() == frame && m.home_epoch.load() == home);
  }
}

// Gated/refused axes issue no uploads and consume no checkpoint. A real failed
// upload consumes one slot; Home/configuration epochs reopen refused startup.
void single_turn_sampler_checkpoint_gates() {
  using namespace rosie::rtcore;
  SampledCycleFixture f(3, 500000, true, false);
  auto& m = *f.storage;
  for (bool epoch_gate : {false, true}) {
    m.verified_configuration_epoch.store(epoch_gate ? 0 : 1);
    m.control_policy.config_verified_mask.store(epoch_gate ? 7 : 6);
    m.absolute_next_axis = 0; m.absolute_last_poll_ns.fill(0);
    const auto uploads = f.master.slaves[1]->encoder_uploads;
    m.sample_absolute_due();
    assert(f.master.slaves[0]->encoder_uploads == 0);
    assert(f.master.slaves[1]->encoder_uploads == uploads + 3);
    assert(f.master.slaves[2]->encoder_uploads == 0);
    assert(m.absolute_next_axis == 2);
  }
  m.verified_configuration_epoch.store(1); m.control_policy.config_verified_mask.store(7);
  f.master.faults.sample_unavailable_from_upload = 1;
  for (unsigned attempt = 1; attempt <= 3; ++attempt) {
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    f.advance(1000000000 + attempt * 1000000);
  }
  assert(m.sampled_bindings[0].metrics.startup_attempts == 3);
  f.master.faults = {};
  m.absolute_next_axis = 0; m.absolute_last_poll_ns.fill(0);
  m.sample_absolute_due();
  assert(f.master.slaves[0]->encoder_uploads == 3);
  assert(f.master.slaves[1]->encoder_uploads == 9);
  assert(f.master.slaves[2]->encoder_uploads == 0);
  m.absolute_next_axis = 1; m.absolute_last_poll_ns.fill(0);
  f.master.faults.sample_unavailable_from_upload = 1;
  m.sample_absolute_due();
  assert(f.master.slaves[1]->encoder_uploads == 10);
  assert(f.master.slaves[2]->encoder_uploads == 0);
  f.master.faults = {};
  // Old-frame refusal is ignored before the cyclic owner publishes a reset.
  m.absolute_frame_epoch[0].fetch_add(1);
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  assert(f.master.slaves[0]->encoder_uploads == 6);
  m.configuration_epoch.store(2); m.verified_configuration_epoch.store(2);
  assert(!m.sample_absolute_axis(0, commissioning_test_clock));
  m.update_mailbox_readiness(true, commissioning_test_now);
  f.advance(1503000000);
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  assert(f.master.slaves[0]->encoder_uploads == 9);
}

// Unverified late-assembly fixture: low-word skew is 1000000 ns and the
// contradicting offset is 5017 counts, one count beyond speed plus tolerance.
// Missed cadence/age may not hide a contradiction or inhibit a coherent axis.
void single_turn_late_consistency() {
  using namespace rosie::rtcore;
  for (bool cadence : {false, true}) {
    for (bool contradicts : {false, true}) {
      SampledCycleFixture f(1, 500000, true);
      auto& m = *f.storage;
      SampledAbsolutePosition sample{1, 0, m.rt_cycle_counter.load(), 1001000000,
          cadence ? 2501000001ULL : 1003000000ULL, contradicts ? 505017 : 500000, 500000, 1};
      sample.bracket_completed_ns = 1001500000; sample.low_completed_ns = 1002000000;
      finish_sampled_acquisition(sample, 1500000000);
      const auto received_ns = cadence ? 2501000001ULL : 4001000001ULL;
      f.advance(received_ns - 1);
      assert(m.latest_feedback.coordinate_valid[0] == 1 && m.execution_fault_reasons.load() == 0);
      const auto previous_cycle = m.rt_cycle_counter.load();
      m.sampled_absolute[0].publish(sample);
      f.advance(received_ns);
      assert(m.rt_cycle_counter.load() == previous_cycle + 1);
      assert(m.execution_fault_reasons.load() == (contradicts ? FaultCoordinateReference : 0));
      assert(m.latest_feedback.coordinate_valid[0] == (contradicts ? 0 : 1));
      assert(m.control_policy.home_valid_mask.load() == (contradicts ? 0 : 1));
    }
  }
}

// A frame reset after admitted uploads retains the complete three-read Epoch
// rejection. A configuration reset fences the first reply as Unavailable; only
// fresh verified configuration and OP evidence may produce a valid new sample.
void single_turn_startup_reset_fences_sample() {
  using namespace rosie::rtcore;
  {
    SampledCycleFixture f(1, 500000, true, false);
    auto& m = *f.storage;
    // Unverified simulated upload interval: 1 ns, driven by the explicit hook.
    f.master.faults.sample_upload_delay_ns = 1;
    f.master.sample_advance_ns = [](uint64_t ns) {
      commissioning_test_now += ns;
      SampledCycleFixture::active->storage->absolute_frame_epoch[0].store(1);
    };
    assert(!m.sample_absolute_axis(0, commissioning_test_clock));
    SampledAbsolutePosition sample{};
    assert(m.sampled_absolute[0].read(sample));
    assert(sample.configuration_epoch == 1 && sample.home_epoch == 0 &&
           sample.reason == uint32_t(SampleInvalidity::Epoch));
    assert(!sample.valid && m.latest_feedback.coordinate_valid[0] == 0 && m.execution_fault_reasons.load() == 0);
    assert(f.master.slaves[0]->encoder_uploads == 3);
    f.master.sample_advance_ns = nullptr;
    assert(m.sample_absolute_axis(0, commissioning_test_clock));
    assert(m.sampled_absolute[0].read(sample) && sample.valid && sample.home_epoch == 1);
    assert(f.master.slaves[0]->encoder_uploads == 6);
  }
  SampledCycleFixture f(1, 500000, true, false);
  auto& m = *f.storage;
  // Unverified simulated upload interval: 1 ns, driven by the explicit hook.
  f.master.faults.sample_upload_delay_ns = 1;
  f.master.sample_advance_ns = [](uint64_t ns) {
    commissioning_test_now += ns;
    SampledCycleFixture::active->storage->configuration_epoch.store(2);
  };
  assert(!m.sample_absolute_axis(0, commissioning_test_clock));
  SampledAbsolutePosition sample{};
  assert(m.sampled_absolute[0].read(sample));
  assert(sample.configuration_epoch == 1 && sample.reason == uint32_t(SampleInvalidity::Unavailable));
  assert(!sample.valid && m.latest_feedback.coordinate_valid[0] == 0 && m.execution_fault_reasons.load() == 0);
  assert(!m.sample_absolute_axis(0, commissioning_test_clock));
  assert(f.master.slaves[0]->encoder_uploads == 1);
  f.master.sample_advance_ns = nullptr;
  // Unverified replacement-configuration fixture: observed OP continuously
  // from 1000000001 ns through 1500000001 ns, with mapping/startup proof in epoch 2.
  m.update_mailbox_readiness(true, 1000000001);
  commissioning_test_now = 1500000001;
  m.verified_configuration_epoch.store(2);
  m.control_policy.config_verified_mask.store(1);
  assert(m.sample_absolute_axis(0, commissioning_test_clock));
  assert(m.sampled_absolute[0].read(sample) && sample.valid && sample.configuration_epoch == 2);
  assert(f.master.slaves[0]->encoder_uploads == 4);
  std::puts("PASS: TestSingleTurnStartupResetFencesSample");
}
}

// Exercise the real job channels and RT reservation policy with explicit
// epochs. Queued, cancelled and stale receipts cannot become new permission.
int main(int argc, char** argv) {
  if (argc == 2) return run_mapping_readback_tests(argv[1]);
  run_mapping_readback_tests("");
  {
    MappingTestFixture fixture;
    fixture.master.active = true;
    g_stop.store(false);
    const auto before = fixture.master.slaves[0]->sdos;
    uint32_t abort = 0;
    for (uint16_t index : {0x1702, 0x1b04, 0x1c12, 0x1c13})
      assert(!fixture.storage->write_sdo_scalar_axis(0, index, 0, SdoScalarType::kU8, 0, &abort));
    assert(fixture.master.slaves[0]->sdos == before);
    assert(fixture.storage->write_sdo_scalar_axis(0, 0x607c, 0, SdoScalarType::kI32, 123, &abort));
    std::puts("PASS: TestRuntimeCannotInvalidatePREOPMapping");
  }
  std::puts("RUN CommissioningClaimRelease");
  commissioning_claim_release();
  std::puts("RUN FixedMappingReadback");
  fixed_mapping_readback();
  std::puts("RUN NativeHomeFailureDiagnostics");
  native_home_failure_diagnostics();
  std::puts("RUN NativeHomeVendorTransaction");
  native_home_executes_vendor_hm35_transaction_exactly();
  std::puts("RUN NativeHomeCyclicBrakeAnchor");
  native_home_cyclic_brake_anchor();
  std::puts("RUN NativeHomeGroupFailureDrain");
  native_home_group_failure_drain();
  std::puts("RUN NativeHomePartialSuccess");
  native_home_partial_success_survives_group_failure();
  std::puts("RUN SampledFixedMappingCycle");
  sampled_fixed_mapping_cycle();
  std::puts("RUN SampledZeroCrossing");
  sampled_zero_crossing_retains_evidence();
  std::puts("RUN SampledLateAudit");
  sampled_late_audit_is_diagnostic_only();
  std::puts("RUN SampledExpiry");
  sampled_expiry_preserves_running_position();
  std::puts("RUN SampledNineAxisExpiry");
  sampled_nine_axis_expiry_preserves_position();
  std::puts("RUN SampledAnchorCommandRefusal");
  sampled_native_anchor_command_refusal();
  std::puts("RUN IndependentRawBracket");
  independent_raw_source_bracket();
  std::puts("RUN IndependentRawRestore");
  independent_raw_home_restore_and_recovery();
  std::puts("RUN IndependentHomePosition");
  independent_home_preserves_actual_position();
  std::puts("RUN RetainedHomePosition");
  retained_home_recovers_moved_position();
  std::puts("RUN CurrentReferenceAnchorRefused");
  current_reference_anchor_file_refused();
  std::puts("RUN IndependentHomeJ2");
  independent_home_measured_j2_acquisition();
  std::puts("RUN HomeCaptureSettling");
  home_capture_waits_for_disabled_settling();
  std::puts("RUN HomeRestorePostPDO");
  home_and_restore_wait_for_post_acquisition_pdo();
  std::puts("RUN SampledDriveFaultPolicy");
  sampled_drive_fault_coordinate_policy();
  Options options;
  options.num_axes = 1;
  OptionsParseResult configuration;
  auto storage = std::make_unique<MachineState>(options, configuration);
  auto& machine = *storage;
  using Kind = MachineState::CommissioningJobKind;
  MachineState::CommissioningJob job{};
  job.kind = Kind::Home;
  job.identity = {17, machine.control_policy.generation.load(), 1, 1};
  job.dirty_epoch = 1;
  job.result.sequence = 17;
  job.home.axis_mask = 1;
  assert(machine.commissioning_job_current(job));
  assert(machine.try_claim_commissioning());
  assert(machine.commissioning_jobs.push(job));
  assert(!machine.try_claim_commissioning());
  assert(!machine.commissioning_jobs.push(job));
  MachineState::CommissioningJob received{};
  assert(machine.commissioning_jobs.pop(received));
  assert(received.identity.job == 17 && received.home.axis_mask == 1);
  assert(!machine.try_claim_commissioning()); // Popping is not completion.
  assert(machine.commissioning_job_receipts.push({received.identity, received.dirty_epoch,
                                                received.result, received.kind}));
  machine.commissioning_claim.store(MachineState::CommissioningClaim::Idle);
  assert(machine.try_claim_commissioning());
  MachineState::CommissioningJobReceipt receipt{};
  assert(machine.commissioning_job_receipts.pop(receipt));
  assert(receipt.identity.job == 17 && receipt.result.sequence == 17 && receipt.dirty_epoch == 1);
  assert(!machine.commissioning_job_receipts.pop(receipt));

  machine.dirty_configuration_epoch.store(2);
  assert(!machine.commissioning_job_current(received));
  machine.dirty_configuration_epoch.store(1);
  machine.configuration_epoch.store(2);
  assert(!machine.commissioning_job_current(received));
  machine.configuration_epoch.store(1);
  machine.home_epoch.store(2);
  assert(!machine.commissioning_job_current(received));
  machine.home_epoch.store(1);
  machine.control_policy.generation.fetch_add(1);
  assert(!machine.commissioning_job_current(received));

  using namespace rosie::rtcore;
  const uint64_t generation = machine.control_policy.generation.load();
  CommissioningContext context{generation, 1, 1, 1, 1, true, true, false, false, false};
  CommissioningRequest reserve{1, {19, generation, 1, 1}, CommissioningOperation::Reserve, 1};
  const auto reserved = evaluate_commissioning({}, context, reserve);
  assert(reserved.accepted && reserved.state.identity.home_epoch == 2);
  machine.commissioning_state = reserved.state;
  machine.home_epoch.store(2);
  machine.control_policy.connected.store(true);
  machine.control_policy.heartbeat_ns.store(now_monotonic_ns());
  MachineState::cycle_owner = &machine;
  MachineState::ControlRequest write{MachineState::ControlOperation::BeginService};
  write.generation = generation;
  write.configuration = 1;
  write.home = 2;
  write.job = 19;
  // A Home identity alone cannot authorize a configuration write.
  assert(!machine.apply_control(write).accepted);
  assert(machine.configuration_epoch.load() == 1 && machine.dirty_configuration_epoch.load() == 1);
  machine.configuration_write_job.store(19);
  assert(machine.apply_control(write).accepted);
  assert(machine.configuration_epoch.load() == 2 && machine.dirty_configuration_epoch.load() == 2);
  assert(machine.control_policy.config_verified_mask.load() == 0);
  assert(machine.axis_enable_mask.load() == 0 && !machine.armed.load());
  context.generation = machine.control_policy.generation.load();
  context.configuration_epoch = 2;
  context.home_epoch = 2;
  context.verified_mask = 0;
  const auto draining = poll_commissioning(reserved.state, context);
  assert(draining.state.phase == CommissioningPhase::Draining);
  assert(draining.state.service_mask == 0);
  const CommissioningRequest late{2, reserved.state.identity, CommissioningOperation::CompleteHome, 1};
  assert(!evaluate_commissioning(draining.state, context, late).accepted);
  const CommissioningRequest finish{3, reserved.state.identity, CommissioningOperation::Finish};
  const auto finished = evaluate_commissioning(draining.state, context, finish);
  assert(finished.accepted && !finished.job_completed);
  assert(finished.state.phase == CommissioningPhase::Idle);
  assert(!evaluate_commissioning(finished.state, context, finish).accepted);
  MachineState::cycle_owner = nullptr;

  Options cycle_options;
  cycle_options.num_axes = 1;
  cycle_options.cycle_ns = 1'000'000;
  cycle_options.max_cycle_lateness_ns = 20'000'000;
  cycle_options.axis[0].counts_per_unit = 1;
  auto cycle_storage = std::make_unique<MachineState>(cycle_options, configuration);
  auto& cyclic = *cycle_storage;
  CommissioningTestHandles handles;
  handles.off[0].cw = 0;
  handles.off[0].target_pos = 4;
  handles.off[0].target_vel = 8;
  handles.off[0].sw = 16;
  handles.off[0].pos = 20;
  handles.off[0].mode_disp = 26;
  handles.off[0].err = 28;
  auto image = handles.cycle_backend.axis(0);
  image.write_u16(16, 0x27);
  image.write_u8(26, 6);
  ec_master_t master;
  master.active = true;
  master.slaves.emplace_back(std::make_unique<ec_slave_config_t>());
  handles.sc[0] = master.slaves[0].get();
  cyclic.shared_master.store(&master);
  cyclic.next_ns = commissioning_test_clock();
  cyclic.previous_cycle_ns = 999'000'000;
  cyclic.bus_ready.store(true);
  cyclic.startup_ready = true;
  settle_mailbox(cyclic, master, 1000000000);
  cyclic.verified_configuration_epoch.store(1);
  cyclic.control_policy.config_verified_mask.store(1);
  cyclic.control_policy.connected.store(true);
  cyclic.control_policy.heartbeat_ns.store(commissioning_test_clock());
  cyclic.commissioning_state = {{23, 1, 1, 1}, CommissioningPhase::Active, 1, 1, 0};
  cyclic.commissioning_phase.store(1);
  cyclic.service_mode_axis_mask.store(1);
  cyclic.service_mode_override[0].store(6);
  cyclic.service_controlword_override[0].store(15);
  cyclic.commissioning_claim.store(MachineState::CommissioningClaim::Command); // Mailbox still in flight.
  const auto permitted = cyclic.step(CycleInputs<CommissioningTestHandles>{handles, nullptr, commissioning_test_clock});
  assert(permitted.submitted_mask == 1);
  assert((EC_READ_U16(handles.cycle_backend.image.data()) & 0x000f) == 0x000f);
  assert(cyclic.commissioning_phase.load() == 1);
  cyclic.previous_cycle_ns = commissioning_test_now;
  commissioning_test_now += 1'000'000;
  cyclic.stop_request_id.store(31);
  assert(cyclic.accept_controls.requests.push({MachineState::ControlOperation::Revoke}));
  const auto output = cyclic.step(CycleInputs<CommissioningTestHandles>{handles, nullptr, commissioning_test_clock});
  assert(handles.cycle_backend.submissions == 2);
  assert(output.submitted_mask == 1 && output.motion_permitted_mask == 0);
  assert(cyclic.commissioning_phase.load() == 2 && cyclic.service_mode_axis_mask.load() == 0);
  assert(cyclic.commissioning_claim.load() == MachineState::CommissioningClaim::Command);
  assert(cyclic.stop_ack_id.load() == 31);
  assert((EC_READ_U16(handles.cycle_backend.image.data()) & 0x000f) != 0x000f);
}
