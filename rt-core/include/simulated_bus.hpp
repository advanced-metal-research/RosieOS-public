#pragma once
#define ROSIE_RT_SIMULATED_BUS 1
// EtherCAT AL state bit values, matching the IgH ecrt.h contract.
#define EC_AL_STATE_INIT 1
#define EC_AL_STATE_PREOP 2

// Test-only CiA402 process-image plant. It exercises the SAME scheduler, PDO
// registration, interpolation and output policy as the live IgH build. It is
// not a drive firmware model or evidence of hardware compatibility. No sockets,
// raw Ethernet, kernel master, or physical outputs are opened by this backend.
#include <array>
#include <atomic>
#include <algorithm>
#include <map>
#include <memory>
#include <vector>
#include <cstring>
#include <cstdint>
#include <cstdio>
#include <cerrno>
#include <mutex>
#include <condition_variable>
#include <limits>
#include <time.h>
#include "engine/brake.hpp"

enum ec_direction_t { EC_DIR_INVALID, EC_DIR_OUTPUT, EC_DIR_INPUT };
enum ec_watchdog_mode_t { EC_WD_DISABLE, EC_WD_ENABLE };
enum ec_wc_state_t { EC_WC_ZERO, EC_WC_INCOMPLETE, EC_WC_COMPLETE };
constexpr unsigned int EC_END = ~0u;
// Simulation-only source objects. Their profile must explicitly declare the
// command-count frame and source validity; neither object is a real-drive default.
constexpr uint16_t kSimContinuousPositionObject = 0x20f0;
constexpr uint16_t kSimContinuousValidityObject = 0x20f1;
struct ec_master_state_t { unsigned int slaves_responding = 0, al_states = 0, link_up = 0; };
struct ec_domain_state_t { unsigned int working_counter = 0; ec_wc_state_t wc_state = EC_WC_ZERO; };
struct ec_slave_config_state_t { unsigned int online = 0, operational = 0, al_state = 0; };
struct ec_pdo_entry_info_t { uint16_t index; uint8_t subindex, bit_length; };
struct ec_pdo_info_t { uint16_t index; unsigned int n_entries; ec_pdo_entry_info_t* entries; };
struct ec_sync_info_t { uint8_t index; ec_direction_t dir; unsigned int n_pdos; ec_pdo_info_t* pdos; ec_watchdog_mode_t watchdog_mode; };
struct ec_pdo_entry_reg_t { uint16_t alias, position; uint32_t vendor_id, product_code; uint16_t index; uint8_t subindex; unsigned int* offset; unsigned int* bit_position; };
struct ec_master_t;
enum class SimDatagramState { Initial, Queued, Sent, Received };
struct ec_domain_t {
  ec_master_t* master;
  std::array<uint8_t, 16384> data{};
  std::array<uint8_t, 16384> submitted{};
  size_t used = 0;
  bool queued = false;
  SimDatagramState datagram_state = SimDatagramState::Initial;
  unsigned int received_working_counter = 0;
  ec_domain_state_t processed_state{};
  uint64_t input_receipts = 0; // Test evidence; queries/process never increment it.
};
// Explicit test controls, configured before activation. These names and APIs
// exist only in the simulated backend; no physical master reads them. Counts
// are deterministic call/cycle counts rather than wall-clock sleep periods.
struct SimExtraSlave {
  uint16_t position = 0;
  uint32_t vendor_id = 0, product_code = 0, revision = 0;
  bool present = true;
};
struct SimFaultControls {
  std::vector<SimExtraSlave> extra_slaves;
  // Unverified simulation-only controls: byte edge, readback delay in cycles,
  // stuck feedback and a receive-call exchange-loss boundary.
  uint16_t dio_inputs=0, dio_edge_inputs=0;
  uint64_t dio_edge_cycle=0, dio_readback_lag_cycles=0, dio_loss_cycle=0;
  bool dio_stuck_readback=false;
  uint64_t enable_delay_cycles = 0;
  uint64_t send_fail_from_call = 0;
  uint64_t send_fail_for_calls = 0; // zero means persistent when from_call != 0
  uint64_t receive_fail_from_call = 0;
  uint64_t receive_fail_for_calls = 0;
  uint64_t queue_fail_from_call = 0;
  uint64_t queue_fail_for_calls = 0;
  uint64_t process_fail_from_call = 0;
  uint64_t process_fail_for_calls = 0;
  uint64_t dc_fail_from_call = 0;
  uint64_t dc_fail_for_calls = 0;
  // WKC reports share the receive-cycle clock so diagnostic query frequency
  // and split-domain layouts cannot consume this injected interval.
  uint64_t wkc_incomplete_from_call = 0;
  uint64_t wkc_incomplete_for_calls = 0;
  uint64_t continuous_invalid_from_call = 0;
  uint64_t continuous_invalid_for_calls = 0;
  uint16_t continuous_invalid_position = 0;
  int send_error = -5;
  int receive_error = -5;
  int queue_error = -5;
  int process_error = -5;
  int dc_error = -5;
  int32_t disabled_displacement_counts = 0;
  // Unverified test-only PDO samples in drive position-frame counts/s and counts.
  int32_t velocity_actual_counts_per_s = 0, following_error_counts = 0;
  int32_t initial_position_counts = 0; // Test fixture count before first receipt.
  // Unverified simulator frame/jitter controls in counts; zero adds no offset.
  int32_t raw_multi_turn_origin_counts = 0;
  int32_t home_completion_jitter_counts = 0, home_encoder_jitter_counts = 0;
  uint32_t mapping_upload_failures = 0; // Test fixture unavailable upload count.
  // Simulated A6-EC U41.05 (0x2041:06) and C10.15 (0x2010:16) values; nonzero
  // encoder_alarm_unreadable makes both uploads abort. Zero is a clear drive.
  uint16_t encoder_fault_state = 0, multi_turn_overflow = 0, encoder_alarm_unreadable = 0;
  bool pdo_mismatch = false; // Test fixture descriptor width differs from registration.
  bool effective_startup_mismatch = false; // Power-cycle key remains at another active value.
  // Unverified simulation fixtures: override only SII; zero disables override.
  uint32_t sii_revision = 0;
  // Unverified upload count: cross zero after this low read; zero disables.
  uint64_t sample_zero_crossing_low_read = 0;
  // Unverified per-upload mailbox delay in ns; zero disables.
  uint64_t sample_upload_delay_ns = 0;
  // Unverified per-axis upload fixtures: one-based call interval, zero disables;
  // zero length keeps absence persistent. Contradiction is an encoder count offset.
  uint64_t sample_unavailable_from_upload = 0, sample_unavailable_for_uploads = 0;
  uint64_t sample_contradiction_from_upload = 0;
  int32_t sample_contradiction_counts = 0;
  int32_t brake_sag_counts = 0;
  uint16_t disabled_displacement_position = 0;
  // Persistent CSP tracking error at one physical EtherCAT slave position.
  // The offset is relative to each submitted target, never accumulated and
  // never applied while OperationEnabled is false.
  int32_t feedback_offset_counts = 0;
  uint16_t feedback_offset_position = 0;
  // Unverified fault-matrix injections: nanoseconds since first enabled sample
  // on slave zero; zero disables injection. They never alter runtime policy.
  uint64_t mode_mismatch_after_enabled_ns = 0;
  uint64_t statusword_fault_after_enabled_ns = 0;
  // Unverified mailbox block in ns, restricted to Home's 0x607c download.
  uint64_t sdo_block_ns = 0;
  uint64_t sdo_block_ms = 0;
  // Model of pinned IgH 1.6.9 master/master.c: a BUSY blocking SDO request is
  // waited on uninterruptibly, and in OPERATION the master FSM advances only
  // when the application sends. After activation each upload/download returns
  // only after this many further ecrt_master_send calls; zero disables. No
  // signal, release or deactivation frees a waiter, as on a RevPi servo host on
  // 2026-10-07 05:22 UTC, where the stuck request held the master until reboot.
  uint64_t mailbox_cycle_sends = 0;
};
// Process-wide so a waiter never touches a master that teardown has freed.
struct SimMailboxGate {
  std::mutex mutex;
  std::condition_variable advanced;
  uint64_t sends = 0;
  uint32_t in_call = 0;
};
inline SimMailboxGate sim_mailbox_gate;
inline uint64_t sim_mailbox_now_ns() {
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  return uint64_t(now.tv_sec) * 1000000000ULL + uint64_t(now.tv_nsec);
}
inline void sim_mailbox_exchange() {
  { std::lock_guard<std::mutex> lock(sim_mailbox_gate.mutex); ++sim_mailbox_gate.sends; }
  sim_mailbox_gate.advanced.notify_all();
}
inline void sim_mailbox_wait(uint64_t sends, uint16_t index, uint8_t subindex) {
  std::unique_lock<std::mutex> lock(sim_mailbox_gate.mutex);
  const uint64_t target = sim_mailbox_gate.sends + sends;
  ++sim_mailbox_gate.in_call;
  std::fprintf(stderr, "sim_mailbox_enter time_ns=%llu index=0x%04x sub=0x%02x in_call=%u\n",
               static_cast<unsigned long long>(sim_mailbox_now_ns()), index, subindex,
               sim_mailbox_gate.in_call);
  sim_mailbox_gate.advanced.wait(lock, [&] { return sim_mailbox_gate.sends >= target; });
  --sim_mailbox_gate.in_call;
  std::fprintf(stderr, "sim_mailbox_exit time_ns=%llu index=0x%04x sub=0x%02x in_call=%u\n",
               static_cast<unsigned long long>(sim_mailbox_now_ns()), index, subindex,
               sim_mailbox_gate.in_call);
}
struct SimEntry { ec_pdo_entry_info_t info; uint8_t sync; ec_domain_t* domain = nullptr; unsigned int offset = 0; };
struct ec_slave_config_t {
  bool sii_readable = true;
  uint32_t sync0_cycle_ns = 0;
  int32_t sync0_shift_ns = 0;
  // Simulated DIO terminal: low byte = channels 0-7, optional high byte = 8-15.
  // An input index of 0 is an output-only terminal (no input PDO).
  bool dio=false;
  uint16_t dio_input_index=0, dio_output_index=0, dio_input_high_index=0, dio_output_high_index=0;
  uint8_t dio_input_subindex=0, dio_output_subindex=0, dio_input_high_subindex=0, dio_output_high_subindex=0;
  uint16_t dio_invert_mask=0, dio_physical=0, dio_pending=0;
  uint64_t dio_transition_cycle=0;
  std::array<uint16_t,16> dio_feedback_mask{};
  uint16_t position;
  uint32_t vendor, product;
  uint32_t sii_revision = 0;
  std::atomic<int64_t> raw_multi_turn_encoder_counts{0};
  int64_t raw_multi_turn_frame_offset_counts = 0;
  std::atomic<int32_t> home_offset_counts{0}, home_method{0};
  bool home_triggered = false, home_attained = false;
  // Unverified AL-state fault injection: zero follows the master's activation.
  std::atomic<uint8_t> al_state_override{0};
  bool raw_multi_turn_encoder_source = false;
  uint64_t encoder_low_reads = 0; // Mailbox owner only.
  uint64_t encoder_uploads = 0; // Mailbox owner only; counts failed uploads too.
  std::vector<SimEntry> entries;
  // Unverified fixture cache: mailbox assignment writes do not refresh it.
  std::map<uint8_t, uint16_t> scanned_assignment;
  std::map<uint32_t, std::vector<uint8_t>> sdos;
  int32_t position_counts = 0;
  uint16_t statusword = 0x0240;
  uint8_t mode_display = 8;
  uint64_t enable_wait_cycles = 0;
  bool observed_operation_enabled = false;
  bool disabled_displacement_applied = false;
  uint64_t first_enabled_ns = 0;
  bool mode_mismatch_injected = false;
  bool statusword_fault_injected = false;
  rosie::rtcore::BrakeConfig brake{};
  uint16_t brake_signal_index = 0;
  uint8_t brake_signal_subindex = 0;
  uint32_t brake_signal_bit = 0;
  uint64_t brake_transition_ns = 0;
  bool brake_enabled = false;
  bool brake_applied = true;
  bool brake_sag_applied = false;
  std::mutex mailbox_mutex;
};
struct ec_master_t {
  std::vector<std::unique_ptr<ec_slave_config_t>> slaves;
  std::vector<std::unique_ptr<ec_domain_t>> domains;
  ec_slave_config_t* reference_clock = nullptr;
  bool active = false;
  // Unverified scan fixture: queue first, complete on the second observation.
  bool retain_for_rescan = false, reserved = true, scan_busy = false, scan_stuck = false;
  unsigned scan_observations = 0, rescan_requests = 0, reservations = 1, releases = 0;
  SimFaultControls faults{};
  uint64_t send_calls = 0;
  uint64_t receive_calls = 0;
  uint64_t queue_calls = 0;
  uint64_t process_calls = 0;
  uint64_t dc_calls = 0;
  uint64_t reference_sync_calls = 0;
  uint64_t application_time_ns = 0, reference_sync_time_ns = 0;
  bool sdo_delay_armed = false; // Mailbox owner only; first runtime write arms read delays.
  // Test-only deterministic clock hook; the daemon uses nanosleep for injection.
  void (*sample_advance_ns)(uint64_t) = nullptr;
};

struct ec_slave_info_t {
  uint32_t revision_number = 0;
  uint8_t al_state = 2;
  uint16_t position = 0;
  uint32_t vendor_id = 0, product_code = 0;
};
inline int ecrt_master_get_slave(ec_master_t* m, uint16_t position, ec_slave_info_t* info) {
  if (m) for (const auto& s : m->faults.extra_slaves) if (s.position == position && s.present) {
    *info = {s.revision, 2, position, s.vendor_id, s.product_code};
    return 0;
  }
  if (m) for (const auto& s : m->slaves) if (s->position == position) {
    if (!s->sii_readable) return -EIO;
    info->revision_number = s->sii_revision;
    info->position = position; info->vendor_id = s->vendor; info->product_code = s->product;
    info->al_state = s->al_state_override.load() ? s->al_state_override.load() : m->active ? 8 : 2;
    return 0;
  }
  return -1;
}

inline bool sim_configure_faults(ec_master_t* m, const SimFaultControls& faults) {
  if (!m || m->active || faults.send_error >= 0 || faults.receive_error >= 0 ||
      faults.queue_error >= 0 || faults.process_error >= 0 || faults.dc_error >= 0 ||
      (!faults.send_fail_from_call && faults.send_fail_for_calls) ||
      (!faults.receive_fail_from_call && faults.receive_fail_for_calls) ||
      (!faults.queue_fail_from_call && faults.queue_fail_for_calls) ||
      (!faults.process_fail_from_call && faults.process_fail_for_calls) ||
      (!faults.dc_fail_from_call && faults.dc_fail_for_calls) ||
      (!faults.wkc_incomplete_from_call && faults.wkc_incomplete_for_calls) ||
      (!faults.continuous_invalid_from_call && faults.continuous_invalid_for_calls)) return false;
  m->faults = faults;
  return true;
}

inline bool sim_fault_on_call(uint64_t call, uint64_t first, uint64_t count) {
  return first != 0 && call >= first && (count == 0 || call - first < count);
}
// Timestamp the injection boundary, independently of the daemon's diagnosis;
// tests must not infer fault latency from the first already-faulted sample.
inline void sim_report_fault_edge(const char* name, uint64_t call, uint64_t first, uint64_t count) {
  if (!first || call < first || (call != first && (!count || call - first != count))) return;
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  std::fprintf(stderr, "sim_%s_%s time_ns=%llu\n", name, call == first ? "failure" : "recovered",
               static_cast<unsigned long long>(uint64_t(now.tv_sec) * 1000000000 + now.tv_nsec));
}
template<class T> inline T sim_read(const void* p) { T v; std::memcpy(&v,p,sizeof v); return v; }
template<class T> inline void sim_write(void* p,T v) { std::memcpy(p,&v,sizeof v); }
#define EC_READ_U8(p) sim_read<uint8_t>(p)
#define EC_READ_U16(p) sim_read<uint16_t>(p)
#define EC_READ_U32(p) sim_read<uint32_t>(p)
#define EC_READ_S16(p) sim_read<int16_t>(p)
#define EC_READ_S32(p) sim_read<int32_t>(p)
#define EC_WRITE_S8(p,v) sim_write<int8_t>(p,v)
#define EC_WRITE_U16(p,v) sim_write<uint16_t>(p,v)
#define EC_WRITE_S16(p,v) sim_write<int16_t>(p,v)
#define EC_WRITE_U32(p,v) sim_write<uint32_t>(p,v)
#define EC_WRITE_S32(p,v) sim_write<int32_t>(p,v)

inline std::unique_ptr<ec_master_t> sim_rescan_master;
inline ec_master_t* ecrt_request_master(unsigned int) {
  if (!sim_rescan_master) return new ec_master_t;
  auto* master = sim_rescan_master.release();
  master->reserved = true; master->retain_for_rescan = false; ++master->reservations;
  return master;
}
inline void ecrt_release_master(ec_master_t* m) {
  if (!m) return;
  if (m->retain_for_rescan) {
    m->reserved = false; m->domains.clear(); ++m->releases;
    if (!sim_rescan_master) sim_rescan_master.reset(m);
  } else delete m;
}
inline ec_master_t* ecrt_open_master(unsigned int) { return sim_rescan_master.get(); }
inline bool sim_queue_pdo_rescan(unsigned int) {
  if (!sim_rescan_master || sim_rescan_master->active || sim_rescan_master->reserved) return false;
  sim_rescan_master->scan_busy = true; sim_rescan_master->scan_observations = 0;
  ++sim_rescan_master->rescan_requests;
  return true;
}
struct ec_master_info_t { unsigned slave_count = 0; bool link_up = false; uint8_t scan_busy = 0; };
inline int ecrt_master(ec_master_t* master, ec_master_info_t* info) {
  if (!master) return -1;
  if (master->scan_busy && !master->scan_stuck && ++master->scan_observations == 2) {
    for (auto& slave : master->slaves) {
      // A scan replaces cached entry geometry as well as the selected preset;
      // a PREOP assignment write alone must never refresh either representation.
      slave->entries.clear();
      for (auto& assignment : slave->scanned_assignment) {
        const auto key = (uint32_t(0x1c10 + assignment.first) << 8) | 1;
        const auto& bytes = slave->sdos.at(key);
        assignment.second = uint16_t(bytes[0]) | uint16_t(bytes[1]) << 8;
        const auto count = slave->sdos.at(uint32_t(assignment.second) << 8).at(0);
        for (unsigned entry = 1; entry <= count; ++entry) {
          const auto& mapped = slave->sdos.at((uint32_t(assignment.second) << 8) | entry);
          const uint32_t value = uint32_t(mapped.at(0)) | uint32_t(mapped.at(1)) << 8 |
              uint32_t(mapped.at(2)) << 16 | uint32_t(mapped.at(3)) << 24;
          slave->entries.push_back({{uint16_t(value >> 16), uint8_t(value >> 8), uint8_t(value)}, assignment.first});
        }
      }
    }
    master->scan_busy = false;
  }
  const auto extras = std::count_if(master->faults.extra_slaves.begin(), master->faults.extra_slaves.end(), [](const auto& s) { return s.present; });
  *info = {unsigned(master->slaves.size() + extras), true, uint8_t(master->scan_busy)};
  return 0;
}
inline int ecrt_master_deactivate(ec_master_t* m) { m->active = false; return 0; }
inline int ecrt_master_select_reference_clock(ec_master_t* m, ec_slave_config_t* sc) { m->reference_clock = sc; return 0; }
inline ec_domain_t* ecrt_master_create_domain(ec_master_t* m) {
  auto d = std::make_unique<ec_domain_t>(); d->master = m;
  auto* ptr = d.get(); m->domains.push_back(std::move(d)); return ptr;
}
inline ec_slave_config_t* ecrt_master_slave_config(ec_master_t* m, uint16_t, uint16_t pos, uint32_t vendor, uint32_t product) {
  // Extra devices have no configuration, PDO image or mailbox map. An attempt
  // to configure one is a test failure, not a silently promoted extra motor.
  for (const auto& s : m->faults.extra_slaves) if (s.position == pos) return nullptr;
  for (auto& s : m->slaves) if (s->position == pos) return s->vendor == vendor && s->product == product ? s.get() : nullptr;
  auto s = std::make_unique<ec_slave_config_t>(); s->position=pos; s->vendor=vendor; s->product=product;
  auto* ptr=s.get(); m->slaves.push_back(std::move(s)); return ptr;
}
inline int ecrt_slave_config_pdos(ec_slave_config_t* s, unsigned int, const ec_sync_info_t* syncs) {
  s->entries.clear();
  s->scanned_assignment.clear();
  auto object = [&](uint16_t index, uint8_t subindex, uint32_t value, size_t bytes) {
    auto& data = s->sdos[(uint32_t(index) << 8) | subindex];
    data.resize(bytes);
    for (size_t i = 0; i < bytes; ++i) data[i] = static_cast<uint8_t>(value >> (8 * i));
  };
  for (auto* sync = syncs; sync->index != 0xff; ++sync) {
    object(0x1c10 + sync->index, 0, sync->n_pdos, 1);
    for (unsigned i = 0; i < sync->n_pdos; ++i) {
      const auto& pdo = sync->pdos[i];
      s->scanned_assignment[sync->index] = pdo.index;
      object(0x1c10 + sync->index, i + 1, pdo.index, 2);
      object(pdo.index, 0, pdo.n_entries, 1);
      for (unsigned j = 0; j < pdo.n_entries; ++j) {
        const auto& entry = pdo.entries[j];
        object(pdo.index, j + 1, (uint32_t(entry.index) << 16) |
            (uint32_t(entry.subindex) << 8) | entry.bit_length, 4);
      }
    }
  }
  for (auto* sync=syncs; sync->index != 0xff; ++sync)
    for (unsigned int i=0; i<sync->n_pdos; ++i)
      for (unsigned int j=0; j<sync->pdos[i].n_entries; ++j)
        s->entries.push_back({sync->pdos[i].entries[j], sync->index});
  return 0;
}
// Present IgH's scan separately from the slave's mailbox so tests cannot pass
// by silently replacing stale registration geometry after an SDO write.
inline int ecrt_master_get_sync_manager(ec_master_t* m, uint16_t pos, uint8_t sync, ec_sync_info_t* out) {
  for (const auto& s : m->slaves) if (s->position == pos) {
    *out = {};
    out->index = sync;
    out->n_pdos = s->scanned_assignment.count(sync);
    return 0;
  }
  return -1;
}
inline int ecrt_master_get_pdo(ec_master_t* m, uint16_t pos, uint8_t sync, uint16_t pdo_pos, ec_pdo_info_t* out) {
  for (const auto& s : m->slaves) if (s->position == pos && pdo_pos == 0 && s->scanned_assignment.count(sync)) {
    *out = {};
    out->index = s->scanned_assignment.at(sync);
    for (const auto& entry : s->entries) if (entry.sync == sync) ++out->n_entries;
    return 0;
  }
  return -1;
}
inline int ecrt_master_get_pdo_entry(ec_master_t* m, uint16_t pos, uint8_t sync, uint16_t pdo_pos,
                                    uint16_t entry_pos, ec_pdo_entry_info_t* out) {
  for (const auto& s : m->slaves) if (s->position == pos && pdo_pos == 0)
    for (const auto& entry : s->entries) if (entry.sync == sync && entry_pos-- == 0) {
      *out = entry.info;
      return 0;
    }
  return -1;
}
inline int sim_register(SimEntry& e, ec_domain_t* d) {
  if (e.domain) return e.domain == d ? static_cast<int>(e.offset) : -1;
  if (d->used + e.info.bit_length/8 > d->data.size()) return -1;
  e.domain=d; e.offset=d->used; d->used+=e.info.bit_length/8; return e.offset;
}
inline int ecrt_domain_reg_pdo_entry_list(ec_domain_t* d, const ec_pdo_entry_reg_t* regs) {
  for (auto* r=regs; r->index; ++r) {
    bool found=false;
    for (auto& s : d->master->slaves) {
      if (s->position!=r->position || s->vendor!=r->vendor_id || s->product!=r->product_code) continue;
      for (auto& e : s->entries) if (e.info.index==r->index && e.info.subindex==r->subindex) {
        int offset=sim_register(e,d); if(offset<0) return -1;
        *r->offset=offset; found=true; break;
      }
    }
    if(!found) return -1;
  }
  return 0;
}
inline int ecrt_slave_config_reg_pdo_entry_pos(ec_slave_config_t* s, uint8_t sync, unsigned int, unsigned int pos, ec_domain_t* d, unsigned int*) {
  for(auto& e:s->entries) if(e.sync==sync && pos--==0) return sim_register(e,d);
  return -1;
}
inline uint8_t* sim_object(ec_slave_config_t* s,uint16_t index) {
  for(auto& e:s->entries) if(e.domain && e.info.index==index && e.info.subindex==0) return e.domain->data.data()+e.offset;
  return nullptr;
}
inline const uint8_t* sim_submitted_object(ec_slave_config_t* s,uint16_t index) {
  for(auto& e:s->entries) if(e.domain && e.info.index==index && e.info.subindex==0)
    return e.domain->submitted.data()+e.offset;
  return nullptr;
}
// Plant progression and host-visible PDO presentation are separate. The plant
// uses its last successfully submitted outputs even when input traffic fails
// or an unqueued domain has no reply. One receive call remains the deterministic
// simulation step; this is not a physical timing/firmware qualification.
inline void sim_step_plant_at(ec_master_t* m, uint64_t now) {
  if(!m->active) return;
  for(auto& owned:m->slaves) {
    auto* s=owned.get();
    if(s->dio) {
      uint16_t command=0;
      for(const auto& e:s->entries) if(e.domain) {
        if(e.info.index==s->dio_output_index && e.info.subindex==s->dio_output_subindex) command|=e.domain->submitted[e.offset];
        if(s->dio_output_high_index && e.info.index==s->dio_output_high_index && e.info.subindex==s->dio_output_high_subindex)
          command|=uint16_t(e.domain->submitted[e.offset])<<8;
      }
      if(command!=s->dio_pending) { s->dio_pending=command; s->dio_transition_cycle=m->receive_calls; }
      if(!m->faults.dio_stuck_readback && m->receive_calls-s->dio_transition_cycle>=m->faults.dio_readback_lag_cycles) s->dio_physical=command;
      continue;
    }
    auto* cwp=sim_submitted_object(s,0x6040); auto* target=sim_submitted_object(s,0x607a);
    const uint16_t cw=cwp ? EC_READ_U16(cwp):0;
    auto* modep=sim_submitted_object(s,0x6060); uint8_t mode=modep ? EC_READ_U8(modep):8;
    bool operation_enabled = (cw&15)==15;
    if (!operation_enabled) s->enable_wait_cycles = 0;
    else if (s->enable_wait_cycles < m->faults.enable_delay_cycles) {
      ++s->enable_wait_cycles;
      operation_enabled = false;
    }
    if (operation_enabled && !s->first_enabled_ns) {
      s->first_enabled_ns = now;
      if (s->position == m->faults.feedback_offset_position && m->faults.feedback_offset_counts)
        std::fprintf(stderr, "sim_feedback_offset time_ns=%llu position_counts=%d\n",
                     static_cast<unsigned long long>(now), m->faults.feedback_offset_counts);
    }
    if (s->position == 0 && s->first_enabled_ns) {
      if (m->faults.mode_mismatch_after_enabled_ns &&
          now - s->first_enabled_ns >= m->faults.mode_mismatch_after_enabled_ns) {
        if (!s->mode_mismatch_injected)
          std::fprintf(stderr, "sim_mode_mismatch time_ns=%llu position_counts=%d\n",
                       static_cast<unsigned long long>(now), s->position_counts);
        s->mode_mismatch_injected = true;
        mode = 9;
      }
      if (m->faults.statusword_fault_after_enabled_ns &&
          now - s->first_enabled_ns >= m->faults.statusword_fault_after_enabled_ns) {
        if (!s->statusword_fault_injected)
          std::fprintf(stderr, "sim_statusword_fault time_ns=%llu position_counts=%d\n",
                       static_cast<unsigned long long>(now), s->position_counts);
        s->statusword_fault_injected = true;
        operation_enabled = false;
      }
    }
    s->statusword = operation_enabled ? 0x0227 : (cw&15)==15 || (cw&15)==7 ? 0x0223 : (cw&15)==6 ? 0x0221 : 0x0240;
    if (s->statusword_fault_injected) s->statusword |= 8;
    if (operation_enabled != s->brake_enabled) {
      s->brake_enabled = operation_enabled;
      s->brake_transition_ns = now;
      if (operation_enabled) s->brake_sag_applied = false;
    }
    if (!s->brake.present) s->brake_applied = false;
    else if (operation_enabled) {
      if (now - s->brake_transition_ns >= s->brake.release_delay_ns) s->brake_applied = false;
    } else if (now - s->brake_transition_ns >= s->brake.hold_delay_ns) s->brake_applied = true;
    if(operation_enabled && !s->brake_applied && mode==8 && target) {
      const int64_t offset = s->position == m->faults.feedback_offset_position
          ? m->faults.feedback_offset_counts : 0;
      const int64_t actual = int64_t(EC_READ_S32(target)) + offset;
      s->position_counts = static_cast<int32_t>(std::max<int64_t>(std::numeric_limits<int32_t>::min(),
          std::min<int64_t>(std::numeric_limits<int32_t>::max(), actual)));
    }
    const bool home = operation_enabled && mode == 6 && (cw & 0x1f) == 0x1f;
    if (home && !s->home_triggered && s->raw_multi_turn_encoder_source && s->home_method.load() == 35) {
      // Linear HM35 re-references continuous 6064 to the configured 607C zero;
      // the independent raw encoder retains its arbitrary origin.
      const int64_t raw_multi = int64_t(s->position_counts) - s->raw_multi_turn_frame_offset_counts + m->faults.home_encoder_jitter_counts;
      s->position_counts = int32_t(std::clamp(int64_t(s->home_offset_counts.load()) + m->faults.home_completion_jitter_counts,
          int64_t(INT32_MIN), int64_t(INT32_MAX)));
      s->raw_multi_turn_frame_offset_counts = int64_t(s->position_counts) - raw_multi;
      s->home_attained = true;
    }
    s->home_triggered = home;
    if (home || s->home_attained) s->statusword |= 0x9000;
    if (operation_enabled) s->observed_operation_enabled = true;
    if (!operation_enabled && s->observed_operation_enabled && !s->brake_applied &&
        !s->brake_sag_applied && m->faults.brake_sag_counts != 0) {
      const int64_t displaced = int64_t(s->position_counts) + m->faults.brake_sag_counts;
      s->position_counts = static_cast<int32_t>(std::clamp(displaced,
          int64_t(std::numeric_limits<int32_t>::min()), int64_t(std::numeric_limits<int32_t>::max())));
      s->brake_sag_applied = true;
    }
    if ((cw&15)!=15 && s->observed_operation_enabled && !s->disabled_displacement_applied &&
        s->position == m->faults.disabled_displacement_position &&
        m->faults.disabled_displacement_counts != 0) {
      const int64_t displaced = int64_t(s->position_counts) + m->faults.disabled_displacement_counts;
      s->position_counts = static_cast<int32_t>(std::max<int64_t>(std::numeric_limits<int32_t>::min(),
          std::min<int64_t>(std::numeric_limits<int32_t>::max(), displaced)));
      s->disabled_displacement_applied = true;
    }
    s->mode_display = mode;
    // The independent motor encoder follows displacement, not an HM35 origin shift.
    s->raw_multi_turn_encoder_counts.store(int64_t(s->position_counts) - s->raw_multi_turn_frame_offset_counts, std::memory_order_release);
  }
}
inline void sim_step_plant(ec_master_t* m) {
  timespec timestamp{};
  clock_gettime(CLOCK_MONOTONIC, &timestamp);
  sim_step_plant_at(m, uint64_t(timestamp.tv_sec) * 1000000000 + timestamp.tv_nsec);
}
inline unsigned int sim_expected_working_counter(const ec_domain_t* d) {
  unsigned int axes = 0;
  for (const auto& axis : d->master->slaves)
    for (const auto& e : axis->entries) if (e.domain == d) { ++axes; break; }
  return 2 * axes;
}
inline void sim_present_domain_inputs(ec_domain_t* d) {
  for(const auto& s:d->master->slaves) if(s->dio) {
    const auto& f=d->master->faults;
    uint16_t value=f.dio_edge_cycle && d->master->receive_calls>=f.dio_edge_cycle ? f.dio_edge_inputs : f.dio_inputs;
    for(unsigned bit=0;bit<16;++bit) {
      value &= uint16_t(~s->dio_feedback_mask[bit]);
      if(s->dio_physical&(1u<<bit)) value|=s->dio_feedback_mask[bit];
    }
    value^=s->dio_invert_mask;
    for(const auto& e:s->entries) if(e.domain==d) {
      if(s->dio_input_index && e.info.index==s->dio_input_index && e.info.subindex==s->dio_input_subindex) d->data[e.offset]=uint8_t(value);
      if(s->dio_input_high_index && e.info.index==s->dio_input_high_index && e.info.subindex==s->dio_input_high_subindex) d->data[e.offset]=uint8_t(value>>8);
    }
  }
  for (const auto& axis : d->master->slaves)
    for (const auto& e : axis->entries) if (e.domain == d && e.info.subindex == 0) {
      auto* p = d->data.data() + e.offset;
      if (e.info.index == 0x606c) EC_WRITE_S32(p, d->master->faults.velocity_actual_counts_per_s);
      if (e.info.index == 0x60f4) EC_WRITE_S32(p, d->master->faults.following_error_counts);
      if (e.info.index == 0x6041) EC_WRITE_U16(p, axis->statusword);
      if (e.info.index == 0x6064) EC_WRITE_S32(p, axis->position_counts);
      if (e.info.index == 0x6061) sim_write<uint8_t>(p, axis->mode_display);
      if (e.info.index == kSimContinuousPositionObject)
        EC_WRITE_S32(p, axis->position_counts);
      if (e.info.index == kSimContinuousValidityObject) {
        const auto& faults = d->master->faults;
        const bool invalid = axis->position == faults.continuous_invalid_position &&
            sim_fault_on_call(d->master->receive_calls, faults.continuous_invalid_from_call,
                             faults.continuous_invalid_for_calls);
        EC_WRITE_U16(p, invalid ? 0 : 1);
      }
    }
  // A mapped feedback bit may share a word with other drive observations.
  // Overlay only that bit after presenting the ordinary input objects.
  for (const auto& axis : d->master->slaves)
    for (const auto& e : axis->entries)
      if (e.domain == d && axis->brake_signal_index && e.info.index == axis->brake_signal_index &&
          e.info.subindex == axis->brake_signal_subindex && axis->brake_signal_bit < e.info.bit_length) {
        auto* p = d->data.data() + e.offset;
        uint32_t value = e.info.bit_length == 8 ? EC_READ_U8(p) :
            e.info.bit_length == 16 ? EC_READ_U16(p) : EC_READ_U32(p);
        const uint32_t mask = uint32_t{1} << axis->brake_signal_bit;
        value = axis->brake_applied ? value & ~mask : value | mask;
        if (e.info.bit_length == 8) sim_write<uint8_t>(p, value);
        else if (e.info.bit_length == 16) EC_WRITE_U16(p, value);
        else if (e.info.bit_length == 32) EC_WRITE_U32(p, value);
      }
}
inline int ecrt_master_receive(ec_master_t* m) {
  ++m->receive_calls;
  sim_report_fault_edge("receive", m->receive_calls, m->faults.receive_fail_from_call, m->faults.receive_fail_for_calls);
  sim_step_plant(m);
  if (sim_fault_on_call(m->receive_calls, m->faults.receive_fail_from_call,
                        m->faults.receive_fail_for_calls)) return m->faults.receive_error;
  if (!m->active) return 0;
  for (const auto& d : m->domains) if (d->datagram_state == SimDatagramState::Sent) {
    sim_present_domain_inputs(d.get());
    d->datagram_state = SimDatagramState::Received;
    const auto expected = sim_expected_working_counter(d.get());
    const bool incomplete = sim_fault_on_call(m->receive_calls,
        m->faults.wkc_incomplete_from_call, m->faults.wkc_incomplete_for_calls) ||
        (m->faults.dio_loss_cycle && m->receive_calls>=m->faults.dio_loss_cycle);
    d->received_working_counter = incomplete && expected != 0 ? expected - 1 : expected;
    ++d->input_receipts;
  }
  return 0;
}
inline int ecrt_master_activate(ec_master_t* m) { m->active=true; return ecrt_master_receive(m); }
inline uint8_t* ecrt_domain_data(ec_domain_t* d) { return d->data.data(); }
inline int ecrt_domain_process(ec_domain_t* d) {
  auto* m = d->master;
  ++m->process_calls;
  sim_report_fault_edge("process", m->process_calls, m->faults.process_fail_from_call, m->faults.process_fail_for_calls);
  if (sim_fault_on_call(m->process_calls, m->faults.process_fail_from_call,
                       m->faults.process_fail_for_calls)) return m->faults.process_error;
  // Like pinned IgH, processing an unchanged RECEIVED datagram reuses its
  // counter. COMPLETE alone therefore cannot prove a new input observation.
  const auto counter = m->active && d->datagram_state == SimDatagramState::Received
      ? d->received_working_counter : 0;
  const auto expected = sim_expected_working_counter(d);
  d->processed_state = {counter, !counter ? EC_WC_ZERO :
      counter == expected ? EC_WC_COMPLETE : EC_WC_INCOMPLETE};
  return 0;
}
inline int ecrt_domain_queue(ec_domain_t* d) {
  auto* m = d->master;
  ++m->queue_calls;
  sim_report_fault_edge("queue", m->queue_calls, m->faults.queue_fail_from_call, m->faults.queue_fail_for_calls);
  if (sim_fault_on_call(m->queue_calls, m->faults.queue_fail_from_call,
                       m->faults.queue_fail_for_calls)) return m->faults.queue_error;
  d->queued = true;
  d->datagram_state = SimDatagramState::Queued;
  return 0;
}
inline int ecrt_master_send(ec_master_t* m) {
  ++m->send_calls;
  sim_report_fault_edge("send", m->send_calls, m->faults.send_fail_from_call, m->faults.send_fail_for_calls);
  if (sim_fault_on_call(m->send_calls, m->faults.send_fail_from_call,
                        m->faults.send_fail_for_calls)) return m->faults.send_error;
  // A send success is not evidence that a particular domain was queued. Keep
  // pending queues on send failure; the output owner must require a successful
  // queue for every affected domain in the current cycle before acknowledging.
  for (auto& d : m->domains) if (d->queued) {
    std::memcpy(d->submitted.data(), d->data.data(), d->used);
    d->queued = false;
    d->datagram_state = SimDatagramState::Sent;
  }
  if (m->faults.mailbox_cycle_sends) sim_mailbox_exchange();
  return 0;
}
inline int ecrt_master_state(ec_master_t* m, ec_master_state_t* s) {
  const auto extras = std::count_if(m->faults.extra_slaves.begin(), m->faults.extra_slaves.end(), [](const auto& x) { return x.present; });
  *s = {static_cast<unsigned int>(m->slaves.size() + extras), (m->active ? 8u : 2u) | (extras ? 2u : 0u), 1};
  return 0;
}
inline int ecrt_domain_state(ec_domain_t* d, ec_domain_state_t* s) {
  *s = d->processed_state;
  return 0;
}
inline int ecrt_slave_config_state(ec_slave_config_t*, ec_slave_config_state_t* s) { *s={1,1,8}; return 0; }
// Delay runtime mailbox traffic after the first download. Startup verification
// remains available before a controller connects; subsequent readbacks also block.
inline void sim_block_sdo(ec_master_t* master) {
  if (!master->sdo_delay_armed || !master->faults.sdo_block_ms) return;
  timespec started{};
  clock_gettime(CLOCK_MONOTONIC, &started);
  const uint64_t started_ns = uint64_t(started.tv_sec) * 1000000000ULL + started.tv_nsec;
  std::fprintf(stderr, "SIM_SDO_BLOCK begin_ns=%llu duration_ms=%llu\n",
               static_cast<unsigned long long>(started_ns),
               static_cast<unsigned long long>(master->faults.sdo_block_ms));
  timespec remaining{static_cast<time_t>(master->faults.sdo_block_ms / 1000),
                     static_cast<long>((master->faults.sdo_block_ms % 1000) * 1000000)};
  while (nanosleep(&remaining, &remaining) != 0 && errno == EINTR) {}
}
inline int ecrt_master_sdo_download(ec_master_t* m,uint16_t pos,uint16_t idx,uint8_t sub,const uint8_t* data,size_t size,uint32_t* abort) {
  // Block only a Home offset download after activation; startup SDOs and the
  // cyclic plant remain independent, preventing a fake Stop success on unblock.
  if (m->active && pos == 0 && idx == 0x607c && m->faults.sdo_block_ns) {
    timespec now{};
    clock_gettime(CLOCK_MONOTONIC, &now);
    const uint64_t start = uint64_t(now.tv_sec) * 1000000000 + now.tv_nsec;
    std::fprintf(stderr, "sim_sdo_block time_ns=%llu\n", static_cast<unsigned long long>(start));
    timespec delay{static_cast<time_t>(m->faults.sdo_block_ns / 1000000000),
                   static_cast<long>(m->faults.sdo_block_ns % 1000000000)};
    while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
    clock_gettime(CLOCK_MONOTONIC, &now);
    std::fprintf(stderr, "sim_sdo_return time_ns=%llu\n",
                 static_cast<unsigned long long>(uint64_t(now.tv_sec) * 1000000000 + now.tv_nsec));
  }
  m->sdo_delay_armed = true;
  sim_block_sdo(m);
  if (m->active && m->faults.mailbox_cycle_sends) sim_mailbox_wait(m->faults.mailbox_cycle_sends, idx, sub);
  for(auto& s:m->slaves) if(s->position==pos) {
    std::lock_guard<std::mutex> lock(s->mailbox_mutex);
    s->sdos[(uint32_t(idx)<<8)|sub]={data,data+size};
    if (idx == 0x607c && sub == 0 && size == 4) s->home_offset_counts.store(EC_READ_S32(data));
    if (idx == 0x6098 && sub == 0 && size == 1) s->home_method.store(int8_t(*data));
    *abort=0; return 0;
  }
  *abort=0x06020000; return -1;
}
// A6-EC identity, nine-motor bench SII 2026-09-13 (config/drives/a6ec.json). The
// drive's own dictionary is keyed by the identity, as a real drive's is by its
// firmware; every other identity keeps the plain plant, which answers only
// objects written to it. `sdos` still holds only host writes.
constexpr uint32_t kSimA6ecVendorId = 0x00400000, kSimA6ecProductCode = 0x0715;
// A real A6-EC answers its persisted Home offset 0x607C:00 (I32) before any
// download; it survives drive power loss (docs/drives/a6ec/ethercat/bringup.md)
// and is the first read of the startup Home-offset readback that qualifies a
// retained Home (docs/history/2026-09-23-fresh-home-completion.md). Without it
// that readback is refused every checkpoint. A new simulated drive has never
// been homed: the stored value is zero, its plant starts in that same frame,
// and statusword bits 12/15 stay clear until HM35 completes, since writing
// 607C alone establishes no Home (vendor reply 2026-04-15).
inline bool sim_a6ec_home_offset(const ec_slave_config_t& s, uint16_t idx, uint8_t sub) {
  return s.vendor == kSimA6ecVendorId && s.product == kSimA6ecProductCode && idx == 0x607c && sub == 0 &&
      !s.sdos.count(uint32_t(idx) << 8);
}
inline int ecrt_master_sdo_upload(ec_master_t* m,uint16_t pos,uint16_t idx,uint8_t sub,uint8_t* data,size_t capacity,size_t* size,uint32_t* abort) {
  sim_block_sdo(m);
  if (m->active && m->faults.mailbox_cycle_sends) sim_mailbox_wait(m->faults.mailbox_cycle_sends, idx, sub);
  if (pos == 0 && idx == 0x1c12 && sub == 0 && m->faults.mapping_upload_failures) {
    --m->faults.mapping_upload_failures;
    *abort = 0x05040000; *size = 0; return -1; // CoE SDO protocol timeout.
  }
  for(auto& s:m->slaves) if(s->position==pos) {
    std::lock_guard<std::mutex> lock(s->mailbox_mutex);
    const bool raw_multi = s->raw_multi_turn_encoder_source && (sub == 0x21 || sub == 0x23);
    if (idx == 0x2040 && capacity >= 4 && raw_multi) {
      const auto upload = ++s->encoder_uploads;
      const auto from = m->faults.sample_unavailable_from_upload;
      if (from && upload >= from && (!m->faults.sample_unavailable_for_uploads ||
          upload - from < m->faults.sample_unavailable_for_uploads)) {
        *abort = 0x05040000; *size = 0; return -1;
      }
      if (m->faults.sample_upload_delay_ns) {
        if (m->sample_advance_ns) m->sample_advance_ns(m->faults.sample_upload_delay_ns);
        else {
          timespec delay{static_cast<time_t>(m->faults.sample_upload_delay_ns / 1000000000),
                        static_cast<long>(m->faults.sample_upload_delay_ns % 1000000000)};
          while (nanosleep(&delay, &delay) != 0 && errno == EINTR) {}
        }
      }
      int64_t counts = s->raw_multi_turn_encoder_counts.load(std::memory_order_acquire);
      if (m->faults.sample_contradiction_from_upload && upload >= m->faults.sample_contradiction_from_upload)
        counts += m->faults.sample_contradiction_counts;
      EC_WRITE_U32(data, sub != 0x23 ? uint32_t(counts) : uint32_t(uint64_t(counts) >> 32));
      if (sub == 0x21 && ++s->encoder_low_reads == m->faults.sample_zero_crossing_low_read) {
        s->raw_multi_turn_encoder_counts.store(counts < 0 ? 0 : -1, std::memory_order_release);
      }
      *size = 4; *abort = 0; return 0;
    }
    if (capacity >= 4 && sim_a6ec_home_offset(*s, idx, sub)) {
      EC_WRITE_S32(data, s->home_offset_counts.load());
      *size = 4; *abort = 0; return 0;
    }
    // Read-only A6-EC encoder-loss flags (manual chapter 11: U41.05, C10.15).
    const bool encoder_alarm = s->vendor == kSimA6ecVendorId && s->product == kSimA6ecProductCode &&
        ((idx == 0x2041 && sub == 0x06) || (idx == 0x2010 && sub == 0x16)) && !s->sdos.count((uint32_t(idx) << 8) | sub);
    if (encoder_alarm && capacity >= 2) {
      if (m->faults.encoder_alarm_unreadable) { *abort = 0x06020000; *size = 0; return -1; }
      EC_WRITE_U16(data, idx == 0x2041 ? m->faults.encoder_fault_state : m->faults.multi_turn_overflow);
      *size = 2; *abort = 0; return 0;
    }
    auto it=s->sdos.find((uint32_t(idx)<<8)|sub);
    if(it==s->sdos.end() || it->second.size()>capacity) break;
    *size=it->second.size(); std::memcpy(data,it->second.data(),*size); *abort=0; return 0;
  }
  *abort=0x06020000; *size=0; return -1;
}
inline int ecrt_slave_config_sdo16(ec_slave_config_t* s,uint16_t idx,uint8_t sub,uint16_t value) {
  std::lock_guard<std::mutex> lock(s->mailbox_mutex);
  std::vector<uint8_t> bytes(2); EC_WRITE_U16(bytes.data(),value); s->sdos[(uint32_t(idx)<<8)|sub]=bytes;
  return 0;
}
inline int ecrt_slave_config_sdo(ec_slave_config_t* s,uint16_t idx,uint8_t sub,const uint8_t* data,size_t size) {
  std::lock_guard<std::mutex> lock(s->mailbox_mutex);
  s->sdos[(uint32_t(idx)<<8)|sub] = std::vector<uint8_t>(data,data+size);
  return 0;
}
inline int ecrt_slave_config_sync_manager(ec_slave_config_t*,uint8_t,ec_direction_t,ec_watchdog_mode_t) { return -1; }
inline int ecrt_slave_config_pdo_assign_clear(ec_slave_config_t*,uint8_t) { return -1; }
inline int ecrt_slave_config_pdo_assign_add(ec_slave_config_t*,uint8_t,uint16_t) { return -1; }
inline int ecrt_slave_config_dc(ec_slave_config_t* s,uint16_t,uint32_t cycle,int32_t shift,uint32_t,int32_t) { s->sync0_cycle_ns = cycle; s->sync0_shift_ns = shift; return 0; }
inline int ecrt_master_application_time(ec_master_t* m,uint64_t ns) { m->application_time_ns = ns; return 0; }
inline int ecrt_master_set_send_interval(ec_master_t*,size_t) { return 0; }
inline int ecrt_master_sync_reference_clock(ec_master_t* m) { m->reference_sync_time_ns = m->application_time_ns; return 0; }
inline int ecrt_master_sync_reference_clock_to(ec_master_t* m,uint64_t ns) { ++m->reference_sync_calls; m->reference_sync_time_ns = ns; return 0; }
inline int ecrt_master_sync_slave_clocks(ec_master_t* m) {
  ++m->dc_calls;
  sim_report_fault_edge("dc", m->dc_calls, m->faults.dc_fail_from_call, m->faults.dc_fail_for_calls);
  if (sim_fault_on_call(m->dc_calls, m->faults.dc_fail_from_call,
                       m->faults.dc_fail_for_calls)) return m->faults.dc_error;
  return 0;
}
inline int ecrt_master_reset(ec_master_t*) { return 0; }
