#pragma once

// IgH 1.6.9 LRW counts two output increments plus one input increment per axis;
// the simulator receipt counts two directions. Held devices contribute zero.
#ifdef ROSIE_RT_SIMULATED_BUS
#define ROSIE_BUS_WKC_LOG "wkc simulator %u/%u: two directions per axis, held devices zero"
#define ROSIE_BUS_WKC_BACKEND "simulator"
#else
#define ROSIE_BUS_WKC_LOG "wkc live lrw %u/%u: two output plus one input increment per axis, held devices zero"
#define ROSIE_BUS_WKC_BACKEND "live lrw"
#endif

// Included inside the daemon namespace after configuration and logging helpers.
// Read the actual assignment and every mapped entry before exposing any PDO as
// configuration-verified. Upload failure is refusal, never an assumed match.
enum class MappingReadback { Matched, Unavailable, Mismatch };

// Unverified retry budget: three complete attempts, then fail closed. A
// transient unavailable upload cannot masquerade as an observed mismatch.
struct MappingVerification {
  unsigned attempts = 0;
  bool verified = false, faulted = false;
  bool pending() const { return !verified && !faulted; }
  void observe(MappingReadback result) {
    if (!pending()) return;
    ++attempts;
    verified = result == MappingReadback::Matched;
    faulted = result == MappingReadback::Mismatch || (!verified && attempts >= 3);
  }
};

#if GRADIENT_HAVE_ECRT
#include "engine/bus_topology.hpp"
struct SdoReadIdentity {
  uint32_t axis;
  uint16_t position, index;
  uint8_t subindex;
  uint64_t sequence;
  bool operator==(const SdoReadIdentity& other) const {
    return axis == other.axis && position == other.position && index == other.index &&
        subindex == other.subindex && sequence == other.sequence;
  }
};

struct SdoReadReply {
  SdoReadIdentity identity{};
  std::array<uint8_t, sizeof(uint32_t)> bytes{};
  size_t size = 0;
  uint32_t abort = 0;
  int error = -EIO;
};

// IgH 1.6.9 master/master.c creates a separate blocking request per upload,
// waits for BUSY to finish, then clears it; master/fsm_coe.c checks index/subindex
// on that slave's response. The API returns no wire sequence. This local receipt
// binds that synchronous invocation, not an invented EtherCAT sequence. Never
// detach a timed-out call or stamp a shared payload with the next caller's ID.
inline SdoReadReply upload_sdo_read(ec_master_t* master, const SdoReadIdentity& identity) {
  SdoReadReply reply;
  reply.identity = identity;
  reply.error = ecrt_master_sdo_upload(master, identity.position, identity.index,
      identity.subindex, reply.bytes.data(), reply.bytes.size(), &reply.size, &reply.abort);
  return reply;
}

struct MappingDifference {
  uint16_t index = 0;
  uint8_t subindex = 0;
  uint32_t expected = 0, observed = 0;
};

// Read the whole configured chain, even after a difference. Missing or stale
// responses invalidate the attempt and must never turn a partial proof into a
// mismatch. Loop bounds come only from the profile, never from drive counts.
template<class Read>
MappingReadback verify_pdo_readback(const DrivePdoConfig& pdo, Read&& read,
                                  MappingDifference* difference_out = nullptr) {
  bool available = true, different = false;
  MappingDifference first{};
  if (difference_out) *difference_out = {};
  auto compare = [&](uint16_t index, uint8_t subindex, unsigned bits, uint32_t expected) {
    uint32_t value = 0;
    if (!read(index, subindex, bits, value)) {
      available = false;
    } else if (value != expected && !different) {
      different = true;
      first = {index, subindex, expected, value};
    }
  };
  auto direction = [&](uint16_t assignment, uint16_t index, const auto& entries) {
    compare(assignment, 0, 8, 1);
    compare(assignment, 1, 16, index);
    compare(index, 0, 8, entries.size());
    for (size_t i = 0; i < entries.size(); ++i) {
      const auto& entry = entries[i];
      const uint32_t expected = (uint32_t(entry.index) << 16) |
          (uint32_t(entry.subindex) << 8) | entry.bits;
      compare(index, static_cast<uint8_t>(i + 1), 32, expected);
    }
  };
  // CoE assignment objects, ETG.1000; SM2 outputs and SM3 inputs on A6-EC.
  direction(0x1c10 + pdo.rx_sync_index, pdo.rx_pdo, pdo.rx_entries);
  direction(0x1c10 + pdo.tx_sync_index, pdo.tx_pdo, pdo.tx_entries);
  if (!available) return MappingReadback::Unavailable;
  if (different && difference_out) *difference_out = first;
  return different ? MappingReadback::Mismatch : MappingReadback::Matched;
}

#include "engine/ethercat_preop.hpp"

// SII identifies the configured slave revision; CoE firmware revision is a
// separate anchor identity and must not qualify a mismatching SII revision.
inline bool verify_sii_revision(ec_master_t* master, uint16_t position, uint32_t revision) {
  ec_slave_info_t info{};
  return master && ecrt_master_get_slave(master, position, &info) == 0 && info.revision_number == revision;
}

// Keep registered offsets, PDO descriptors and checked cyclic handles at stable
// addresses until master teardown. Copying would leave brake/PDO pointers stale.
struct BusHandles {
  BusTopologyMonitor topology;
  rosie::rtcore::CellIOWire cell_io_wire;
  ec_slave_config_t* io_sc=nullptr;

  // Register only after read-only identity, complete mapping and scanned-map
  // agreement; an output echo is never registered as input feedback.
  bool configure_cell_io(ec_domain_t* target_domain) {
    const auto& c=opt.cell_io; if(!c.enabled()) return true;
    if(c.backend==rosie::rtcore::IoBackend::picontrol) {
      if(cell_io_wire.picontrol.open(c)) return true;
      logf("ERROR: cell I/O piControl device %s cannot be opened: %s", c.device.c_str(), std::strerror(errno));
      return false;
    }
    for(uint32_t i=0;i<opt.num_axes;++i) if(opt.slave_position[i]==c.position) return false;
    io_sc=ecrt_master_slave_config(master,0,c.position,c.vendor,c.product);
    if(!io_sc || !target_domain) return false;
    if(c.schema>=2) return configure_mailbox_free_terminal(target_domain);
    DrivePdoConfig pdo;
    pdo.rx_pdo=c.rx_pdo; pdo.tx_pdo=c.tx_pdo; pdo.rx_sync_index=c.rx_sync; pdo.tx_sync_index=c.tx_sync;
    // Unverified fixed terminal layout: one byte per direction, no remapping.
    pdo.rx_entries={{"outputs",c.output_index,c.output_subindex,8}};
    pdo.tx_entries={{"inputs",c.input_index,c.input_subindex,8}};
#if ROSIE_RT_SIMULATION
    ec_pdo_entry_info_t output{c.output_index,c.output_subindex,8}, input{c.input_index,c.input_subindex,8};
    ec_pdo_info_t rx{c.rx_pdo,1,&output},tx{c.tx_pdo,1,&input};
    ec_sync_info_t syncs[]={{c.rx_sync,EC_DIR_OUTPUT,1,&rx,EC_WD_ENABLE},{c.tx_sync,EC_DIR_INPUT,1,&tx,EC_WD_DISABLE},{0xff,EC_DIR_INVALID,0,nullptr,EC_WD_DISABLE}};
    if(ecrt_slave_config_pdos(io_sc,EC_END,syncs)) return false;
    io_sc->sii_revision=c.revision;
    io_sc->dio=true; io_sc->dio_input_index=c.input_index; io_sc->dio_input_subindex=c.input_subindex;
    io_sc->dio_output_index=c.output_index; io_sc->dio_output_subindex=c.output_subindex;
    for(uint32_t i=0;i<c.output_count;++i) if(c.outputs[i].wired) {
      io_sc->dio_feedback_mask[c.outputs[i].bit]=1u<<c.outputs[i].readback_bit;
      if(!c.outputs[i].readback_high) io_sc->dio_invert_mask|=1u<<c.outputs[i].readback_bit;
    }
#else
    if(ecrt_slave_config_sync_manager(io_sc,c.rx_sync,EC_DIR_OUTPUT,EC_WD_ENABLE) ||
       ecrt_slave_config_sync_manager(io_sc,c.tx_sync,EC_DIR_INPUT,EC_WD_DISABLE)) return false;
#endif
    if(!verify_sii_revision(master,c.position,c.revision)) return false;
    MappingVerification verified;
    while(verified.pending()) {
      verified.observe(verify_pdo_readback(pdo,[&](uint16_t index,uint8_t sub,unsigned bits,uint32_t& value) {
        auto reply=upload_sdo_read(master,{0,c.position,index,sub,0});
        if(reply.error || reply.abort || reply.size!=bits/8) return false;
        value=bits==8?EC_READ_U8(reply.bytes.data()):bits==16?EC_READ_U16(reply.bytes.data()):EC_READ_U32(reply.bytes.data()); return true;
      }));
    }
    MappingDifference difference{};
    if(!verified.verified || verify_scanned_pdo(master,c.position,pdo,difference)!=MappingReadback::Matched) return false;
    std::array<ec_pdo_entry_reg_t,3> regs{};
    regs[0]={0,c.position,c.vendor,c.product,c.input_index,c.input_subindex,&cell_io_wire.input_offsets[0],nullptr};
    regs[1]={0,c.position,c.vendor,c.product,c.output_index,c.output_subindex,&cell_io_wire.output_offsets[0],nullptr};
    return ecrt_domain_reg_pdo_entry_list(target_domain,regs.data())==0;
  }

  // Schema 2 terminal without a CoE mailbox (PLANET IECS-1116-DO): no SDO can
  // read its assignment, so the declared fixed PDOs are configured explicitly
  // and must agree with the SII-scanned sync managers before registration. An
  // SII without PDO categories is refused, never assumed to match.
  bool configure_mailbox_free_terminal(ec_domain_t* target_domain) {
    const auto& c=opt.cell_io;
    std::array<ec_pdo_entry_info_t,2*rosie::rtcore::io_wire_bytes> entries{};
    std::array<ec_pdo_info_t,2*rosie::rtcore::io_wire_bytes> pdos{};
    std::array<ec_sync_info_t,2*rosie::rtcore::io_wire_bytes+1> syncs{};
    size_t n=0;
    auto declare=[&](const rosie::rtcore::IoPdoConfig& p, ec_direction_t direction) {
      entries[n]={p.index,p.subindex,8};
      pdos[n]={p.pdo,1,&entries[n]};
      syncs[n]={p.sync,direction,1,&pdos[n],direction==EC_DIR_OUTPUT ? EC_WD_ENABLE : EC_WD_DISABLE};
      ++n;
    };
    for(uint32_t i=0;i<c.output_pdo_count;++i) declare(c.output_pdos[i],EC_DIR_OUTPUT);
    for(uint32_t i=0;i<c.input_pdo_count;++i) declare(c.input_pdos[i],EC_DIR_INPUT);
    syncs[n]={0xff,EC_DIR_INVALID,0,nullptr,EC_WD_DISABLE};
    if(ecrt_slave_config_pdos(io_sc,EC_END,syncs.data())) return false;
#if ROSIE_RT_SIMULATION
    io_sc->sii_revision=c.revision;
    io_sc->dio=true;
    io_sc->dio_output_index=c.output_pdos[0].index; io_sc->dio_output_subindex=c.output_pdos[0].subindex;
    if(c.output_pdo_count>1) { io_sc->dio_output_high_index=c.output_pdos[1].index; io_sc->dio_output_high_subindex=c.output_pdos[1].subindex; }
    io_sc->dio_input_index=c.input_pdo_count ? c.input_pdos[0].index : 0;
    io_sc->dio_input_subindex=c.input_pdo_count ? c.input_pdos[0].subindex : 0;
    if(c.input_pdo_count>1) { io_sc->dio_input_high_index=c.input_pdos[1].index; io_sc->dio_input_high_subindex=c.input_pdos[1].subindex; }
    for(uint32_t i=0;i<c.output_count;++i) if(c.outputs[i].wired) {
      io_sc->dio_feedback_mask[c.outputs[i].bit]=uint16_t(1u<<c.outputs[i].readback_bit);
      if(!c.outputs[i].readback_high) io_sc->dio_invert_mask|=uint16_t(1u<<c.outputs[i].readback_bit);
    }
#endif
    if(!verify_sii_revision(master,c.position,c.revision)) return false;
    for(size_t i=0;i<n;++i) {
      ec_sync_info_t sync{}; ec_pdo_info_t scanned{}; ec_pdo_entry_info_t entry{};
      if(ecrt_master_get_sync_manager(master,c.position,syncs[i].index,&sync) || sync.n_pdos!=1 ||
         ecrt_master_get_pdo(master,c.position,syncs[i].index,0,&scanned) ||
         scanned.index!=pdos[i].index || scanned.n_entries!=1 ||
         ecrt_master_get_pdo_entry(master,c.position,syncs[i].index,0,0,&entry) ||
         entry.index!=entries[i].index || entry.subindex!=entries[i].subindex || entry.bit_length!=8) {
        logf("ERROR: cell I/O terminal position=%u sync=%u: SII-scanned PDO differs from profile pdo=0x%04x entry=0x%04x:%u (scanned n_pdos=%u pdo=0x%04x entry=0x%04x:%u bits=%u)",
             unsigned(c.position), unsigned(syncs[i].index), unsigned(pdos[i].index), unsigned(entries[i].index),
             unsigned(entries[i].subindex), unsigned(sync.n_pdos), unsigned(scanned.index), unsigned(entry.index),
             unsigned(entry.subindex), unsigned(entry.bit_length));
        return false;
      }
    }
    std::array<ec_pdo_entry_reg_t,2*rosie::rtcore::io_wire_bytes+1> regs{};
    size_t r=0;
    for(uint32_t i=0;i<c.output_pdo_count;++i)
      regs[r++]={0,c.position,c.vendor,c.product,c.output_pdos[i].index,c.output_pdos[i].subindex,&cell_io_wire.output_offsets[i],nullptr};
    for(uint32_t i=0;i<c.input_pdo_count;++i)
      regs[r++]={0,c.position,c.vendor,c.product,c.input_pdos[i].index,c.input_pdos[i].subindex,&cell_io_wire.input_offsets[i],nullptr};
    return ecrt_domain_reg_pdo_entry_list(target_domain,regs.data())==0;
  }
  const Options& opt;
  const std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES>& axis_profiles;
  std::atomic<ec_master_t*>& master;
  ec_domain_t* domain = nullptr;
  std::array<ec_domain_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain{};
  uint8_t* domain_pd = nullptr;
  rosie::rtcore::CycleBus cycle_backend;
  std::array<uint8_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> axis_domain_pd{};

  static constexpr unsigned int kInvalidOffset = std::numeric_limits<unsigned int>::max();
  static constexpr size_t kMaxPdoRegsPerAxis = 32;

  struct AxisOffsets {
    unsigned int cw = kInvalidOffset;
    unsigned int target_pos = kInvalidOffset;
    unsigned int target_vel = kInvalidOffset;
    unsigned int target_torque = kInvalidOffset;
    unsigned int mode = kInvalidOffset;
    unsigned int tp_func = kInvalidOffset;
    unsigned int max_profile_vel = kInvalidOffset;

    unsigned int err = kInvalidOffset;
    unsigned int manufacturer_err = kInvalidOffset;
    unsigned int sw = kInvalidOffset;
    unsigned int pos = kInvalidOffset;
    unsigned int continuous_pos = kInvalidOffset;
    unsigned int continuous_valid = kInvalidOffset;
    unsigned int torque = kInvalidOffset;
    unsigned int velocity_actual = kInvalidOffset;
    unsigned int following_error = kInvalidOffset;
    unsigned int mode_disp = kInvalidOffset;
    unsigned int tp_status = kInvalidOffset;
    unsigned int tp_pos1 = kInvalidOffset;
    unsigned int tp_pos2 = kInvalidOffset;
    unsigned int di = kInvalidOffset;

    // Phase 1 (2026-04-20): extended A6-EC 0x2040 PDO subitems. Left
    // kInvalidOffset until the drive profile's tx_pdo_layout registers
    // them; per-axis feedback reads below skip anything still invalid.
    unsigned int ext_bus_voltage = kInvalidOffset;         // 0x2040:0x07
    unsigned int ext_load_rate = kInvalidOffset;           // 0x2040:0x08
    unsigned int ext_position_error = kInvalidOffset;      // 0x2040:0x11
    unsigned int ext_multi_turn_lo = kInvalidOffset;       // 0x2040:0x21
    unsigned int ext_multi_turn_hi = kInvalidOffset;       // 0x2040:0x23
    unsigned int ext_igbt_temp = kInvalidOffset;           // 0x2040:0x31
    unsigned int ext_motor_temp = kInvalidOffset;          // 0x2040:0x32
    unsigned int ext_drive_not_ready = kInvalidOffset;     // 0x2040:0x43
    unsigned int ext_motor_not_rotating = kInvalidOffset;  // 0x2040:0x44
  };

  std::array<ec_slave_config_t*, gradient::ipc::v1::GRADIENT_MAX_AXES> sc{};
  std::array<AxisOffsets, gradient::ipc::v1::GRADIENT_MAX_AXES> off{};
  std::array<unsigned int*, gradient::ipc::v1::GRADIENT_MAX_AXES> brake_signal_offsets{};
  struct PdoRuntime {
    std::vector<ec_pdo_entry_info_t> rx, tx;
    std::array<ec_pdo_info_t, 2> pdos{};
    std::array<ec_sync_info_t, 3> syncs{};
  };
  std::array<PdoRuntime, gradient::ipc::v1::GRADIENT_MAX_AXES> pdo_runtime{};
  std::array<OriginalPdoAssignments, gradient::ipc::v1::GRADIENT_MAX_AXES> original_assignment{};
  bool ecrt_ok = false;
  bool master_activated = false;
  bool stop_cycle = false;
  const uint32_t expected_wkc;
  uint64_t startup_begin_ns = 0;
  uint64_t startup_cycle_counter = 0;

  BusHandles(const BusHandles&) = delete;
  BusHandles& operator=(const BusHandles&) = delete;
  BusHandles(BusHandles&&) = delete;
  BusHandles& operator=(BusHandles&&) = delete;

  // Called with the mailbox teardown mutex held after cyclic output stops.
  // Deactivate before restoring: changing an active process image is forbidden.
  void restore_assignments() {
    bool needed = false;
    for (uint32_t i = 0; i < opt.num_axes; ++i)
      needed |= axis_profiles[i].pdo.restore_assignment_on_exit &&
          (original_assignment[i].tx.changed || original_assignment[i].rx.changed);
    if (!master || !needed) return;
    if (master_activated && ecrt_master_deactivate(master) != 0) {
      logf("ERROR: pdo_assignment restore=unavailable phase=deactivate");
      return;
    }
    ecrt_ok = false;
    master_activated = false;
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      const auto& pdo = axis_profiles[i].pdo;
      for (bool rx : {false, true}) {
        auto& original = rx ? original_assignment[i].rx : original_assignment[i].tx;
        if (!pdo.restore_assignment_on_exit || !original.changed) continue;
        MappingVerification progress;
        while (progress.pending()) {
          uint32_t count = 0, preset = 0;
          const uint16_t assignment = 0x1c10 + (rx ? pdo.rx_sync_index : pdo.tx_sync_index);
          const bool available = wait_pdo_preop(master, opt.slave_position[i], opt.preop_to_safeop_timeout_ms, true) &&
              write_preop_assignment(master, i, opt.slave_position[i], assignment, original.preset) &&
              read_preop_pdo(master, opt.slave_position[i], assignment, 0, 8, count) &&
              read_preop_pdo(master, opt.slave_position[i], assignment, 1, 16, preset);
          progress.observe(!available ? MappingReadback::Unavailable :
              count == 1 && preset == original.preset ? MappingReadback::Matched : MappingReadback::Mismatch);
        }
        logf("EtherCAT pdo_assignment axis=%u phase=PREOP restore=%s preset=0x%04x attempts=%u rescan_required=1",
             i, progress.verified ? "verified" : "failed", original.preset, progress.attempts);
        original.changed = false;
      }
    }
  }

  auto aggregate_domain_state(ec_domain_state_t* out) {
    auto result = rosie::rtcore::checked_bus_result(rosie::rtcore::BusOperation::DomainState, 0);
    if (!out) {
      return rosie::rtcore::checked_bus_result(rosie::rtcore::BusOperation::DomainState, -EINVAL);
    }
    *out = ec_domain_state_t{};
    bool have_domain_state = false;
    auto merge_domain_state = [&](ec_domain_t* dom) {
      if (!dom) {
        return;
      }
      ec_domain_state_t cur{};
      const auto query = rosie::rtcore::checked_bus_result(
          rosie::rtcore::BusOperation::DomainState, ecrt_domain_state(dom, &cur));
      if (query.failed() && !result.failed()) result = query;
      if (!have_domain_state || cur.wc_state < out->wc_state) {
        out->wc_state = cur.wc_state;
      }
      out->working_counter += cur.working_counter;
      have_domain_state = true;
    };
    if (opt.split_domains_per_axis) {
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        merge_domain_state(axis_domain[i]);
      }
    } else {
      merge_domain_state(domain);
    }
    return result;
  }

  void log_phase_summary(const char* phase, uint64_t elapsed_ns, uint64_t cycle) {
    if (!master) {
      return;
    }
    ec_master_state_t phase_master_state{};
    ecrt_master_state(master, &phase_master_state);
    ec_domain_state_t phase_domain_state{};
    (void)aggregate_domain_state(&phase_domain_state);

    std::array<ec_slave_config_state_t, gradient::ipc::v1::GRADIENT_MAX_AXES> phase_sc_state{};
    uint32_t phase_online_slaves = 0;
    uint32_t phase_operational_slaves = 0;
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      if (sc[i] && ecrt_slave_config_state(sc[i], &phase_sc_state[i]) == 0) {
        if (phase_sc_state[i].online) {
          phase_online_slaves++;
        }
        if (phase_sc_state[i].operational) {
          phase_operational_slaves++;
        }
      }
    }

    logf("EtherCAT phase=%s cycle=%llu elapsed_ms=%llu link_up=%u responding=%u/%u online=%u/%u "
         "operational=%u/%u master_al=0x%x domain_wc=%u " ROSIE_BUS_WKC_LOG,
         phase,
         static_cast<unsigned long long>(cycle),
         static_cast<unsigned long long>(elapsed_ns / 1000000ULL),
         phase_master_state.link_up ? 1u : 0u,
         phase_master_state.slaves_responding,
         topology.snapshot.responding_expected ? topology.snapshot.responding_expected : opt.num_axes,
         phase_online_slaves,
         opt.num_axes,
         phase_operational_slaves,
         opt.num_axes,
         phase_master_state.al_states,
         static_cast<unsigned int>(phase_domain_state.wc_state),
         static_cast<unsigned int>(phase_domain_state.working_counter),
         expected_wkc);
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      if (!sc[i]) {
        continue;
      }
      logf("EtherCAT phase=%s axis=%u slave_pos=%u online=%u operational=%u al=%s(0x%x)",
           phase,
           i,
           static_cast<unsigned int>(opt.slave_position[i]),
           phase_sc_state[i].online ? 1u : 0u,
           phase_sc_state[i].operational ? 1u : 0u,
           al_state_label(static_cast<uint8_t>(phase_sc_state[i].al_state)),
           static_cast<unsigned int>(phase_sc_state[i].al_state));
    }
  }

  // Configure live and simulated buses through the same descriptor path.
  // Preserve early termination separately from a nonactive bus so startup
  // failures retain their original process status and cyclic cleanup behavior.
  template<class Feedback>
  BusHandles(const Options& options, const OptionsParseResult& parsed,
             std::atomic<ec_master_t*>& shared_master, Feedback& latest_feedback,
             std::atomic<int>& process_exit_code)
      : opt(options), axis_profiles(parsed.axis_profiles), master(shared_master),
        // IgH 1.6.9 master/datagram_pair.c: LRW adds 2 per output FMMU
        // and 1 per input FMMU. The simulator's existing receipt model uses
        // two directions per configured axis. Held slaves add zero; terminal contributions are unverified on hardware.
        // A terminal has one FMMU per PDO sync manager; piControl I/O is not on the bus.
#ifdef ROSIE_RT_SIMULATED_BUS
        expected_wkc(2 * (opt.num_axes + (opt.cell_io.on_bus() ? 1u : 0u))) {
#else
        expected_wkc(3 * opt.num_axes + (opt.cell_io.on_bus() ?
            2 * opt.cell_io.output_pdo_count + opt.cell_io.input_pdo_count : 0u)) {
#endif
    const auto& startup_sdos = parsed.startup_sdos;
#if ROSIE_RT_SIMULATION
    const auto& sim_faults = parsed.sim_faults;
#endif
    const uint64_t period = opt.cycle_ns;
    const uint32_t configured_mask = (1u << opt.num_axes) - 1;
    constexpr int kExitCodeMasterReservationFailed = 75;
    for (uint32_t i = 0; i < opt.num_axes; ++i) {
      const auto& p = axis_profiles[i].pdo;
      auto& r = pdo_runtime[i];
      for (const auto& e : p.rx_entries) r.rx.push_back({e.index, e.subindex, e.bits});
      for (const auto& e : p.tx_entries) r.tx.push_back({e.index, e.subindex, e.bits});
      r.pdos = {{{p.rx_pdo, static_cast<unsigned int>(r.rx.size()), r.rx.data()},
                 {p.tx_pdo, static_cast<unsigned int>(r.tx.size()), r.tx.data()}}};
      r.syncs = {{{p.rx_sync_index, EC_DIR_OUTPUT, 1, &r.pdos[0],
                  opt.disable_output_watchdog ? EC_WD_DISABLE : EC_WD_ENABLE},
                 {p.tx_sync_index, EC_DIR_INPUT, 1, &r.pdos[1], EC_WD_DISABLE},
                 {0xff, EC_DIR_INVALID, 0, nullptr, EC_WD_DISABLE}}};
    }

    auto log_registered_offsets = [&]() {
      for (uint32_t i = 0; i < opt.num_axes; ++i) {
        if (!sc[i]) {
          continue;
        }
        logf("EtherCAT phase=pdo_register axis=%u slave_pos=%u offsets cw=%u target_pos=%u sw=%u pos=%u err=%u mfg_err=%u di=%u",
             i,
             static_cast<unsigned int>(opt.slave_position[i]),
             off[i].cw,
             off[i].target_pos,
             off[i].sw,
             off[i].pos,
             off[i].err,
             off[i].manufacturer_err,
             off[i].di);
        // Phase 1 (2026-04-20): log the extended-PDO offsets separately
        // so operators can tell from RTCore boot log which drives accepted
        // the extended 0x2040 mapping. Any value of kInvalidOffset means
        // the catalog did not request that subitem or the drive rejected
        // its registration; Python side falls back to the SDO poll for
        // such axes.
        logf("EtherCAT phase=pdo_register axis=%u ext offsets pe=%u mt_lo=%u mt_hi=%u vbus=%u load=%u igbt=%u motor=%u dnr=%u mnr=%u",
             i,
             off[i].ext_position_error,
             off[i].ext_multi_turn_lo,
             off[i].ext_multi_turn_hi,
             off[i].ext_bus_voltage,
             off[i].ext_load_rate,
             off[i].ext_igbt_temp,
             off[i].ext_motor_temp,
             off[i].ext_drive_not_ready,
             off[i].ext_motor_not_rotating);
      }
    };

    auto rx_offset_for_semantic = [&](AxisOffsets& axis_offsets, const std::string& semantic)
        -> unsigned int* {
      if (semantic == "cw") {
        return &axis_offsets.cw;
      }
      if (semantic == "target_pos") {
        return &axis_offsets.target_pos;
      }
      if (semantic == "target_vel") {
        return &axis_offsets.target_vel;
      }
      if (semantic == "target_torque") {
        return &axis_offsets.target_torque;
      }
      if (semantic == "mode") {
        return &axis_offsets.mode;
      }
      if (semantic == "tp_func") {
        return &axis_offsets.tp_func;
      }
      if (semantic == "max_profile_vel") {
        return &axis_offsets.max_profile_vel;
      }
      return nullptr;
    };
    auto tx_offset_for_semantic = [&](AxisOffsets& axis_offsets, const std::string& semantic)
        -> unsigned int* {
      if (semantic == "err") {
        return &axis_offsets.err;
      }
      if (semantic == "manufacturer_err") {
        return &axis_offsets.manufacturer_err;
      }
      if (semantic == "sw") {
        return &axis_offsets.sw;
      }
      if (semantic == "pos") {
        return &axis_offsets.pos;
      }
      if (semantic == "continuous_pos") return &axis_offsets.continuous_pos;
      if (semantic == "continuous_valid") return &axis_offsets.continuous_valid;
      if (semantic == "velocity_actual") return &axis_offsets.velocity_actual;
      if (semantic == "following_error") return &axis_offsets.following_error;
      if (semantic == "torque") {
        return &axis_offsets.torque;
      }
      if (semantic == "mode_disp") {
        return &axis_offsets.mode_disp;
      }
      if (semantic == "tp_status") {
        return &axis_offsets.tp_status;
      }
      if (semantic == "tp_pos1") {
        return &axis_offsets.tp_pos1;
      }
      if (semantic == "tp_pos2") {
        return &axis_offsets.tp_pos2;
      }
      if (semantic == "di") {
        return &axis_offsets.di;
      }
      // Phase 1 (2026-04-20) extended A6-EC diagnostics PDO entries.
      if (semantic == "bus_voltage") {
        return &axis_offsets.ext_bus_voltage;
      }
      if (semantic == "load_rate") {
        return &axis_offsets.ext_load_rate;
      }
      if (semantic == "position_error") {
        return &axis_offsets.ext_position_error;
      }
      if (semantic == "multi_turn_lo") {
        return &axis_offsets.ext_multi_turn_lo;
      }
      if (semantic == "multi_turn_hi") {
        return &axis_offsets.ext_multi_turn_hi;
      }
      if (semantic == "igbt_temp") {
        return &axis_offsets.ext_igbt_temp;
      }
      if (semantic == "motor_temp") {
        return &axis_offsets.ext_motor_temp;
      }
      if (semantic == "drive_not_ready") {
        return &axis_offsets.ext_drive_not_ready;
      }
      if (semantic == "motor_not_rotating") {
        return &axis_offsets.ext_motor_not_rotating;
      }
      return nullptr;
    };

    auto fill_axis_regs = [&](uint32_t axis, auto& regs, size_t& reg_i) {
      const auto& drive_pdo = axis_profiles[axis].pdo;
      const uint16_t slave_pos = opt.slave_position[axis];
      auto add_reg = [&](uint16_t index, uint8_t subindex, unsigned int* offset) {
        regs[reg_i++] = {0,
                         slave_pos,
                         axis_profiles[axis].vendor_id,
                         axis_profiles[axis].product_code,
                         index,
                         subindex,
                         offset,
                         nullptr};
      };
      for (const auto& entry : drive_pdo.rx_entries) {
        if (unsigned int* offset = rx_offset_for_semantic(off[axis], entry.semantic)) {
          add_reg(entry.index, entry.subindex, offset);
        }
      }
      for (const auto& entry : drive_pdo.tx_entries) {
        if (unsigned int* offset = tx_offset_for_semantic(off[axis], entry.semantic)) {
          add_reg(entry.index, entry.subindex, offset);
        }
      }
    };

    bool fixed_mapping_notice_logged = false;
    auto apply_pdo_profile = [&](uint32_t axis, uint16_t slave_pos) {
      const auto& drive_pdo = axis_profiles[axis].pdo;
      const auto& runtime_syncs = pdo_runtime[axis].syncs;
      // The nine-motor bench's measured A6 identity and 372e9b25 robot default preset.
      const bool robot_default = axis_profiles[axis].vendor_id == 4194304 &&
          axis_profiles[axis].product_code == 1813 && drive_pdo.tx_pdo == 0x1b02;
      if (!opt.explicit_pdo_config) {
#if ROSIE_RT_SIMULATION
        // The synthetic slave has no scanned defaults; install its fixture.
        if (ecrt_slave_config_pdos(sc[axis], EC_END, runtime_syncs.data())) {
          logf("ERROR: ecrt_slave_config_pdos failed for axis=%u slave_pos=%u", axis, slave_pos);
          return false;
        }
#else
        // Give IgH no PDO lists: mailbox assignment and both physical/cache
        // comparisons below must finish before entry registration. Never pass
        // fixed preset entries to ecrt_slave_config_pdos/mapping_add.
        // IgH 1.6.9 master/fsm_pdo.c:ec_fsm_pdo_conf_action_check_mapping
        // nevertheless always writes when SII advertises PDO configuration.
        // Its entry FSM stops after a rejected clear; userspace has no opt-out.
        if (robot_default && !fixed_mapping_notice_logged) {
          logf("EtherCAT fixed_mapping kernel=IgH-1.6.9 mapping_write=kernel_unavoidable "
               "registration=scanned_mapping userspace_mapping_writes=0 "
               "bench_abort=0x06010002 bench=2026-09-13 "
               "kernel_clear_attempts=1_per_PDO_per_configuration readback=required");
          fixed_mapping_notice_logged = true;
        }
        for (size_t i = 0; runtime_syncs[i].index != 0xff; ++i) {
          const auto& sync = runtime_syncs[i];
          if (ecrt_slave_config_sync_manager(sc[axis], sync.index, sync.dir, sync.watchdog_mode))
            return false;
        }
#endif
        (void)fixed_mapping_notice_logged;
        logf("EtherCAT config phase=slave_config_pdos axis=%u slave_pos=%u profile=%s mode=fixed_default readback=%s",
             axis,
             slave_pos,
             drive_pdo.label.c_str(), robot_default ? "pending" : "PREOP_required");
        return true;
      }

      for (size_t sync_i = 0; runtime_syncs[sync_i].index != 0xff; ++sync_i) {
        const ec_sync_info_t& sync = runtime_syncs[sync_i];
        if (ecrt_slave_config_sync_manager(sc[axis], sync.index, sync.dir, sync.watchdog_mode)) {
          logf("ERROR: ecrt_slave_config_sync_manager failed for axis=%u slave_pos=%u sync=%u",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync.index));
          return false;
        }
        if (ecrt_slave_config_pdo_assign_clear(sc[axis], sync.index)) {
          logf("ERROR: ecrt_slave_config_pdo_assign_clear failed for axis=%u slave_pos=%u sync=%u",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync.index));
          return false;
        }
        for (unsigned int pdo_i = 0; pdo_i < sync.n_pdos; ++pdo_i) {
          const ec_pdo_info_t& pdo = sync.pdos[pdo_i];
          if (ecrt_slave_config_pdo_assign_add(sc[axis], sync.index, pdo.index)) {
            logf("ERROR: ecrt_slave_config_pdo_assign_add failed for axis=%u slave_pos=%u sync=%u pdo=0x%04x",
                 axis,
                 slave_pos,
                 static_cast<unsigned int>(sync.index),
                 static_cast<unsigned int>(pdo.index));
            return false;
          }
        }
      }

      logf("EtherCAT config phase=slave_config_pdos axis=%u slave_pos=%u profile=%s mode=explicit_assign readback=pending",
           axis,
           slave_pos,
           drive_pdo.label.c_str());
      return true;
    };

    // Write on change: read each object in PREOP and queue only those that
    // differ or cannot be read. A drive that stores SDO writes (A6-EC C13.10)
    // is then written zero times per steady-state start. IgH replays queued
    // objects at every reconfiguration of the slave; a skipped object is
    // already held by the drive. Readback verification is unchanged.
    auto schedule_axis_startup_sdos = [&](uint32_t axis, uint16_t slave_pos) {
      if (std::none_of(startup_sdos.begin(), startup_sdos.end(),
                       [&](const auto& d) { return (d.axis_mask & (1u << axis)) != 0; })) {
        return true;
      }
      std::vector<StartupSdoPrior> prior(startup_sdos.size());
      for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
        const auto& descriptor = startup_sdos[descriptor_i];
        if (!(descriptor.axis_mask & (1u << axis))) continue;
        // Startup SDOs run again during activation. They must not invalidate
        // the PREOP proof by editing a PDO map or assignment behind its owner.
        if ((descriptor.index >= 0x1600 && descriptor.index <= 0x1bff) ||
            (descriptor.index >= 0x1c10 && descriptor.index <= 0x1c2f)) {
          logf("ERROR: pdo_assignment axis=%u startup_sdo_conflict index=0x%04x activation=blocked",
               axis, descriptor.index);
          return false;
        }
        auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
        feedback.configured[axis] = descriptor.valid ? 1u : 0u;
        feedback.commanded[axis] = descriptor.valid ? descriptor.values[axis] : 0u;
        const int64_t value = descriptor.values[axis];
        // A failed, aborted or wrong-width upload is unread and is written.
        const auto reply = upload_sdo_read(master, {axis, slave_pos, descriptor.index, descriptor.subindex, 0});
        if (!reply.error && !reply.abort && reply.size == startup_sdo_size(descriptor.type))
          prior[descriptor_i] = {true, startup_sdo_decode(descriptor.type, reply.bytes.data())};
        if (descriptor.effective_after_power_cycle &&
            (!prior[descriptor_i].read || prior[descriptor_i].value != value)) {
          logf("ERROR: axis=%u slave_pos=%u key=%s power_cycle_effective_mismatch expected=%lld observed=%s activation=blocked",
               axis, slave_pos, descriptor.key.c_str(), static_cast<long long>(value),
               prior[descriptor_i].read ? std::to_string(prior[descriptor_i].value).c_str() : "unread");
          return false;
        }
      }
      const auto plan = plan_axis_startup_sdos(startup_sdos, axis, prior);
      if (plan.refused) {
        logf("ERROR: axis=%u slave_pos=%u startup_trial without parameter_storage activation=blocked", axis, slave_pos);
        return false;
      }
      auto prior_text = [&](size_t d) {
        return prior[d].read ? std::to_string(prior[d].value) : std::string("unread");
      };
      for (const auto& step : plan.steps) {
        const auto& descriptor = startup_sdos[step.descriptor];
        const auto raw = startup_sdo_encode(descriptor.type, step.value);
        const int rc = ecrt_slave_config_sdo(sc[axis], descriptor.index, descriptor.subindex,
                                              raw.data(), startup_sdo_size(descriptor.type));
        if (rc != 0) {
          logf("ERROR: ecrt_slave_config_sdo failed for axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%lld rc=%d",
               axis,
               slave_pos,
               descriptor.key.c_str(),
               static_cast<unsigned int>(descriptor.index),
               static_cast<unsigned int>(descriptor.subindex),
               static_cast<long long>(step.value),
               rc);
          return false;
        }
        const bool window = step.kind == StartupSdoStepKind::kStorageOff || step.kind == StartupSdoStepKind::kStorageOn;
        logf("EtherCAT config phase=slave_config_sdo axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%lld ok %s%s",
             axis,
             slave_pos,
             descriptor.key.c_str(),
             static_cast<unsigned int>(descriptor.index),
             static_cast<unsigned int>(descriptor.subindex),
             static_cast<long long>(step.value),
             window ? (step.kind == StartupSdoStepKind::kStorageOff ? "storage_window=open" : "storage_window=close")
                    : ("prior=" + prior_text(step.descriptor)).c_str(),
             window ? "" : step.kind == StartupSdoStepKind::kRamOnly ? " storage=ram_only" : " storage=stored");
      }
      std::string trial_keys;
      for (size_t d = 0; d < startup_sdos.size(); ++d) {
        const auto& descriptor = startup_sdos[d];
        if ((descriptor.axis_mask & (1u << axis)) && descriptor.ram_only)
          trial_keys += (trial_keys.empty() ? "" : ",") + descriptor.key;
        if (plan.action[d] != StartupSdoAction::kSkippedIdentical) continue;
        logf("EtherCAT config phase=slave_config_sdo axis=%u slave_pos=%u key=%s index=0x%04x sub=0x%02x value=%lld prior=%lld skipped=identical",
             axis,
             slave_pos,
             descriptor.key.c_str(),
             static_cast<unsigned int>(descriptor.index),
             static_cast<unsigned int>(descriptor.subindex),
             static_cast<long long>(descriptor.values[axis]),
             static_cast<long long>(prior[d].value));
      }
      logf("EtherCAT startup_sdo_summary axis=%u slave_pos=%u written=%u skipped=%u persisted=%u ram_only=%u unread=%u storage_window=%u",
           axis, slave_pos, plan.written, plan.skipped, plan.persisted, plan.ram_only, plan.unread,
           plan.storage_window ? 1u : 0u);
      if (!trial_keys.empty())
        logf("WARNING: startup_trial axis=%u slave_pos=%u keys=%s storage=ram_only: these values live in drive RAM only; "
             "a drive power cycle reverts them to the stored values and rt-core writes them again only at its next start",
             axis, slave_pos, trial_keys.c_str());
      return true;
    };

    auto register_axis_offsets_explicit = [&](uint32_t axis, ec_domain_t* target_domain) {
      const auto& drive_pdo = axis_profiles[axis].pdo;
      const uint16_t slave_pos = opt.slave_position[axis];
      auto reg = [&](uint8_t sync_index,
                     unsigned int entry_pos,
                     unsigned int* offset,
                     const char* field_name) {
        const int rc =
            ecrt_slave_config_reg_pdo_entry_pos(sc[axis], sync_index, 0, entry_pos, target_domain, nullptr);
        if (rc < 0) {
          logf("ERROR: ecrt_slave_config_reg_pdo_entry_pos failed for axis=%u slave_pos=%u "
               "sync=%u entry_pos=%u field=%s rc=%d",
               axis,
               slave_pos,
               static_cast<unsigned int>(sync_index),
               entry_pos,
               field_name,
               rc);
          return false;
        }
        *offset = static_cast<unsigned int>(rc);
        return true;
      };

      for (size_t entry_pos = 0; entry_pos < drive_pdo.rx_entries.size(); ++entry_pos) {
        const auto& entry = drive_pdo.rx_entries[entry_pos];
        if (unsigned int* offset = rx_offset_for_semantic(off[axis], entry.semantic)) {
          if (!reg(drive_pdo.rx_sync_index,
                   static_cast<unsigned int>(entry_pos),
                   offset,
                   entry.semantic.c_str())) {
            return false;
          }
        }
      }
      for (size_t entry_pos = 0; entry_pos < drive_pdo.tx_entries.size(); ++entry_pos) {
        const auto& entry = drive_pdo.tx_entries[entry_pos];
        if (unsigned int* offset = tx_offset_for_semantic(off[axis], entry.semantic)) {
          if (!reg(drive_pdo.tx_sync_index,
                   static_cast<unsigned int>(entry_pos),
                   offset,
                   entry.semantic.c_str())) {
            return false;
          }
        }
      }

      return true;
    };

    master = ecrt_request_master(0);
    if (!master) {
      logf("ERROR: ecrt_request_master(0) failed; exit_code=%d (master reservation failed, likely stale owner or hung EtherCAT kernel task)",
           kExitCodeMasterReservationFailed);
      process_exit_code.store(kExitCodeMasterReservationFailed, std::memory_order_relaxed);
      g_stop.store(true, std::memory_order_relaxed);
      stop_cycle = true;
      return;
    } else {
      logf("EtherCAT config phase=request_master ok");
#if ROSIE_RT_SIMULATION
      if (!sim_configure_faults(master, sim_faults)) {
        logf("ERROR: invalid simulation fault controls");
        g_stop.store(true);
        stop_cycle = true;
        return;
      }
#endif
      bool rescanned = false;
      for (;;) {
      bool have_domains = true;
      if (opt.split_domains_per_axis) {
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          axis_domain[i] = ecrt_master_create_domain(master);
          if (!axis_domain[i]) {
            logf("ERROR: ecrt_master_create_domain failed for axis=%u slave_pos=%u",
                 i,
                 static_cast<unsigned int>(opt.slave_position[i]));
            have_domains = false;
            break;
          }
          logf("EtherCAT config phase=create_domain axis=%u slave_pos=%u ok",
               i,
               static_cast<unsigned int>(opt.slave_position[i]));
        }
      } else {
        domain = ecrt_master_create_domain(master);
        if (!domain) {
          logf("ERROR: ecrt_master_create_domain failed");
          have_domains = false;
        } else {
          logf("EtherCAT config phase=create_domain ok");
        }
      }
      if (have_domains) {
        bool slave_config_ok = true;
        // Configure each slave at (alias=0, position=i).
        for (uint32_t i = 0; i < opt.num_axes; ++i) {
          const uint16_t slave_pos = opt.slave_position[i];
          sc[i] = ecrt_master_slave_config(master, 0, slave_pos,
                                           axis_profiles[i].vendor_id,
                                           axis_profiles[i].product_code);
          if (!sc[i]) {
            logf("ERROR: ecrt_master_slave_config failed for axis=%u slave_pos=%u", i, slave_pos);
            slave_config_ok = false;
            break;
          }
          logf("EtherCAT config phase=slave_config axis=%u slave_pos=%u ok", i, slave_pos);
          log_phase_summary("after_slave_config", 0, 0);

          // Give the drive a conservative PLC-like convergence window before SAFEOP/OP.
#ifdef EC_HAVE_FLAGS
          if (opt.wait_before_safeop_ms > 0) {
            const int rc = ecrt_slave_config_flag(
                sc[i], "WaitBeforeSAFEOPms", static_cast<int32_t>(opt.wait_before_safeop_ms));
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_flag(WaitBeforeSAFEOPms=%u) failed for axis=%u slave_pos=%u rc=%d",
                   opt.wait_before_safeop_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
#endif
#ifdef EC_HAVE_STATE_TIMEOUT
          if (opt.preop_to_safeop_timeout_ms > 0) {
            const int rc = ecrt_slave_config_state_timeout(
                sc[i], EC_AL_STATE_PREOP, EC_AL_STATE_SAFEOP, opt.preop_to_safeop_timeout_ms);
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_state_timeout(PREOP->SAFEOP=%u ms) failed for axis=%u slave_pos=%u rc=%d",
                   opt.preop_to_safeop_timeout_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
          if (opt.safeop_to_op_timeout_ms > 0) {
            const int rc = ecrt_slave_config_state_timeout(
                sc[i], EC_AL_STATE_SAFEOP, EC_AL_STATE_OP, opt.safeop_to_op_timeout_ms);
            if (rc != 0) {
              logf("WARNING: ecrt_slave_config_state_timeout(SAFEOP->OP=%u ms) failed for axis=%u slave_pos=%u rc=%d",
                   opt.safeop_to_op_timeout_ms,
                   i,
                   slave_pos,
                   rc);
            }
          }
#endif

          // Assign the requested PDO profile before activation.
          if (!apply_pdo_profile(i, slave_pos)) {
            slave_config_ok = false;
            break;
          }
          if (axis_profiles[i].brake.feedback_required)
            brake_signal_offsets[i] = tx_offset_for_semantic(off[i], axis_profiles[i].brake_released_semantic);
#if ROSIE_RT_SIMULATION
          sc[i]->sii_revision = sim_faults.sii_revision ? sim_faults.sii_revision : axis_profiles[i].revision_no;
          // Unverified simulated identity: seed CoE 0x1018:3 separately from
          // SII so mailbox readback cannot substitute an injected SII fault.
          auto& coe_revision = sc[i]->sdos[0x101803];
          coe_revision.resize(sizeof(uint32_t));
          EC_WRITE_U32(coe_revision.data(), axis_profiles[i].revision_no);
          // Model the value active at this simulated power-up. The mismatch
          // fixture proves startup refuses instead of writing and continuing.
          for (const auto& descriptor : startup_sdos) {
            if (!(descriptor.axis_mask & (1u << i)) || !descriptor.effective_after_power_cycle) continue;
            const auto value = descriptor.values[i] +
                (sim_faults.effective_startup_mismatch && i == 0 ? 1 : 0);
            const auto raw = startup_sdo_encode(descriptor.type, value);
            auto& stored = sc[i]->sdos[(uint32_t(descriptor.index) << 8) | descriptor.subindex];
            stored.assign(raw.begin(), raw.begin() + startup_sdo_size(descriptor.type));
          }
          sc[i]->raw_multi_turn_encoder_source = axis_profiles[i].independent_absolute_origin();
          if (i == 0) {
            sc[i]->position_counts = sim_faults.initial_position_counts;
            sc[i]->raw_multi_turn_frame_offset_counts = sim_faults.raw_multi_turn_origin_counts;
            sc[i]->raw_multi_turn_encoder_counts.store(
                int64_t(sim_faults.initial_position_counts) - sim_faults.raw_multi_turn_origin_counts);
            if (sim_faults.pdo_mismatch)
              sc[i]->sdos[(uint32_t(axis_profiles[i].pdo.tx_pdo) << 8) | 1][0] ^= 8;
          }
          sc[i]->brake = axis_profiles[i].brake;
          for (const auto& entry : axis_profiles[i].pdo.tx_entries)
            if (entry.semantic == axis_profiles[i].brake_released_semantic) {
              sc[i]->brake_signal_index = entry.index;
              sc[i]->brake_signal_subindex = entry.subindex;
              sc[i]->brake_signal_bit = axis_profiles[i].brake_released_bit;
            }
#endif
          log_phase_summary("after_slave_config_pdos", 0, 0);

          if (!schedule_axis_startup_sdos(i, slave_pos)) {
            slave_config_ok = false;
            break;
          }

          if (opt.use_dc) {
            // Separate the drive's synchronous update from the host's early-
            // cycle PDO exchange. Zero phase lets ordinary computation jitter
            // straddle the drive update and alternate duplicate/doubled feedback.
            // Half a configured period maximizes the margin to either boundary;
            // this changes scheduling only, never coordinate or motion limits.
            const auto sync0_shift_ns = static_cast<int32_t>(period / 2);
            const int rc = ecrt_slave_config_dc(sc[i], axis_profiles[i].dc_assign_activate,
                                               period, sync0_shift_ns, 0, 0);
            if (rc != 0) {
              logf("ERROR: ecrt_slave_config_dc failed for axis=%u slave_pos=%u rc=%d",
                   i,
                   slave_pos,
                   rc);
              slave_config_ok = false;
              break;
            }
            logf("EtherCAT config phase=slave_config_dc axis=%u slave_pos=%u assign_activate=0x%04x cycle_ns=%llu sync0_shift_ns=%d ok",
                 i,
                 slave_pos,
                 axis_profiles[i].dc_assign_activate,
                 static_cast<unsigned long long>(period), sync0_shift_ns);
            log_phase_summary("after_slave_config_dc", 0, 0);
          }
        }

        // The simulator's declared physical terminal must exist in the scan
        // before topology admission; PDO registration still follows admission.
#if ROSIE_RT_SIMULATION
        if (slave_config_ok && opt.cell_io.on_bus()) {
          const auto& io = opt.cell_io;
          auto* terminal = ecrt_master_slave_config(master, 0, io.position, io.vendor, io.product);
          if (terminal) terminal->sii_revision = io.revision;
          else slave_config_ok = false;
        }
#endif
        if (slave_config_ok && !topology.sample(master, opt, true)) {
          process_exit_code.store(1); g_stop.store(true); stop_cycle = true;
          return;
        }
        // IgH master.c:ec_master_request_op also raises its reference clock to
        // OP. Pin it to an axis so automatic selection cannot promote a held
        // device ahead of the configured subset, including with --no-dc.
        if (slave_config_ok && (opt.bus.hold || !opt.bus.unused.empty()) &&
            ecrt_master_select_reference_clock(master, sc[0]) != 0) {
          logf("ERROR: cannot select configured axis as DC reference; activation=blocked");
          process_exit_code.store(1); g_stop.store(true); stop_cycle = true;
          return;
        }
        // Fixed presets require physical proof before domain registration.
        // Explicit assignments belong to IgH at activation; the admitted runtime
        // readback must verify them before configuration readiness is granted.
        bool preop_ok = slave_config_ok;
        bool rescan_required = false;
        for (uint32_t i = 0; slave_config_ok && !opt.explicit_pdo_config && i < opt.num_axes; ++i) {
          MappingVerification progress;
          while (progress.pending()) {
            MappingDifference difference{};
            const auto result = wait_pdo_preop(master, opt.slave_position[i], opt.preop_to_safeop_timeout_ms) ?
                prepare_preop_mapping(master, i, opt.slave_position[i], axis_profiles[i].pdo,
                    original_assignment[i], difference, true) : MappingReadback::Unavailable;
            progress.observe(result);
            logf("EtherCAT mapping_readback axis=%u phase=PREOP result=%s attempts=%u index=0x%04x sub=0x%02x expected=0x%08x observed=0x%08x",
                 i, result == MappingReadback::Matched ? "verified" : result == MappingReadback::Mismatch ?
                 "pdo_mapping_mismatch" : "unavailable_retry", progress.attempts,
                 difference.index, difference.subindex, difference.expected, difference.observed);
          }
          if (!progress.verified) {
            preop_ok = false;
            logf("ERROR: mapping_readback axis=%u phase=PREOP result=refused attempts=%u %sactivation=blocked",
                 i, progress.attempts, rescanned ? "rescan_required=1 " : "");
          } else {
            MappingDifference difference{};
            const auto cached = verify_scanned_pdo(master, opt.slave_position[i], axis_profiles[i].pdo, difference);
            if (cached != MappingReadback::Matched) {
              rescan_required = true;
              logf("EtherCAT mapping_readback axis=%u phase=PREOP source=scanned_cache result=%s index=0x%04x sub=0x%02x expected=0x%08x observed=0x%08x rescan_required=1",
                   i, cached == MappingReadback::Mismatch ? "pdo_mapping_mismatch" : "unavailable",
                   difference.index, difference.subindex, difference.expected, difference.observed);
            }
          }
        }
        if (preop_ok && rescan_required) {
          const bool restore = std::any_of(axis_profiles.begin(), axis_profiles.begin() + opt.num_axes,
              [](const auto& profile) { return profile.pdo.restore_assignment_on_exit; });
          if (!rescanned && !restore && rescan_pdo_cache(master, opt, axis_profiles, now_monotonic_ns,
                                                       topology.snapshot.responding_expected)) {
            rescanned = true;
            domain = nullptr; axis_domain.fill(nullptr); sc.fill(nullptr);
            off.fill(AxisOffsets{}); brake_signal_offsets.fill(nullptr);
            continue;
          }
          logf("ERROR: mapping_readback phase=PREOP source=scanned_cache rescan_required=1 activation=blocked restoration=%s",
               restore ? "must_be_off" : "off");
          preop_ok = false;
        }
        if (!preop_ok) {
          process_exit_code.store(1);
          g_stop.store(true);
          slave_config_ok = false;
        }
        if (slave_config_ok) {
          // Register PDO entries -> domain offsets.
          // NOTE: ec_pdo_entry_reg_t layout differs across some IgH versions.
          // If compilation fails here once IgH is installed, adjust field count/order.
          bool domain_regs_ok = true;
          if (opt.explicit_pdo_config) {
            for (uint32_t i = 0; i < opt.num_axes; ++i) {
              ec_domain_t* target_domain = opt.split_domains_per_axis ? axis_domain[i] : domain;
              if (!register_axis_offsets_explicit(i, target_domain)) {
                domain_regs_ok = false;
                break;
              }
              logf("EtherCAT config phase=domain_reg_pdo_entry_pos axis=%u slave_pos=%u ok",
                   i,
                   static_cast<unsigned int>(opt.slave_position[i]));
            }
          } else {
            if (opt.split_domains_per_axis) {
              for (uint32_t i = 0; i < opt.num_axes; ++i) {
                std::array<ec_pdo_entry_reg_t, kMaxPdoRegsPerAxis + 1> regs{};
                size_t reg_i = 0;
                fill_axis_regs(i, regs, reg_i);
                regs[reg_i] = {};
                if (ecrt_domain_reg_pdo_entry_list(axis_domain[i], regs.data())) {
                  logf("ERROR: ecrt_domain_reg_pdo_entry_list failed for axis=%u slave_pos=%u",
                       i,
                       static_cast<unsigned int>(opt.slave_position[i]));
                  domain_regs_ok = false;
                  break;
                }
                logf("EtherCAT config phase=domain_reg_pdo_entry_list axis=%u slave_pos=%u regs=%zu ok",
                     i,
                     static_cast<unsigned int>(opt.slave_position[i]),
                     reg_i);
              }
            } else {
              std::array<ec_pdo_entry_reg_t,
                         (gradient::ipc::v1::GRADIENT_MAX_AXES * kMaxPdoRegsPerAxis) + 1>
                  regs{};
              size_t reg_i = 0;
              for (uint32_t i = 0; i < opt.num_axes; ++i) {
                fill_axis_regs(i, regs, reg_i);
              }
              regs[reg_i] = {};
              if (ecrt_domain_reg_pdo_entry_list(domain, regs.data())) {
                logf("ERROR: ecrt_domain_reg_pdo_entry_list failed");
                domain_regs_ok = false;
              } else {
                logf("EtherCAT config phase=domain_reg_pdo_entry_list ok regs=%zu", reg_i);
              }
            }
          }
          if (domain_regs_ok && !configure_cell_io(opt.split_domains_per_axis ? axis_domain[0] : domain)) {
            logf("ERROR: P6-05 cell I/O identity or fixed PDO mapping unavailable/mismatched");
            domain_regs_ok=false;
          }
          if (!domain_regs_ok) {
          } else {
            const uint64_t send_interval_us_u64 =
                opt.cycle_ns <= 1000ULL ? 1ULL : ((opt.cycle_ns + 999ULL) / 1000ULL);
            const size_t send_interval_us = static_cast<size_t>(send_interval_us_u64);
            log_registered_offsets();
            log_phase_summary("after_domain_reg_pdo_entry_list", 0, 0);
            const int send_interval_rc = ecrt_master_set_send_interval(master, send_interval_us);
            if (send_interval_rc != 0) {
              logf("ERROR: ecrt_master_set_send_interval(%zu us) failed rc=%d",
                   send_interval_us,
                   send_interval_rc);
            } else {
              logf("EtherCAT config phase=set_send_interval send_interval_us=%zu ok",
                   send_interval_us);
            }
            if (send_interval_rc != 0) {
              // Do not activate if the master rejected the advertised cycle interval.
            } else if (ecrt_master_activate(master)) {
              logf("ERROR: ecrt_master_activate failed");
            } else {
              master_activated = true;
              bool have_domain_pd = true;
              if (opt.split_domains_per_axis) {
                for (uint32_t i = 0; i < opt.num_axes; ++i) {
                  axis_domain_pd[i] = ecrt_domain_data(axis_domain[i]);
                  if (!axis_domain_pd[i]) {
                    logf("ERROR: ecrt_domain_data returned null for axis=%u slave_pos=%u",
                         i,
                         static_cast<unsigned int>(opt.slave_position[i]));
                    have_domain_pd = false;
                    break;
                  }
                }
              } else {
                domain_pd = ecrt_domain_data(domain);
                if (!domain_pd) {
                  logf("ERROR: ecrt_domain_data returned null");
                  have_domain_pd = false;
                }
              }
              if (!have_domain_pd) {
              } else {
                if (!startup_sdos.empty()) {
                  for (size_t descriptor_i = 0; descriptor_i < startup_sdos.size(); ++descriptor_i) {
                    auto& feedback = latest_feedback.startup_drive_config_feedback[descriptor_i];
                    for (uint32_t i = 0; i < opt.num_axes; ++i) {
                      feedback.readback_valid[i] = 0u;
                      feedback.readback[i] = 0u;
                      feedback.verified[i] = 0u;
                    }
                  }
                  logf("EtherCAT startup readback deferred for %zu descriptor(s); metrics thread will verify after startup_ready to avoid blocking rt-cycle during bring-up",
                       startup_sdos.size());
                }
                std::array<rosie::rtcore::DomainBinding,
                           gradient::ipc::v1::GRADIENT_MAX_AXES> bindings{};
                const size_t binding_count = opt.split_domains_per_axis ? opt.num_axes : 1;
                if (opt.split_domains_per_axis) {
                  for (uint32_t axis = 0; axis < opt.num_axes; ++axis)
                    bindings[axis] = {axis_domain[axis], 1u << axis, axis_domain_pd[axis]};
                } else {
                  bindings[0] = {domain, configured_mask, domain_pd};
                }
                ecrt_ok = cycle_backend.configure(master, configured_mask, bindings.data(), binding_count);
                if (!ecrt_ok) logf("ERROR: invalid cyclic bus domain geometry");
                startup_begin_ns = now_monotonic_ns();
                startup_cycle_counter = 0;
                logf("IgH libecrt active (num_axes=%u, expected_wkc=%u) backend=" ROSIE_BUS_WKC_BACKEND, opt.num_axes, expected_wkc);
                log_phase_summary("after_master_activate", 0, 0);
              }
            }
          }
        }
      }
      break;
      }
    }
  }
};

// C++17 prvalue construction guarantees the returned handles never move after
// their descriptors have registered pointers into the object's offset storage.
template<class Feedback>
BusHandles bring_up_bus(const Options& opt, const OptionsParseResult& parsed,
                        std::atomic<ec_master_t*>& shared_master, Feedback& feedback,
                        std::atomic<int>& process_exit_code) {
  return BusHandles(opt, parsed, shared_master, feedback, process_exit_code);
}

#endif  // GRADIENT_HAVE_ECRT
