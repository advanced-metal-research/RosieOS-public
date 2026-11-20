#pragma once

// Included in the daemon namespace. SII queries read IgH's scanned identity;
// no CoE uploads, slave configurations or process data belong to this inventory.
struct BusTopologyMonitor {
  rosie::rtcore::BusTopologySnapshot snapshot{};

  // Validate declarations on every observation under refuse, preventing a
  // replacement or disappearance from silently shrinking the admitted group.
  // Hold keeps unused-device changes diagnostic. A configured I/O terminal
  // always requires its declared identity and presence, preventing hold from
  // granting output permission to an unreadable or replaced terminal.
  bool sample(ec_master_t* master, const Options& opt, bool initial) {
    ec_master_info_t info{};
    if (ecrt_master(master, &info) != 0 || info.scan_busy) {
      if (initial) logf("ERROR: bus topology unavailable; activation=blocked");
      return false;
    }
    if (info.slave_count > opt.num_axes + rosie::rtcore::kMaxUnusedSlaves) {
      if (initial || snapshot.admitted) logf("ERROR: bus topology capacity exceeded; readiness=blocked");
      snapshot.admitted = false; ++snapshot.generation;
      snapshot.reason = rosie::rtcore::BusAdmissionReason::CapacityExceeded;
      return false;
    }
    auto next = snapshot;
    next.reason = rosie::rtcore::BusAdmissionReason::None;
    for (uint32_t i = 0; i < next.count; ++i) { next.slaves[i].present = false; next.slaves[i].al_state = 0; }
    std::vector<uint16_t> positions;
#ifdef ROSIE_RT_SIMULATED_BUS
    // Synthetic configurations can have sparse ring positions; enumerate their
    // actual physical fixtures without inventing intervening slaves.
    for (const auto& slave : master->slaves) positions.push_back(slave->position);
    for (const auto& slave : master->faults.extra_slaves) if (slave.present) positions.push_back(slave.position);
    std::sort(positions.begin(), positions.end());
#else
    for (uint32_t i = 0; i < info.slave_count; ++i) positions.push_back(static_cast<uint16_t>(i));
#endif
    bool admitted = true;
    using Reason = rosie::rtcore::BusAdmissionReason;
    using Disposition = rosie::rtcore::BusSlaveDisposition;
    auto refuse = [&](Reason reason, uint16_t position) {
      admitted = false;
      if (next.reason == Reason::None) { next.reason = reason; next.reason_position = position; }
    };
    uint32_t present = 0;
    const auto& io = opt.cell_io;
    const rosie::rtcore::BusSlaveIdentity io_identity{io.vendor, io.product, io.revision, io.position, true};
    for (uint16_t position : positions) {
      if (std::find(opt.slave_position.begin(), opt.slave_position.begin() + opt.num_axes, position) !=
          opt.slave_position.begin() + opt.num_axes) continue;
      ec_slave_info_t slave{};
      const bool readable = ecrt_master_get_slave(master, position, &slave) == 0;
      rosie::rtcore::BusSlaveIdentity identity{slave.vendor_id, slave.product_code, slave.revision_number, position, true};
      auto* entry = std::find_if(next.slaves.data(), next.slaves.data() + next.count,
          [&](const auto& s) { return s.identity.position == position; });
      const auto declared = std::find_if(opt.bus.unused.begin(), opt.bus.unused.end(),
          [&](const auto& s) { return s.position == position; });
      if (entry == next.slaves.data() + next.count) {
        if (next.count == rosie::rtcore::kMaxUnusedSlaves) {
          if (initial || snapshot.admitted) logf("ERROR: retained bus topology capacity exceeded; readiness=blocked");
          snapshot.admitted = false; ++snapshot.generation;
          snapshot.reason = Reason::CapacityExceeded;
          return false;
        }
        entry = &next.slaves[next.count++];
        entry->identity.position = position;
        entry->declared = declared != opt.bus.unused.end();
      }
      // A responding position is an observation even when SII is unavailable.
      // Publish it so refuse cannot retain admission from an older generation.
      entry->present = true;
      ++present;
      if (io.on_bus() && position == io.position) {
        entry->io_terminal = true;
        entry->expected = io_identity;
        entry->identity_readable = readable;
        entry->observed = identity;
        entry->al_state = slave.al_state;
        if (readable && !entry->identity_known) {
          entry->identity = identity;
          entry->identity_known = true;
        }
        if (!readable) {
          entry->disposition = Disposition::IdentityUnreadable;
          refuse(Reason::IdentityUnreadable, position);
        } else if (!io_identity.matches(identity)) {
          entry->disposition = Disposition::RefusedIdentity;
          refuse(Reason::DeclaredIdentityMismatch, position);
        } else if (!initial && slave.al_state != 8) {
          entry->disposition = Disposition::RefusedState;
          refuse(Reason::IoTerminalNotOperational, position);
        } else {
          entry->disposition = Disposition::IoTerminal;
        }
        if (initial || entry->disposition != snapshot.slaves[entry - next.slaves.data()].disposition)
          logf("bus io_terminal position=%u vendor_id=0x%08x product_code=0x%08x revision=0x%08x disposition=%s admission=%s",
               unsigned(position), identity.vendor_id, identity.product_code, identity.revision,
               rosie::rtcore::bus_slave_disposition(entry->disposition),
               entry->disposition == Disposition::IoTerminal ? "admitted" : "blocked");
        continue;
      }
      if (!readable) {
        if (entry->identity_readable || snapshot.generation == 0 ||
            entry >= next.slaves.data() + snapshot.count)
          logf("%sbus slave position=%u identity=unknown disposition=identity_unreadable state_policy=PREOP configuration=none PDOs=0 startup_SDOs=0 outputs=0 admission=%s",
               opt.bus.hold ? "WARNING: " : "ERROR: ", unsigned(position), opt.bus.hold ? "held" : "blocked");
        entry->identity_readable = false;
        entry->observed = {}; entry->observed.position = position;
        entry->disposition = rosie::rtcore::BusSlaveDisposition::IdentityUnreadable;
        if (!opt.bus.hold) refuse(Reason::IdentityUnreadable, position);
        continue;
      }
      entry->identity_readable = true;
      if (!entry->identity_known) {
        entry->identity = identity;
        entry->identity_known = true;
        const bool unknown = !entry->declared && !opt.bus.hold;
        const bool mismatch = entry->declared && !declared->matches(identity) && (initial || !opt.bus.hold);
        logf("%sbus slave position=%u vendor_id=0x%08x product_code=0x%08x revision=0x%08x disposition=%s state_policy=PREOP configuration=none PDOs=0 startup_SDOs=0 outputs=0",
             unknown || mismatch ? "ERROR: " : "", unsigned(position), identity.vendor_id, identity.product_code, identity.revision,
             unknown ? "refused_unknown" : mismatch ? "refused_identity" : entry->declared ? "declared_unused" : "held");
        if (mismatch) logf("ERROR: bus declaration position=%u expected_vendor_id=0x%08x expected_product_code=0x%08x expected_revision=0x%08x revision_required=%u activation=blocked",
            unsigned(position), declared->vendor_id, declared->product_code, declared->revision, declared->revision_required ? 1u : 0u);
      }
      const bool unknown = !entry->declared && !opt.bus.hold;
      const bool mismatch = entry->declared && !declared->matches(identity);
      entry->disposition = unknown ? Disposition::RefusedUnknown :
          mismatch ? (initial || !opt.bus.hold ? Disposition::RefusedIdentity : Disposition::IdentityChanged) :
          entry->declared ? Disposition::DeclaredUnused :
          !entry->identity.matches(identity) ? Disposition::IdentityChanged : Disposition::Held;
      if (unknown) refuse(Reason::UnknownSlave, position);
      if ((initial || !opt.bus.hold) && mismatch) {
        refuse(Reason::DeclaredIdentityMismatch, position);
        if (initial || entry->disposition != snapshot.slaves[entry - next.slaves.data()].disposition)
          logf("ERROR: bus position=%u vendor_id=0x%08x product_code=0x%08x revision=0x%08x expected_vendor_id=0x%08x expected_product_code=0x%08x expected_revision=0x%08x disposition=refused_identity reason=declared_identity_mismatch readiness=blocked",
               unsigned(position), identity.vendor_id, identity.product_code, identity.revision,
               declared->vendor_id, declared->product_code, declared->revision);
      }
      entry->observed = identity; entry->present = true; entry->al_state = slave.al_state;
      // IgH must leave unconfigured devices below SAFEOP. A device observed in
      // SAFEOP/OP is not held: refuse until its state is safe, even after startup.
      if (slave.al_state != EC_AL_STATE_PREOP && slave.al_state != EC_AL_STATE_INIT) {
        refuse(Reason::UnsafeState, position);
        entry->disposition = Disposition::RefusedState;
        if (initial || entry->al_state != snapshot.slaves[entry - next.slaves.data()].al_state)
          logf("ERROR: bus held position=%u al_state=0x%x policy=refuse_until_PREOP_or_INIT", unsigned(position), unsigned(slave.al_state));
      }
    }
    uint32_t missing_required = 0;
    if (io.on_bus()) {
      auto* entry = std::find_if(next.slaves.data(), next.slaves.data() + next.count,
          [&](const auto& s) { return s.identity.position == io.position; });
      if (entry == next.slaves.data() + next.count) {
        if (next.count == rosie::rtcore::kMaxUnusedSlaves) {
          refuse(Reason::CapacityExceeded, io.position);
        } else {
          entry = &next.slaves[next.count++];
          entry->identity.position = io.position;
          entry->io_terminal = true;
          entry->expected = io_identity;
        }
      }
      if (entry == next.slaves.data() + next.count || !entry->present) {
        if (initial || snapshot.admitted)
          logf("ERROR: bus io_terminal missing position=%u disposition=refused_missing reason=declared_slave_missing readiness=blocked",
               unsigned(io.position));
        refuse(Reason::DeclaredSlaveMissing, io.position);
        ++missing_required;
      }
    }
    if (initial || !opt.bus.hold) {
      for (const auto& declaration : opt.bus.unused) {
        if (std::none_of(next.slaves.begin(), next.slaves.begin() + next.count,
              [&](const auto& s) { return s.present && s.identity.position == declaration.position; })) {
          if (initial || snapshot.admitted)
            logf("ERROR: bus declared_unused missing position=%u vendor_id=0x%08x product_code=0x%08x revision=0x%08x disposition=refused_missing reason=declared_slave_missing readiness=blocked",
                 unsigned(declaration.position), declaration.vendor_id, declaration.product_code, declaration.revision);
          refuse(Reason::DeclaredSlaveMissing, declaration.position);
          ++missing_required;
        }
      }
    }
    for (uint32_t i = 0; i < next.count; ++i) {
      auto& entry = next.slaves[i];
      if (!entry.present) {
        entry.identity_readable = false;
        entry.disposition = entry.io_terminal || (entry.declared && (initial || !opt.bus.hold)) ? Disposition::RefusedMissing : Disposition::Disappeared;
      }
    }
    if (!initial) {
      for (uint32_t i = 0; i < snapshot.count; ++i) {
        const auto& before = snapshot.slaves[i]; const auto& after = next.slaves[i];
        if (after.io_terminal) continue;
        if (before.present != after.present || before.identity_readable != after.identity_readable ||
            (after.identity_readable && !before.observed.matches(after.observed))) {
          logf("%sbus unused position=%u present=%u vendor_id=0x%08x product_code=0x%08x revision=0x%08x policy=%s reason=%s outputs=0",
               opt.bus.hold ? "WARNING: " : "ERROR: ",
               unsigned(after.identity.position), after.present ? 1u : 0u, after.observed.vendor_id, after.observed.product_code, after.observed.revision,
               opt.bus.hold ? "log_and_continue_axes" : "refuse",
               !after.present ? "disappeared" : !after.identity_readable ? "identity_unreadable" : !before.present ? "appeared" : "identity_changed");
        }
      }
    }
    next.responding_expected = opt.num_axes + present + missing_required;
    next.admitted = admitted;
    ++next.generation;
    snapshot = next;
    return admitted;
  }
};
