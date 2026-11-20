// Replay the measured scheduling delay against fixed budgets. This prevents
// fixture tuning from disabling deadline enforcement or absorbing mailbox work.
void test_simulation_cycle_budget() {
  struct Case { uint64_t budget, lateness; bool fault; };
  // Measured program-travel run at 3d538662: 115,525,612 ns wake lateness,
  // 581,617 ns cycle work. The 200,000,000 ns cap is unverified beyond this
  // host sample; one ns beyond it must still fault and inhibit.
  for (const auto sample : {Case{100'000'000, 115'525'612, true},
                            Case{200'000'000, 115'525'612, false},
                            Case{200'000'000, 200'000'001, true}}) {
    Fixture f;
    f.options.max_cycle_lateness_ns = sample.budget;
    f.machine.armed.store(false);
    f.machine.axis_enable_mask.store(0);
    // Unverified reservation fixture: job 42, generation/configuration/Home 1.
    f.machine.commissioning_state = {{42, 1, 1, 1}, rosie::rtcore::CommissioningPhase::Active, 1, 1, 0};
    f.machine.service_mode_axis_mask.store(1);
    f.machine.next_ns = test_now - sample.lateness;
    f.machine.previous_cycle_ns = f.machine.next_ns - f.options.cycle_ns;
    const auto output = f.step();
    assert(f.machine.execution_fault_reasons.load() == (sample.fault ? 4u : 0u));
    assert(f.machine.control_policy.generation.load() == (sample.fault ? 2u : 1u));
    assert(f.machine.service_mode_axis_mask.load() == (sample.fault ? 0u : 1u));
    assert(output.motion_permitted_mask == 0 && f.bus.cycle_backend.axis(0).read_u16(0) == 0);
  }
  std::puts("PASS: measured simulator lateness and fixed deadline boundary");
}

// Exercise the real Home worker and cyclic permit with an injected clock miss.
// A set_mode cancellation must retain its cause and never re-enable the group.
void test_home_set_mode_cycle_deadline() {
  test_simulation_cycle_budget();
  Fixture f;
  auto& m = f.machine;
  m.armed.store(false);
  m.axis_enable_mask.store(0);
  m.commissioning_running.store(true);
  m.control_policy.application_required.store(false);
  m.latest_feedback.seq.store(2);
  auto& home = f.configuration.axis_profiles[0].home;
  home.valid = home.truth_source.valid = true;
  home.commissioning_mode = 6;
  home.steady_state_mode = 8;
  NativeHomeOp mode{};
  mode.kind = NativeHomeOpKind::kSetMode;
  mode.value_i32 = 6;
  home.transaction.push_back(mode);
  // Unverified job fixture: job 42, native/configuration/Home generation 1.
  const rosie::rtcore::CommissioningIdentity identity{42, 1, 1, 1};
  m.commissioning_state = {identity, rosie::rtcore::CommissioningPhase::Active, 1, 0, 0};
  m.control_policy.heartbeat_ns.store(now_monotonic_ns());
  unsigned services = 0;
  uint64_t ticket = 0;
  auto request = [&](rosie::rtcore::CommissioningRequest request,
                     rosie::rtcore::CommissioningIdentity* = nullptr,
                     bool* = nullptr,
                     rosie::rtcore::CommissioningReason* reason = nullptr) {
    request.ticket = ++ticket;
    test_now = now_monotonic_ns();
    m.next_ns = test_now;
    m.control_policy.heartbeat_ns.store(test_now);
    if (request.operation == rosie::rtcore::CommissioningOperation::Service && ++services == 2) {
      // Unverified injection: 20,000,001 ns late against the fixed 20,000,000 ns budget.
      m.next_ns = test_now - 20'000'001;
    }
    m.previous_cycle_ns = m.next_ns - 1'000'000;
    assert(m.commissioning_requests.push({request}));
    const auto output = m.step(CycleInputs<SyntheticHandles>{f.bus, nullptr, synthetic_clock});
    if (services >= 2) {
      assert(output.motion_permitted_mask == 0);
      assert(f.bus.cycle_backend.axis(0).read_u16(0) == 0);
    }
    MachineState::CommissioningReceipt receipt{};
    assert(m.commissioning_receipts.pop(receipt));
    assert(receipt.ticket == ticket);
    if (reason) *reason = receipt.reason;
    return receipt.accepted;
  };
  assert(!m.native_home_axis(0, identity, request));
  assert(services == 2);
  assert(m.execution_fault_reasons.load() == 4);
  assert(m.safety_fault_mask.load() == 1);
  assert(m.control_policy.generation.load() == 2);
  assert(m.control_policy.home_valid_mask.load() == 0);
  assert(m.service_mode_axis_mask.load() == 0);
  assert(!m.armed.load() && m.axis_enable_mask.load() == 0);
  assert(m.latest_feedback.native_home_state[0] == gradient::ipc::v1::NATIVE_HOME_STATE_FAILED);
  assert(m.latest_feedback.native_home_last_abort_code[0] == gradient::ipc::v1::NATIVE_HOME_ABORT_SERVICE_REFUSED);
  MachineState::NativeHomeFailure failure{};
  assert(m.native_home_failures[0].read(failure));
  assert(rosie::rtcore::same_commissioning_identity(failure.identity, identity));
  assert(failure.axis == 0 && failure.step == 0);
  assert(failure.cancellation_reason == rosie::rtcore::CommissioningReason::CycleDeadline);
  assert(std::string(failure.operation) == "set_mode" && std::string(failure.reason) == "cycle_deadline");
  std::ostringstream json;
  append_native_home_failure_json(json, failure);
  assert(json.str().find("\"cancellation_reason\":\"cycle_deadline\"") != std::string::npos);
  assert(json.str().find("\"operation\":\"set_mode\"") != std::string::npos);
  assert(request({0, identity, rosie::rtcore::CommissioningOperation::Finish}));
  assert(m.commissioning_state.phase == rosie::rtcore::CommissioningPhase::Idle);
  assert(m.native_home_failures[0].read(failure) && failure.step == 0);
  // Unverified successor fixture: job 43 and native generation 2. The earlier
  // deadline identity cannot relabel this independently refused reservation.
  const rosie::rtcore::CommissioningIdentity successor{43, 2, 1, 1};
  assert(!m.native_home_axis(0, successor, request));
  assert(m.native_home_failures[0].read(failure));
  assert(failure.identity.job == 43);
  assert(failure.cancellation_reason != rosie::rtcore::CommissioningReason::CycleDeadline);
  std::puts("PASS: injected Home set_mode cycle deadline cancels and inhibits");
}
