#define ROSIE_RT_SIMULATION 1
// Unverified fixture identity: parse_options reaches --version, and test
// binaries never link the daemon's source identity.
#define ROSIE_RT_VERSION "configuration-identity-test"
#include "engine/cycle_machine.hpp"
#include <cassert>

// Unverified one-axis fixture, pinned independently in tools/rtctl/identity_test.go.
constexpr const char* machine_sha = "9cadc16d850fc08c87abcc144b196894c5624f3af2d041e17df0bdb9fe314cd2";
// Unverified deployment fixture: simulation, CPU 0 and FIFO priority 1.
constexpr const char* deployment_sha = "e85113761b0ab11cc3c98d5006228826748b44db859d30a391c89e01a6b0198c";
// Unverified whole-artifact fixture; the daemon preserves this legacy binding.
constexpr const char* configuration_sha = "308f2229c485a3d6c27e28bc84eeb8ec7a1723bf3b0728c801836fbd781bfd7d";

int main() {
  namespace id = rosie::rtcore::identity;
  assert(id::digest("") == "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
  assert(id::digest("abc") == "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
  assert(id::digest(std::string(1000000,'a')) == "cdc76e5c9914fb9281a1c7e284d73e67f1809a48a497200e046d39ccc7112cd0");
  assert(id::number(-0.0) == "-0");
  assert(id::number(0.00001) == "1.0000000000000001e-05");
  assert(id::canonical({{"b","2"},{"a","1"}}) == "a=1\nb=2\n");
  const std::string deployment = "\"backend\"=\"simulation\"\n\"runtime\"={}\n\"runtime\".\"cpu\"=0\n\"runtime\".\"priority\"=1\n";
  assert(id::digest(deployment) == deployment_sha);
  char path[] = "build/configuration-identity-XXXXXX";
  const int fd = mkstemp(path);
  assert(fd >= 0);
  close(fd);
  std::ofstream profile(path);
  profile << R"(axis=0
name=axis_0
vendor_id=1
product_code=1
revision_no=0
rx_pdo=5632
tx_pdo=6656
rx_sync=2
tx_sync=3
dc_quantum_ns=250000
dc_assign_activate=768
rx_layout=cw|24640|0|16;target_pos|24698|0|32;mode|24672|0|8
tx_layout=sw|24641|0|16;pos|24676|0|32;mode_disp|24673|0|8
home_truth_sign=1
native_home=
absolute_feedback=
absolute_pair_field=
startup_sdos=
limits=-1.0|1.0|1.0|0
tracking=1.0|0.1|0.2|0.01|100|500
jog=200000000|300000000
brake=0|100|100|0|-|0
)";
  // Unverified synthetic count contract; all thirteen declarations are explicit.
  profile << "coordinate_identity=" << id::hex_text(
      "anchor.source=none|0\ndirection.sign=1\ndrive.product_code=1\n"
      "drive.revision_no=0\ndrive.vendor_id=1\nencoder.counts_per_motor_rev=1000\n"
      "evidence.mode=multi_turn\ngear.denominator=1\ngear.numerator=1\nhome.policy=none\n"
      "tracking.mode=continuous\nwire.counts_per_rev=1000\nwire.revs_per_axis_rev=1\n") << '\n';
  profile.close();
  std::vector<std::string> args{"identity-test","--axis-profiles",path,"--configuration-hash",configuration_sha,
      "--machine-sha256",machine_sha,"--deployment-sha256",deployment_sha,
      "--rt-cpu","0","--rt-priority","1","--num-axes","1","--cycle-ns","1000000",
      "--max-cycle-lateness-ns","20000000","--counts-per-rev","1000","--gear-ratio","1",
      "--max-rpm","100"};
  std::vector<char*> argv;
  for (auto& arg : args) argv.push_back(arg.data());
  Options options;
  auto parsed = parse_options(int(argv.size()),argv.data(),&options);
  assert(parsed.exit_code < 0);
  auto missing = parsed.axis_profiles[0];
  missing.coordinate_identity.clear();
  bool refused = false;
  try { id::coordinate_fields(0, options.axis[0], missing, parsed.startup_sdos); }
  catch (const std::invalid_argument& e) {
    refused = std::string(e.what()) == "missing coordinate_identity axis=0 name=axis_0";
  }
  assert(refused);
  assert(options.machine_sha256 == machine_sha);
  assert(options.machine_digest == id::sha256(id::canonical(id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos))));
  assert(options.deployment_sha256 == deployment_sha);
  assert(options.configuration_hash == configuration_sha);
  // Unverified synthetic count frame: 1000 counts/rev, internal gearing 1/1.
  const id::Fields coordinate_fields{{"anchor.source","none|0"}, {"direction.sign","1"},
      {"drive.product_code","1"}, {"drive.revision_no","0"}, {"drive.vendor_id","1"},
      {"encoder.counts_per_motor_rev","1000"}, {"evidence.mode","multi_turn"},
      {"gear.denominator","1"}, {"gear.numerator","1"}, {"home.policy","none"},
      {"tracking.mode","continuous"}, {"wire.counts_per_rev","1000"}, {"wire.revs_per_axis_rev","1"}};
  auto coordinate_profile = parsed.axis_profiles[0];
  coordinate_profile.coordinate_identity = id::canonical(coordinate_fields);
  assert(id::coordinate_fields(0, options.axis[0], coordinate_profile, parsed.startup_sdos) == coordinate_profile.coordinate_identity);
  // Unverified fixture digests, independently pinned with the same Go fields.
  assert(id::digest(coordinate_profile.coordinate_identity) == "4856cdd9c8f30c6b63944406bd2b1b515e98fe9933a333ac811366f7d6c7a674");
  // The source is reconstructed from the admitted profile, never trusted from
  // a stale compiled table. Changing its version must also change the digest.
  coordinate_profile.anchor_source = {1, 1, 2, "synthetic_raw", "raw_multi_turn_encoder", 4};
  const auto next_source = id::coordinate_fields(0, options.axis[0], coordinate_profile, parsed.startup_sdos);
  assert(id::parse_fields(next_source).at("anchor.source") == "raw_multi_turn_encoder|4");
  assert(id::digest(next_source) != id::digest(coordinate_profile.coordinate_identity));
  assert(id::first_difference(coordinate_profile.coordinate_identity, next_source) == "anchor.source");
  coordinate_profile.anchor_source = {};
  for (const auto& entry : coordinate_fields) {
    auto incomplete = coordinate_fields;
    incomplete.erase(entry.first);
    coordinate_profile.coordinate_identity = id::canonical(incomplete);
    bool refused = false;
    try { (void)id::coordinate_fields(0, options.axis[0], coordinate_profile, parsed.startup_sdos); }
    catch (const std::invalid_argument&) { refused = true; }
    assert(refused);
  }
  for (const auto& key : {"pair.id", "pair.revision"}) {
    auto unexpected = coordinate_fields;
    unexpected[key] = "1";
    coordinate_profile.coordinate_identity = id::canonical(unexpected);
    bool refused = false;
    try { (void)id::coordinate_fields(0, options.axis[0], coordinate_profile, parsed.startup_sdos); }
    catch (const std::invalid_argument&) { refused = true; }
    assert(refused);
  }
  // Current-pose capture was removed: a coordinate identity still carrying its
  // reference.capture field is refused, and the canonical bytes are unchanged.
  {
    auto profile = parsed.axis_profiles[0];
    auto fields = coordinate_fields;
    fields["reference.capture"] = "independent_absolute_current_pose_v1,current,same_pose_only";
    profile.coordinate_identity = id::canonical(fields);
    bool refused = false;
    try { (void)id::coordinate_fields(0, options.axis[0], profile, parsed.startup_sdos); }
    catch (const std::invalid_argument&) { refused = true; }
    assert(refused);
    profile.coordinate_identity = id::canonical(coordinate_fields);
    assert(id::digest(id::coordinate_fields(0, options.axis[0], profile, parsed.startup_sdos)) ==
           "4856cdd9c8f30c6b63944406bd2b1b515e98fe9933a333ac811366f7d6c7a674");
  }
  // Native parsing and identity must carry measured brake tolerance. Legacy
  // six-field profiles retain one count; malformed seventh fields fail closed.
  assert(parsed.axis_profiles[0].brake.hold_displacement_tolerance_counts == 1);
  auto measured_profiles = parsed.axis_profiles;
  measured_profiles[0].brake.hold_displacement_tolerance_counts = 16;
  assert(id::machine_fields(options, measured_profiles, parsed.startup_sdos).at("axis.0.brake") == "0|100|100|0|-|0|16");
  assert(id::digest(id::canonical(id::machine_fields(options, measured_profiles, parsed.startup_sdos))) != machine_sha);
  std::ifstream original_profile(path);
  const std::string legacy_profile((std::istreambuf_iterator<char>(original_profile)), {});
  original_profile.close();
  // Optional restoration must coexist with mandatory coordinate identity;
  // neither restoration nor an explicit empty table may bypass axis admission.
  for (const auto* restore : {"", "0", "1", "2", "true", "0\nrestore_assignment_on_exit=1", "1\nrestore_assignment_on_exit=0",
                            "0\nrestore_assignment_on_exit=0", "1\nrestore_assignment_on_exit=1"}) {
    for (const auto* coordinate : {"complete", "missing", "empty", "duplicate"}) {
      // The same parser and CLI refusals must name the axis with name first or
      // last; declaration order cannot hide the source of rejected input.
      for (const bool name_last : {false, true}) {
        for (const bool explicit_pdo : {false, true}) {
          std::string encoded = legacy_profile;
          const auto begin = encoded.find("coordinate_identity=");
          assert(begin != std::string::npos);
          const auto end = encoded.find('\n', begin);
          if (std::string(coordinate) == "missing") encoded.erase(begin, end - begin + 1);
          if (std::string(coordinate) == "empty")
            encoded.erase(begin + std::strlen("coordinate_identity="), end - begin - std::strlen("coordinate_identity="));
          if (std::string(coordinate) == "duplicate") encoded += encoded.substr(begin, end - begin + 1);
          if (*restore) encoded += std::string("restore_assignment_on_exit=") + restore + '\n';
          if (name_last) {
            encoded.erase(encoded.find("name=axis_0\n"), std::strlen("name=axis_0\n"));
            encoded += "name=axis_0\n";
          }
          std::ofstream replacement(path); replacement << encoded; replacement.close();
          Options native_options;
          native_options.num_axes = 1;
          native_options.explicit_pdo_config = explicit_pdo;
          std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
          std::vector<StartupSdoConfig> native_sdos;
          std::string diagnostic;
          const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos, &diagnostic);
          const bool valid = std::string(coordinate) == "complete" &&
              (!*restore || (!explicit_pdo && (std::string(restore) == "0" || std::string(restore) == "1")));
          assert(accepted == valid);
          std::string expected_diagnostic;
          if (std::string(coordinate) == "duplicate")
            expected_diagnostic = "axis=0 name=axis_0 key=coordinate_identity reason=duplicate";
          else if (*restore && (explicit_pdo || (std::string(restore) != "0" && std::string(restore) != "1"))) {
            const bool repeated_same = !explicit_pdo && (std::string(restore) == "0\nrestore_assignment_on_exit=0" ||
                std::string(restore) == "1\nrestore_assignment_on_exit=1");
            expected_diagnostic = "axis=0 name=axis_0 key=restore_assignment_on_exit reason=" +
                std::string(repeated_same ? "duplicate" : "conflict");
          }
          assert(diagnostic == expected_diagnostic);
          if (!expected_diagnostic.empty()) {
            // Exercise the actual CLI error path as well as the parser result;
            // callers must see the axis/key instead of only a generic refusal.
            char capture_path[] = "build/identity-refusal-XXXXXX";
            const int capture_fd = mkstemp(capture_path);
            assert(capture_fd >= 0);
            FILE* captured = fdopen(capture_fd, "w+");
            assert(captured);
            const int saved_stderr = dup(STDERR_FILENO);
            assert(saved_stderr >= 0 && dup2(fileno(captured), STDERR_FILENO) >= 0);
            auto cli_args = args;
            if (explicit_pdo) cli_args.push_back("--explicit-pdo-config");
            std::vector<char*> cli_argv;
            for (auto& arg : cli_args) cli_argv.push_back(arg.data());
            Options cli_options;
            const auto rejected = parse_options(int(cli_argv.size()), cli_argv.data(), &cli_options);
            std::fflush(stderr);
            assert(dup2(saved_stderr, STDERR_FILENO) >= 0);
            close(saved_stderr);
            std::rewind(captured);
            std::string log;
            // Unverified log read chunk: 512 bytes; all chunks are retained.
            char buffer[512];
            while (std::fgets(buffer, sizeof(buffer), captured)) log += buffer;
            std::fclose(captured);
            std::remove(capture_path);
            assert(rejected.exit_code == 2);
            assert(log.find("ERROR: invalid --axis-profiles file " + expected_diagnostic) != std::string::npos);
          }
          if (accepted) {
            assert(native_profiles[0].coordinate_identity == parsed.axis_profiles[0].coordinate_identity);
            assert(native_profiles[0].pdo.restore_assignment_on_exit == (std::string(restore) == "1"));
          }
        }
      }
    }
  }
  std::puts("PASS TestNamedRefusalsIndependentOfNameOrder");
  for (const auto& tolerance : {"0", "1", "16", "2147483647", "-1", "1.5", "2147483648", "", "16|1"}) {
    std::string encoded = legacy_profile;
    const auto brake = encoded.find("brake=0|100|100|0|-|0");
    encoded.insert(brake + std::strlen("brake=0|100|100|0|-|0"), std::string("|") + tolerance);
    std::ofstream replacement(path); replacement << encoded; replacement.close();
    Options native_options;
    native_options.num_axes = 1;
    std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
    std::vector<StartupSdoConfig> native_sdos;
    const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
    const bool valid = std::string(tolerance) == "0" || std::string(tolerance) == "1" ||
        std::string(tolerance) == "16" || std::string(tolerance) == "2147483647";
    assert(accepted == valid);
    if (accepted) assert(native_profiles[0].brake.hold_displacement_tolerance_counts == std::stoul(tolerance));
  }
  // The optional transition policy must survive native parsing and identity;
  // existing profiles retain their exact published digest when it is absent.
  assert(!parsed.axis_profiles[0].brake.transition.enabled());
  for (const std::string transition : {"327|6553|100|100|500", "0|6553|100|100|500",
      "15|6553|100|100|500", "327|0|100|100|500", "327|6553|100|100|199",
      "327|6553|100|100|500|1", "327|6553|100|100", "-1|6553|100|100|500"}) {
    for (bool velocity : {false,true}) {
      std::string encoded = legacy_profile;
      const auto brake = encoded.find("brake=0|100|100|0|-|0");
      encoded.replace(brake, std::strlen("brake=0|100|100|0|-|0"), "brake=1|100|100|0|-|0|16");
      if (velocity) {
        const auto layout = encoded.find("tx_layout=");
        encoded.insert(encoded.find('\n',layout), ";velocity_actual|24684|0|32");
      }
      encoded += "brake_transition=" + transition + "\n";
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 1;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
      assert(accepted == (velocity && transition == "327|6553|100|100|500"));
      if (accepted) {
        assert(native_profiles[0].brake.hold_displacement_tolerance_counts == 16);
        auto fields = id::machine_fields(options, native_profiles, native_sdos);
        assert(fields.at("axis.0.brake_transition") == transition);
        assert(id::digest(id::canonical(fields)) != machine_sha);
      }
    }
  }
  // Opening requires an explicit closing policy and identity-pins every bound.
  for (const std::string opening : {"327|13107|500", "15|13107|500", "327|0|500",
      "327|13107|0", "327|13107", "327|13107|500|1", "-1|13107|500"}) {
    for (bool closing : {false, true}) {
      std::string encoded = legacy_profile;
      const auto brake = encoded.find("brake=0|100|100|0|-|0");
      encoded.replace(brake, std::strlen("brake=0|100|100|0|-|0"), "brake=1|100|100|0|-|0|16");
      const auto layout = encoded.find("tx_layout=");
      encoded.insert(encoded.find('\n',layout), ";velocity_actual|24684|0|32");
      if (closing) encoded += "brake_transition=327|6553|100|100|500\n";
      encoded += "brake_opening=" + opening + "\n";
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 1;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
      assert(accepted == (closing && opening == "327|13107|500"));
      if (accepted) {
        auto fields = id::machine_fields(options, native_profiles, native_sdos);
        assert(fields.at("axis.0.brake_opening") == opening);
        assert(native_profiles[0].brake.hold_displacement_tolerance_counts == 16);
      }
    }
  }
  // drive_supervision is optional (absent: disabled, no line, digest unchanged);
  // when present every bound is positive and identity-pinned.
  assert(!parsed.axis_profiles[0].drive_supervision.configured());
  assert(!id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos).count("axis.0.drive_supervision"));
  for (const std::string supervision : {"500|500|7509", "0|500|7509", "500|0|7509", "500|500|0",
      "500|500", "500|500|7509|16", "500|500|2147483648", "-1|500|7509"}) {
    std::string encoded = legacy_profile + "drive_supervision=" + supervision + "\n";
    std::ofstream replacement(path); replacement << encoded; replacement.close();
    Options native_options; native_options.num_axes = 1;
    std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
    std::vector<StartupSdoConfig> native_sdos;
    const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
    assert(accepted == (supervision == "500|500|7509"));
    if (accepted) {
      const auto& d = native_profiles[0].drive_supervision;
      assert(d.enable_timeout_ns == 500000000 && d.release_slack_ns == 500000000 &&
             d.disarm_motion_counts == 7509);
      auto fields = id::machine_fields(options, native_profiles, native_sdos);
      assert(fields.at("axis.0.drive_supervision") == supervision);
      assert(id::digest(id::canonical(fields)) != machine_sha);
    }
  }
  // tracking_lag_allowance is optional (absent: static tracking bounds, no
  // line, digest unchanged); when present it is 1..1000 ms, identity-pinned,
  // and one machine-level value on every axis.
  assert(parsed.axis_profiles[0].limits.tracking_lag_allowance_ns == 0);
  assert(!id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos).count("axis.0.tracking_lag_allowance"));
  for (const std::string lag : {"50", "1", "1000", "0", "1001", "-50", "+50", "50.0", "0.05", "", "50|50", "nan"}) {
    std::string encoded = legacy_profile + "tracking_lag_allowance=" + lag + "\n";
    std::ofstream replacement(path); replacement << encoded; replacement.close();
    Options native_options; native_options.num_axes = 1;
    std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
    std::vector<StartupSdoConfig> native_sdos;
    const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
    assert(accepted == (lag == "50" || lag == "1" || lag == "1000"));
    if (accepted) {
      assert(native_profiles[0].limits.tracking_lag_allowance_ns == std::stoull(lag) * 1000000);
      auto fields = id::machine_fields(options, native_profiles, native_sdos);
      assert(fields.at("axis.0.tracking_lag_allowance") == lag);
      assert(id::digest(id::canonical(fields)) != machine_sha);
    }
  }
  {
    std::string second = legacy_profile;
    second.replace(second.find("axis=0"), 6, "axis=1");
    second.insert(second.find("\nname=") + 6, "b");
    for (const auto* pair : {"50,50", "50,", ",50", "50,40"}) {
      const std::string first_lag = std::string(pair).substr(0, std::string(pair).find(','));
      const std::string second_lag = std::string(pair).substr(std::string(pair).find(',') + 1);
      std::string encoded = legacy_profile + (first_lag.empty() ? "" : "tracking_lag_allowance=" + first_lag + "\n") +
          second + (second_lag.empty() ? "" : "tracking_lag_allowance=" + second_lag + "\n");
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 2;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      assert(load_axis_profiles(path, native_options, &native_profiles, &native_sdos) == (std::string(pair) == "50,50"));
    }
  }
  // jog_jerk=<jerk>|<window cycles> is optional (absent: unsmoothed jog, no
  // line, digest unchanged); when present both are decimal integers, jerk
  // 1..100000, window 2..1024, identity-pinned, one value on every axis, and the
  // window must bound the axis's jerk: 2 x 1 rad/s^2 / (window x 1 ms) <= jerk,
  // so jerk 20 rad/s^3 needs at least 100 cycles.
  assert(parsed.axis_profiles[0].jog.smoothing_cycles == 0);
  assert(!id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos).count("axis.0.jog_jerk"));
  for (const std::string jerk : {"20|100", "20|1024", "100000|100", "20|99", "20|1025", "20|1", "0|100", "100001|100",
                                 "20.0|100", "+20|100", "20|", "|100", "20", "20|100|1", "-20|100"}) {
    std::string encoded = legacy_profile + "jog_jerk=" + jerk + "\n";
    std::ofstream replacement(path); replacement << encoded; replacement.close();
    Options native_options; native_options.num_axes = 1;
    std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
    std::vector<StartupSdoConfig> native_sdos;
    const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
    assert(accepted == (jerk == "20|100" || jerk == "20|1024" || jerk == "100000|100"));
    if (accepted) {
      auto fields = id::machine_fields(options, native_profiles, native_sdos);
      assert(fields.at("axis.0.jog_jerk") == jerk);
      assert(id::digest(id::canonical(fields)) != machine_sha);
    }
  }
  {
    std::string second = legacy_profile;
    second.replace(second.find("axis=0"), 6, "axis=1");
    second.insert(second.find("\nname=") + 6, "b");
    for (const auto* pair : {"20|100,20|100", "20|100,", ",20|100", "20|100,20|101", "20|100,21|100"}) {
      const std::string first = std::string(pair).substr(0, std::string(pair).find(','));
      const std::string other = std::string(pair).substr(std::string(pair).find(',') + 1);
      std::string encoded = legacy_profile + (first.empty() ? "" : "jog_jerk=" + first + "\n") +
          second + (other.empty() ? "" : "jog_jerk=" + other + "\n");
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 2;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      assert(load_axis_profiles(path, native_options, &native_profiles, &native_sdos) == (std::string(pair) == "20|100,20|100"));
    }
  }
  // Per-joint drive tuning (owner decision 2026-10-07): mixed-width startup
  // writes keep their sign and width; an observe-only list joins machine
  // identity only when present and may not name a key or object written on
  // the same axis (its startup readback already journals it).
  {
    auto load = [&](const std::string& sdos, const char* observe) {
      std::string encoded = legacy_profile;
      encoded.replace(encoded.find("startup_sdos=\n"), 14, "startup_sdos=" + sdos + "\n");
      if (observe) encoded += std::string("startup_observe=") + observe + "\n";
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 1;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
      return std::make_tuple(accepted, native_profiles, native_sdos);
    };
    const std::string mixed = "a|u16|8193|20|1;b|i16|8194|59|-123;c|u32|8193|2|4294967295;d|i8|8194|3|-1";
    auto [accepted, profiles, sdos] = load(mixed, "e|u16|8192|7;f|i16|8194|60;g|u32|8194|4");
    assert(accepted && sdos.size() == 4);
    assert(sdos[1].type == StartupSdoValueType::kI16 && sdos[1].values[0] == -123);
    assert(sdos[2].type == StartupSdoValueType::kU32 && sdos[2].values[0] == 4294967295LL);
    assert(sdos[3].type == StartupSdoValueType::kI8 && sdos[3].values[0] == -1);
    for (const auto& d : sdos) {
      const auto raw = startup_sdo_encode(d.type, d.values[0]);
      assert(startup_sdo_decode(d.type, raw.data()) == d.values[0]);
    }
    assert(startup_sdo_encode(StartupSdoValueType::kI16, -123)[0] == 0x85 && startup_sdo_encode(StartupSdoValueType::kI16, -123)[1] == 0xff);
    const auto& observe = profiles[0].startup_observe;
    assert(observe.size() == 3 && observe[1].type == StartupSdoValueType::kI16 && observe[1].index == 0x2002 && observe[1].subindex == 0x3c);
    auto fields = id::machine_fields(options, profiles, sdos);
    assert(fields.at("axis.0.startup_sdos") == mixed);
    assert(fields.at("axis.0.startup_observe") == "e|u16|8192|7;f|i16|8194|60;g|u32|8194|4");
    auto [plain_accepted, plain_profiles, plain_sdos] = load(mixed, nullptr);
    assert(plain_accepted && plain_profiles[0].startup_observe.empty());
    assert(!id::machine_fields(options, plain_profiles, plain_sdos).count("axis.0.startup_observe"));
    for (const char* refused : {"", "e|u16|8192", "e|u17|8192|7", "e|u16|8192|7;e|u16|8192|8", "e|u16|8192|7;x|u16|8192|7",
                                "a|u16|9|9", "z|u16|8193|20", "bad-key|u16|1|1", "e|u16|0|7", "e|u16|8192|256"})
      assert(!std::get<0>(load(mixed, refused)));
    std::string full, over;
    for (int k = 1; k <= 33; ++k) {
      const std::string entry = "k" + std::to_string(k) + "|u16|8193|" + std::to_string(k) + "|0";
      if (k <= 32) full += (full.empty() ? "" : ";") + entry;
      over += (over.empty() ? "" : ";") + entry;
    }
    assert(std::get<0>(load(full, nullptr)) && !std::get<0>(load(over, nullptr)));
    std::string observe64, observe65;
    for (int k = 1; k <= 65; ++k) {
      const std::string entry = "o" + std::to_string(k) + "|u16|8194|" + std::to_string(k);
      if (k <= 64) observe64 += (observe64.empty() ? "" : ";") + entry;
      observe65 += (observe65.empty() ? "" : ";") + entry;
    }
    assert(std::get<0>(load("", observe64.c_str())) && !std::get<0>(load("", observe65.c_str())));
  }
  // Storage policy (write-on-change, RAM-only trials): parameter_storage names
  // this axis's storage-mode key and its RAM-only value; startup_trial names
  // keys written with storage off. Both are machine identity only when present.
  {
    const std::string sdos = "s|u16|8211|17|1;a|u16|8192|7|800;b|u16|8193|21|500";
    auto load = [&](const char* storage, const char* trial) {
      std::string encoded = legacy_profile;
      encoded.replace(encoded.find("startup_sdos=\n"), 14, "startup_sdos=" + sdos + "\n");
      if (storage) encoded += std::string("parameter_storage=") + storage + "\n";
      if (trial) encoded += std::string("startup_trial=") + trial + "\n";
      std::ofstream replacement(path); replacement << encoded; replacement.close();
      Options native_options; native_options.num_axes = 1;
      std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES> native_profiles{};
      std::vector<StartupSdoConfig> native_sdos;
      const bool accepted = load_axis_profiles(path, native_options, &native_profiles, &native_sdos);
      return std::make_tuple(accepted, native_profiles, native_sdos);
    };
    auto [accepted, profiles, parsed_sdos] = load("s|0", "a");
    assert(accepted && parsed_sdos.size() == 3);
    assert(parsed_sdos[0].storage_mode && parsed_sdos[0].storage_off == 0 && parsed_sdos[0].values[0] == 1 && !parsed_sdos[0].ram_only);
    assert(parsed_sdos[1].ram_only && !parsed_sdos[1].storage_mode);
    assert(!parsed_sdos[2].ram_only && !parsed_sdos[2].storage_mode);
    auto fields = id::machine_fields(options, profiles, parsed_sdos);
    assert(fields.at("axis.0.startup_sdos") == sdos);
    assert(fields.at("axis.0.parameter_storage") == "s|0" && fields.at("axis.0.startup_trial") == "a");
    auto [plain_accepted, plain_profiles, plain_sdos] = load(nullptr, nullptr);
    assert(plain_accepted && !plain_sdos[0].storage_mode && !plain_sdos[1].ram_only);
    const auto plain = id::machine_fields(options, plain_profiles, plain_sdos);
    assert(!plain.count("axis.0.parameter_storage") && !plain.count("axis.0.startup_trial"));
    auto [verify_accepted, verify_profiles, verify_sdos] = load("s|0", nullptr);
    assert(verify_accepted && verify_sdos[0].storage_mode && !verify_sdos[1].ram_only);
    // A trial needs the storage object; names must be this axis's keys; the
    // RAM-only value must fit the type and differ from the store value.
    for (const auto& refused : std::vector<std::pair<const char*, const char*>>{
             {nullptr, "a"}, {"s|0", "z"}, {"s|0", "s"}, {"s|0", "a;a"}, {"s|0", ""}, {"x|0", nullptr},
             {"s|1", nullptr}, {"s|-1", nullptr}, {"s|65536", nullptr}, {"s", nullptr}, {"s|0|1", nullptr}, {"s|zero", nullptr}})
      assert(!std::get<0>(load(refused.first, refused.second)));
  }
  const auto expected = id::canonical(id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos));
  auto changed = id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos);
  changed["axis.0.limits"] = "-1|2|1|0";
  changed["max_rpm"] = "200";
  assert(id::verify(machine_sha,expected,id::canonical(changed)) == "axis.0.limits");
  assert(id::verify(machine_sha,expected,expected).empty());
  assert(id::verify(machine_sha,"",id::canonical(changed)) == "machine_sha256 (expected field table unavailable)");
  assert(id::verify(machine_sha,id::canonical(changed),expected) == "machine_sha256 (identity artifact digest mismatch)");
  options.rt_cpu = 7;
  options.socket_path = "/run/changed.sock";
  assert(id::digest(id::canonical(id::machine_fields(options,parsed.axis_profiles,parsed.startup_sdos))) == machine_sha);
  auto identity = rosie::rtcore::CoordinateIdentity{1,1,1,options.machine_digest};
  auto other = identity;
  other.coordinate_identity_sha256[0] ^= 1;
  assert(!(identity == other));
  assert(identity == identity);
  const rosie::rtcore::CoordinatePolicy policy{1000,10000,0,0,true,0};
  const rosie::rtcore::AcquisitionEvidence evidence{1,1000000000,0,0,true,true,true};
  const rosie::rtcore::CoordinateSample sample{0,evidence};
  const rosie::rtcore::CoordinateAnchor anchor{identity,1,rosie::rtcore::CoordinateAnchorSource::AbsolutePosition,0,evidence,true};
  assert(rosie::rtcore::reconcile_coordinate({},identity,policy,sample,anchor).accepted);
  assert(!rosie::rtcore::reconcile_coordinate({},other,policy,sample,anchor).accepted);
  std::remove(path);
}
