#pragma once

// Compile-time detection of IgH libecrt availability.
//
// When IgH is installed, `ecrt.h` should be available and RTCore will compile the
// real EtherCAT loop. Without it, RTCore builds in "IPC-only" mode.
//
// IMPORTANT:
// We gate on a build-system define (`GRADIENT_ECRT_ENABLED`) rather than using
// `__has_include(<ecrt.h>)` so that we also link the correct libraries when the
// header is present.

#ifndef GRADIENT_ECRT_ENABLED
#define GRADIENT_ECRT_ENABLED 0
#endif

#if defined(ROSIE_RT_SIMULATION) && ROSIE_RT_SIMULATION
namespace rosie::rtcore {
// Measured backend contract: bare -D and =1 both select the simulated bus.
inline constexpr bool kSimulationBackend = true;
}
#define GRADIENT_HAVE_ECRT 1
#include "simulated_bus.hpp"
#else
namespace rosie::rtcore {
// Measured backend contract: absent or zero simulation define forbids fallback.
inline constexpr bool kSimulationBackend = false;
}
#if GRADIENT_ECRT_ENABLED
#define GRADIENT_HAVE_ECRT 1
extern "C" {
#include <ecrt.h>
}
// The checked bus boundary consumes return codes that older IgH APIs discarded.
// The build pins the 1.6.9 package; check the actually included header too, since
// explicit compiler flags can point at a different installation than pkg-config.
#if !defined(ECRT_VER_MAJOR) || !defined(ECRT_VER_MINOR) || ECRT_VER_MAJOR != 1 || ECRT_VER_MINOR != 6
#error "RT-core requires the IgH 1.6 API from the pinned 1.6.9 package"
#endif
#include <type_traits>
static_assert(std::is_same_v<decltype(ecrt_master_application_time(nullptr, 0)), int> &&
              std::is_same_v<decltype(ecrt_master_receive(nullptr)), int> &&
              std::is_same_v<decltype(ecrt_domain_process(nullptr)), int> &&
              std::is_same_v<decltype(ecrt_domain_queue(nullptr)), int> &&
              std::is_same_v<decltype(ecrt_master_send(nullptr)), int>,
              "IgH cyclic transport operations must return checked int results");
static_assert(std::is_same_v<decltype(ecrt_master_sync_reference_clock_to(nullptr, 0)), int> &&
              std::is_same_v<decltype(ecrt_master_sync_slave_clocks(nullptr)), int>,
              "IgH clock operations must return checked int results");
static_assert(std::is_same_v<decltype(ecrt_domain_state(nullptr, nullptr)), int> &&
              std::is_same_v<decltype(ecrt_master_state(nullptr, nullptr)), int> &&
              std::is_same_v<decltype(ecrt_slave_config_state(nullptr, nullptr)), int>,
              "IgH state operations must return checked int results");
#else
#define GRADIENT_HAVE_ECRT 0
#endif
#endif
