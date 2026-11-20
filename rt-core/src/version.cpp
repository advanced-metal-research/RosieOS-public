// Without the Makefile's identity the header would define the function as a
// call to itself; refuse to compile instead of printing a fabricated version.
#ifndef ROSIE_RT_VERSION
#error "src/version.cpp requires -DROSIE_RT_VERSION from the Makefile's source identity"
#endif
#include "source_version.hpp"

const char* rosie_rt_source_version() noexcept { return ROSIE_RT_VERSION; }
