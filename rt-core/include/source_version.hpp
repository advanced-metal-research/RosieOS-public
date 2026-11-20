#pragma once

// The daemon's source identity is linked from src/version.cpp, the only
// translation unit compiled with it: a commit that changes no C++ recompiles
// that file and relinks, instead of recompiling src/main.cpp per variant.
// Fixtures that define ROSIE_RT_VERSION before this header keep their own
// literal and need no link-time identity.
const char* rosie_rt_source_version() noexcept;
#ifndef ROSIE_RT_VERSION
#define ROSIE_RT_VERSION rosie_rt_source_version()
#endif
