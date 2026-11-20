#pragma once
#include <cstddef>

// Measured Ubuntu 22.04/GCC 11.4 runtime: libtsan intercepts
// pthread_mutex_timedlock/pthread_cond_timedwait but neither clocklock nor
// clockwait. Select libstdc++'s existing fallbacks so ownership remains visible instead
// of producing false unlock/race reports. Only sanitizer builds include this.
// GCC's upstream fix: commit 3b7cb33033fbe6af6c1c4ef014f7353479d1dd6b.
// Undefine rather than set to zero: GCC 11's mutex header mixes #if and #ifdef.
#if defined(__SANITIZE_THREAD__) && defined(_GLIBCXX_RELEASE) && _GLIBCXX_RELEASE == 11
#undef _GLIBCXX_USE_PTHREAD_MUTEX_CLOCKLOCK
#undef _GLIBCXX_USE_PTHREAD_COND_CLOCKWAIT
#endif
