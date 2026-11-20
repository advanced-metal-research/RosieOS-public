#pragma once

#include "publication.hpp"
#include "control_protocol.hpp"
#include "protocol_generated.hpp"
#include "shared_words.hpp"
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <filesystem>
#include <limits>
#include <semaphore.h>

namespace rosie::rtcore {
// Unverified admission limits: 512 slots cover 400 cycles per 100 ms at 250 us
// plus 112 slots of scheduling margin. Stalls beyond capacity report loss.
inline constexpr size_t kCycleCaptureCapacity = 512;
// Unverified commissioning file budget: at most 65536 attempted cycle records.
inline constexpr uint32_t kMaxCaptureCycles = 65536;
// Measured schema bound: absolute feedback configuration allows 16 fields/axis.
inline constexpr size_t kCaptureAbsoluteFields = 16;

using CycleCaptureAxis = gradient::ipc::v1::CycleCaptureAxisV2;
// Preserve fast-trace t_ns, seq, ax[i,p,tp,sw,er,mfr,af[k,v,ok]]. Immutable
// configured af keys are resolved by the reader; cycle identifies actual cycles
// and work_ns measures wake through payload publication, sampled immediately
// before the final work_ns/stamp stores (excluding sleep). The maximum cycle-work
// metric also includes those final stores. Measured WSL/g++11 -O2, 10000 steps,
// 16 axes, V2: wake-to-submission mean 1718 ns, wake-to-publication mean 4334 ns,
// delta 2616 ns. These observations are not a hardware admission bound.
using CycleCaptureRecord = gradient::ipc::v1::CycleCaptureRecordV2;

// Unverified resident ceiling: 2 GiB, including the 4096-byte mapping header.
inline constexpr uint64_t kTelemetryMaxBytes = uint64_t{2} * 1024 * 1024 * 1024;
// Measured V2 geometry at 16 axes/250000 ns: 100000 ms uses 1955204096 bytes;
// 300000 ms would use 5865604096 bytes, above the 2 GiB resident ceiling.
// Unverified default retention: 100000 ms; minimum remains one publisher period.
inline constexpr uint64_t kTelemetryDefaultRetentionMs = 100000;
// Measured publisher period: 100000000 ns.
inline constexpr uint64_t kTelemetryMinimumRetentionMs = 100;
// Unverified retention ceiling: 3600000 ms, additionally constrained by bytes.
inline constexpr uint64_t kTelemetryMaximumRetentionMs = 3600000;
// Measured V2 ABI: 4096-byte metadata page, then stamp + 272 + 288*axes bytes/slot.
inline constexpr size_t kTelemetryHeaderBytes = 4096;
// Unverified trigger admission: 64 events per producer between publisher drains.
inline constexpr size_t kTelemetryTriggerCapacity = 64;
// Unverified metadata cap: 262144 bytes; reject larger trigger/key metadata.
inline constexpr size_t kTelemetryDumpHeaderBytes = 262144;

struct TelemetryTrigger {
  uint64_t end = 0;
  uint32_t reason = 0, length = 0;
  uint32_t fault_reason = 0;
  // Measured axis sentinel: UINT32_MAX denotes a group Stop/mark/exit.
  uint32_t axis = UINT32_MAX;
  std::array<uint8_t, 128> label{};
};

struct TelemetryDumpRequest {
  std::array<TelemetryTrigger, 2 * kTelemetryTriggerCapacity> triggers{};
  size_t count = 0;
  uint64_t end = 0;
  bool skipped = false;
};

// One publisher posts only after a successful bounded enqueue; shutdown posts
// once after that publisher joins. The worker consumes one post per request,
// preventing both lost wakeups and an unbounded notification count. No cyclic
// path calls this semaphore, and posting never waits for the disk worker.
class TelemetryDumpWake {
 public:
  TelemetryDumpWake() : ready_(sem_init(&semaphore_, 0, 0) == 0) {}
  ~TelemetryDumpWake() { if (ready_) sem_destroy(&semaphore_); }
  TelemetryDumpWake(const TelemetryDumpWake&) = delete;
  TelemetryDumpWake& operator=(const TelemetryDumpWake&) = delete;
  bool post() noexcept { return ready_ && sem_post(&semaphore_) == 0; }
  bool wait() noexcept {
    if (!ready_) return false;
    int result;
    do { result = sem_wait(&semaphore_); } while (result != 0 && errno == EINTR);
    return result == 0;
  }
 private:
  sem_t semaphore_{};
  bool ready_;
};

// Permanent daemon-lifetime mapping: create/prefault before starting threads,
// destroy only after cycle, publisher and dump worker join. A memfd prevents continuous
// eMMC writes even when the runtime directory is on a persistent filesystem.
// Same-UID readers use /proc; the protected companion Unix socket transfers
// only an O_RDONLY descriptor to the separately owned rt-control service.
// The intended trust boundary is the protected runtime directory plus socket
// mode 0660 and ipc group membership, including supplementary groups. Every
// admitted member may observe all axes; SO_PEERCRED UID filtering would narrow
// that intentional reader policy. The sealed O_RDONLY fd grants no command,
// write or truncate authority and cannot be used to change cyclic state.
class TelemetryRing {
 public:
  TelemetryRing() = default;
  TelemetryRing(const TelemetryRing&) = delete;
  TelemetryRing& operator=(const TelemetryRing&) = delete;
  ~TelemetryRing() {
    if (!path_.empty()) ::unlink(path_.c_str());
    if (!socket_path_.empty()) ::unlink(socket_path_.c_str());
    if (listener_ >= 0) ::close(listener_);
    if (read_fd_ >= 0) ::close(read_fd_);
    if (words_) ::munmap(words_, bytes_);
    if (fd_ >= 0) ::close(fd_);
  }
  static size_t record_bytes(uint32_t axes) noexcept {
    return offsetof(CycleCaptureRecord, ax) + axes * sizeof(CycleCaptureAxis);
  }
  static uint64_t mapping_bytes(uint64_t period, uint64_t retention_ms, uint32_t axes) noexcept {
    if (!period || !axes || axes > rosie::protocol::max_axes ||
        retention_ms < kTelemetryMinimumRetentionMs || retention_ms > kTelemetryMaximumRetentionMs) return 0;
    const uint64_t slots = (retention_ms * 1000000 + period - 1) / period;
    const uint64_t bytes = kTelemetryHeaderBytes + slots * (8 + record_bytes(axes));
    return bytes <= kTelemetryMaxBytes ? bytes : 0;
  }
  bool create(const std::string& path, uint64_t period, uint64_t retention_ms,
              uint32_t axes, gid_t group, const std::array<uint8_t, 16>& incarnation = {}) {
    if (words_ || !(bytes_ = mapping_bytes(period, retention_ms, axes))) return false;
    record_bytes_ = record_bytes(axes);
    stride_words_ = (8 + record_bytes_) / 8;
    capacity_ = (bytes_ - kTelemetryHeaderBytes) / (stride_words_ * 8);
    fd_ = ::memfd_create("rt-telemetry", MFD_CLOEXEC | MFD_ALLOW_SEALING);
    if (fd_ < 0 || ::ftruncate(fd_, bytes_) != 0) return false;
    void* mapped = ::mmap(nullptr, bytes_, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, 0);
    if (mapped == MAP_FAILED) return false;
    words_ = static_cast<uint64_t*>(mapped);
    std::memset(words_, 0, bytes_); // Touch every writable page before RT admission.
    // Seal extent changes; existing read-only readers cannot truncate a live ring.
    if (::fcntl(fd_, F_ADD_SEALS, F_SEAL_SHRINK | F_SEAL_GROW | F_SEAL_SEAL) != 0) return false;
    if (group != static_cast<gid_t>(-1) && ::fchown(fd_, -1, group) != 0) return false;
    if (::fchmod(fd_, 0440) != 0) return false;
    // Measured wire magic: eight little-endian bytes identifying telemetry v1.
    words_[0] = 0x31474e4952545452ULL;
    words_[1] = rosie::protocol::version_major | (uint64_t(rosie::protocol::version_minor) << 32);
    words_[2] = bytes_; words_[3] = capacity_; words_[4] = record_bytes_;
    words_[5] = period; words_[6] = axes;
    // Immutable daemon identity at bytes 72..87, installed before exposing
    // either observer endpoint so a replaced ring cannot impersonate a connection.
    std::memcpy(words_ + 9, incarnation.data(), incarnation.size());
    std::memcpy(words_ + 16, gradient::ipc::v1::CycleCaptureRecordV2LayoutDigest, 64);
    if (!path.empty()) {
      const auto target = "/proc/" + std::to_string(::getpid()) + "/fd/" + std::to_string(fd_);
      // The socket owner excludes another daemon. Replace only our fixed symlink,
      // refusing regular files and directories instead of clobbering user data.
      struct stat st{};
      if (::lstat(path.c_str(), &st) == 0) {
        if (!S_ISLNK(st.st_mode) || ::unlink(path.c_str()) != 0) return false;
      } else if (errno != ENOENT) return false;
      if (::symlink(target.c_str(), path.c_str()) != 0) return false;
      path_ = path;
      read_fd_ = ::open(target.c_str(), O_RDONLY | O_CLOEXEC);
      if (read_fd_ < 0) return false;
      const auto socket_path = path + ".sock";
      sockaddr_un address{}; address.sun_family = AF_UNIX;
      if (socket_path.size() >= sizeof(address.sun_path)) return false;
      std::memcpy(address.sun_path, socket_path.c_str(), socket_path.size() + 1);
      if (::lstat(socket_path.c_str(), &st) == 0) {
        if (!S_ISSOCK(st.st_mode) || ::unlink(socket_path.c_str()) != 0) return false;
      } else if (errno != ENOENT) return false;
      listener_ = ::socket(AF_UNIX, SOCK_SEQPACKET | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
      if (listener_ < 0 || ::bind(listener_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0)
        return false;
      socket_path_ = socket_path;
      if (group != static_cast<gid_t>(-1) && ::chown(socket_path.c_str(), -1, group) != 0) return false;
      // Unverified reader admission: 16 pending connections, protected by the
      // native runtime directory and the same group as the control socket.
      if (::chmod(socket_path.c_str(), 0660) != 0 || ::listen(listener_, 16) != 0) return false;
    }
    return true;
  }
  bool active() const noexcept { return words_ != nullptr; }
  // Main thread only. Fixed admission work transfers a read-only descriptor;
  // neither slow readers nor disk dumps can hold the cyclic owner or this loop.
  void serve_readers() noexcept {
    // Unverified accept budget: eight descriptor transfers per main-loop pass.
    for (unsigned n = 0; n < 8 && listener_ >= 0; ++n) {
      const int client = ::accept4(listener_, nullptr, nullptr, SOCK_CLOEXEC | SOCK_NONBLOCK);
      if (client < 0) break;
      char tag = 'T';
      iovec vector{&tag, 1};
      alignas(cmsghdr) char ancillary[CMSG_SPACE(sizeof(int))]{};
      msghdr message{}; message.msg_iov = &vector; message.msg_iovlen = 1;
      message.msg_control = ancillary; message.msg_controllen = sizeof(ancillary);
      auto* control = CMSG_FIRSTHDR(&message);
      control->cmsg_level = SOL_SOCKET; control->cmsg_type = SCM_RIGHTS;
      control->cmsg_len = CMSG_LEN(sizeof(int));
      std::memcpy(CMSG_DATA(control), &read_fd_, sizeof(read_fd_));
      const auto sent = ::sendmsg(client, &message, MSG_DONTWAIT | MSG_NOSIGNAL);
      (void)sent;
      ::close(client);
    }
  }
  uint64_t capacity() const noexcept { return capacity_; }
  uint64_t bytes() const noexcept { return bytes_; }
  uint64_t written() const noexcept { return words_ ? __atomic_load_n(words_ + 7, __ATOMIC_SEQ_CST) : 0; }
  uint64_t dropped() const noexcept { return words_ ? __atomic_load_n(words_ + 8, __ATOMIC_SEQ_CST) : 0; }

  // Exactly one cyclic writer, no retry, allocation, lock or syscall. Sequence
  // stamps AND payload words are atomic, preventing undefined torn C++ reads.
  // Drops count overwritten window records, independent of observer cursors.
  // Keep payload and stamps sequentially consistent: in their single order,
  // any payload from a later write is preceded by its odd stamp, so the final
  // equal-stamp check cannot accept it. Go atomic.LoadUint64 is already
  // sequentially consistent. Relaxed payload stores would require a writer-side
  // release fence between the odd stamp and payload, plus release on the even
  // stamp; release stamps alone do not supply that ordering proof.
  // The supplied existing cycle clock measures through the payload copy; work_ns
  // is installed last, before publication, avoiding a self-referential timestamp.
  template<class Clock>
  bool push(CycleCaptureRecord& record, uint64_t begin, Clock clock) noexcept {
    if (!words_) return false;
    const uint64_t index = written();
    if (index >= (std::numeric_limits<uint64_t>::max() / 2) - 1) return false;
    record.seq = index + 1;
    auto* slot = words_ + kTelemetryHeaderBytes / 8 + (index % capacity_) * stride_words_;
    __atomic_store_n(slot, record.seq * 2 - 1, __ATOMIC_SEQ_CST);
    for (size_t i = 0; i < record_bytes_ / 8; ++i) {
      uint64_t word; std::memcpy(&word, reinterpret_cast<const uint8_t*>(&record) + i * 8, 8);
      __atomic_store_n(slot + 1 + i, word, __ATOMIC_SEQ_CST);
    }
    record.work_ns = clock() - begin;
    __atomic_store_n(slot + 4, record.work_ns, __ATOMIC_SEQ_CST);
    if (index >= capacity_) __atomic_store_n(words_ + 8, index + 1 - capacity_, __ATOMIC_SEQ_CST);
    __atomic_store_n(slot, record.seq * 2, __ATOMIC_SEQ_CST);
    __atomic_store_n(words_ + 7, record.seq, __ATOMIC_SEQ_CST);
    return true;
  }
  // A bounded single attempt leaves output untouched on busy/overwritten slots.
  // Readers own cursors; a stalled observer can never hold up cyclic publication.
  bool read(uint64_t sequence, CycleCaptureRecord& output) const noexcept {
    if (!sequence || sequence > written()) return false;
    auto* slot = words_ + kTelemetryHeaderBytes / 8 + ((sequence - 1) % capacity_) * stride_words_;
    if (__atomic_load_n(slot, __ATOMIC_SEQ_CST) != sequence * 2) return false;
    CycleCaptureRecord candidate{};
    for (size_t i = 0; i < record_bytes_ / 8; ++i) {
      const uint64_t word = __atomic_load_n(slot + 1 + i, __ATOMIC_SEQ_CST);
      std::memcpy(reinterpret_cast<uint8_t*>(&candidate) + i * 8, &word, 8);
    }
    if (__atomic_load_n(slot, __ATOMIC_SEQ_CST) != sequence * 2) return false;
    output = candidate;
    return true;
  }
 private:
  uint64_t* words_ = nullptr;
  uint64_t bytes_ = 0, capacity_ = 0;
  size_t record_bytes_ = 0, stride_words_ = 0;
  int fd_ = -1;
  int listener_ = -1, read_fd_ = -1;
  std::string path_, socket_path_;
};

// One RT writer, one publisher reader; arm once before acquisition. push never
// waits, allocates, retries or overwrites unread evidence. Exhausted/full capture
// consumes its finite attempt budget and reports exactly each lost record.
class CycleCapture {
  static_assert(std::atomic<uint32_t>::is_always_lock_free);
 public:
  bool arm(uint32_t cycles) noexcept {
    if (!cycles || cycles > kMaxCaptureCycles || armed_) return false;
    armed_ = true; // Reader-only: the CLI permits one capture per process.
    remaining_.store(cycles, std::memory_order_release);
    return true;
  }
  bool active() const noexcept { return remaining_.load(std::memory_order_acquire) != 0; }
  bool push(const CycleCaptureRecord& record) noexcept {
    const auto remaining = remaining_.load(std::memory_order_acquire);
    if (!remaining) return false;
    const bool accepted = queue_.push(record);
    if (!accepted) dropped_.fetch_add(1, std::memory_order_relaxed);
    remaining_.store(remaining - 1, std::memory_order_release);
    return accepted;
  }
  bool pop(CycleCaptureRecord& record) noexcept { return queue_.pop(record); }
  uint64_t dropped() const noexcept { return dropped_.load(std::memory_order_relaxed); }
 private:
  SpscQueue<CycleCaptureRecord, kCycleCaptureCapacity> queue_;
  std::atomic<uint32_t> remaining_{0};
  std::atomic<uint64_t> dropped_{0};
  bool armed_ = false;
};
}
