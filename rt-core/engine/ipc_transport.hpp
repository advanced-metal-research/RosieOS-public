#pragma once

// Included inside the daemon namespace after options and logging helpers.
#include <sys/file.h>
#include "socket_owner.hpp"

struct ShmRegion {
  int fd = -1;
  void* base = nullptr;
  size_t bytes = 0;

  ShmRegion() = default;
  ShmRegion(const ShmRegion&) = delete;
  ShmRegion& operator=(const ShmRegion&) = delete;

  ShmRegion(ShmRegion&& other) noexcept
      : fd(other.fd), base(other.base), bytes(other.bytes) {
    other.fd = -1;
    other.base = nullptr;
    other.bytes = 0;
  }

  ShmRegion& operator=(ShmRegion&& other) noexcept {
    if (this == &other) {
      return *this;
    }
    reset();
    fd = other.fd;
    base = other.base;
    bytes = other.bytes;
    other.fd = -1;
    other.base = nullptr;
    other.bytes = 0;
    return *this;
  }

  void reset() {
    if (base && bytes) {
      munmap(base, bytes);
    }
    if (fd >= 0) {
      close(fd);
    }
    fd = -1;
    base = nullptr;
    bytes = 0;
  }

  ~ShmRegion() { reset(); }
};

int set_cloexec(int fd) {
  int flags = fcntl(fd, F_GETFD);
  if (flags < 0) {
    return -1;
  }
  return fcntl(fd, F_SETFD, flags | FD_CLOEXEC);
}

ShmRegion create_memfd_region(const char* name, size_t bytes) {
  ShmRegion region;

  int fd = -1;
#ifdef __linux__
  // memfd_create() is declared when _GNU_SOURCE is enabled.
  fd = memfd_create(name, MFD_CLOEXEC | MFD_ALLOW_SEALING);
#endif
  if (fd < 0) {
    logf("ERROR: memfd_create('%s') failed: %s", name, std::strerror(errno));
    return region;
  }

  if (ftruncate(fd, static_cast<off_t>(bytes)) != 0) {
    logf("ERROR: ftruncate(memfd:%s, %zu) failed: %s", name, bytes,
         std::strerror(errno));
    close(fd);
    return region;
  }

  // Shared mappings must not be truncated underneath either process. These
  // seals preserve writable payloads while fixing the negotiated geometry.
  if (fcntl(fd, F_ADD_SEALS, F_SEAL_GROW | F_SEAL_SHRINK | F_SEAL_SEAL) != 0) {
    logf("ERROR: sealing memfd '%s' failed: %s", name, std::strerror(errno));
    close(fd);
    return region;
  }

  void* base =
      mmap(nullptr, bytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  if (base == MAP_FAILED) {
    logf("ERROR: mmap(memfd:%s, %zu) failed: %s", name, bytes,
         std::strerror(errno));
    close(fd);
    return region;
  }

  region.fd = fd;
  region.base = base;
  region.bytes = bytes;
  return region;
}

bool eventfd_write_one(int efd) {
  uint64_t one = 1;
  ssize_t n = write(efd, &one, sizeof(one));
  if (n == static_cast<ssize_t>(sizeof(one))) {
    return true;
  }
  if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
    return false;
  }
  return false;
}

bool eventfd_drain(int efd) {
  uint64_t value = 0;
  while (true) {
    ssize_t n = read(efd, &value, sizeof(value));
    if (n == static_cast<ssize_t>(sizeof(value))) {
      continue;
    }
    if (n < 0 && (errno == EAGAIN || errno == EINTR)) {
      return true;
    }
    return (n == 0);
  }
}

struct RingView {
  gradient::ipc::v1::RingHeaderV1* header = nullptr;
  uint8_t* entries = nullptr;
  uint32_t capacity = 0;
  uint32_t msg_bytes = 0;
};

RingView make_ring_view(void* shm_base, const gradient::ipc::v1::ShmHeaderV1* hdr) {
  RingView view;
  if (!shm_base || !hdr) {
    return view;
  }
  auto* base = static_cast<uint8_t*>(shm_base);
  auto* ring_hdr =
      reinterpret_cast<gradient::ipc::v1::RingHeaderV1*>(base + hdr->ring_offset);

  const size_t ring_hdr_bytes_aligned = align_up(sizeof(*ring_hdr), 8);
  view.header = ring_hdr;
  view.entries = base + hdr->ring_offset + ring_hdr_bytes_aligned;
  view.capacity = hdr->ring_capacity;
  view.msg_bytes = hdr->ring_msg_bytes;
  return view;
}

bool ring_write(RingView ring,
                uint16_t type,
                const void* payload,
                size_t payload_bytes,
                uint64_t* seq_counter,
                uint64_t time_ns) {
  if (!ring.header || !ring.entries || ring.capacity == 0 || ring.msg_bytes == 0) {
    return false;
  }
  // SPSC ring: producer owns write_idx, consumer owns read_idx.
  const uint32_t w = __atomic_load_n(&ring.header->write_idx, __ATOMIC_RELAXED);
  const uint32_t r = __atomic_load_n(&ring.header->read_idx, __ATOMIC_ACQUIRE);
  if ((w - r) >= ring.capacity) {
    ring.header->dropped += 1;
    return false;
  }

  const uint32_t slot = w % ring.capacity;
  uint8_t* slot_ptr = ring.entries + static_cast<size_t>(slot) * ring.msg_bytes;
  std::memset(slot_ptr, 0, ring.msg_bytes);

  auto* mh = reinterpret_cast<gradient::ipc::v1::MsgHeader*>(slot_ptr);
  mh->type = type;
  mh->flags = 0;
  mh->bytes = static_cast<uint32_t>(sizeof(*mh) + payload_bytes);
  mh->seq = (*seq_counter)++;
  mh->time_ns = time_ns;

  if (payload && payload_bytes > 0) {
    std::memcpy(slot_ptr + sizeof(*mh), payload, payload_bytes);
  }

  // Publish.
  __atomic_store_n(&ring.header->write_idx, w + 1, __ATOMIC_RELEASE);
  return true;
}

// The lock inode persists across owners; removing it would split flock ownership.
// Hold it until the bound socket is gone, including partial startup failures.
struct SocketOwner {
  int lock_fd = -1;
  std::string path;
  struct stat bound{};
  bool owns_socket = false;
  ~SocketOwner() {
    struct stat current{};
    if (owns_socket && lstat(path.c_str(), &current) == 0 &&
        current.st_dev == bound.st_dev && current.st_ino == bound.st_ino)
      unlink(path.c_str());
    if (lock_fd >= 0) close(lock_fd);
  }
};

// Probe before taking the lock, then repeat under it. A successful connection
// proves a live endpoint even when HELLO is refused or its response is delayed.
socket_owner::Probe probe_socket_owner(const std::string& path) {
  struct stat st{};
  if (lstat(path.c_str(), &st) != 0)
    return errno == ENOENT ? socket_owner::Probe::Missing : socket_owner::Probe::Unknown;
  if (!S_ISSOCK(st.st_mode) || st.st_uid != geteuid()) return socket_owner::Probe::Unknown;
  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (path.size() >= sizeof(addr.sun_path)) return socket_owner::Probe::Unknown;
  std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
  const int fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
  if (fd < 0) return socket_owner::Probe::Unknown;
  const int rc = connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr));
  const int error = errno;
  if (rc == 0) {
    // Invalid role asks for a refused HELLO without becoming the controller
    // or disturbing the live owner's application grant.
    gradient::ipc::v1::HelloV1 hello{};
    (void)send(fd, &hello, sizeof(hello), MSG_NOSIGNAL);
  }
  close(fd);
  if (rc == 0) return socket_owner::Probe::Live;
  return error == ECONNREFUSED ? socket_owner::Probe::Refused : socket_owner::Probe::Unknown;
}

struct IpcServerResult {
  int fd = -1;
  gid_t gid = static_cast<gid_t>(-1);
  int exit_code = 0;
  std::unique_ptr<SocketOwner> owner;
};

// Publish the description and bind the protected bootstrap socket in startup
// order. Preserve the original exit status when any required operation fails.
IpcServerResult create_ipc_server(const Options& opt,
    const std::array<AxisDriveProfile, gradient::ipc::v1::GRADIENT_MAX_AXES>& axis_profiles) {
  IpcServerResult result;
  // Create parent directory for the socket.
  try {
    std::filesystem::path sock_path(opt.socket_path);
    std::filesystem::path parent = sock_path.parent_path();
    if (!parent.empty()) {
      std::filesystem::create_directories(parent);
    }
  } catch (const std::exception& e) {
    logf("ERROR: failed to create socket directory for %s: %s",
         opt.socket_path.c_str(), e.what());
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }

  auto owner = std::make_unique<SocketOwner>();
  owner->path = opt.socket_path;
  auto probe = probe_socket_owner(opt.socket_path);
  if (probe == socket_owner::Probe::Live) {
    logf("socket owned by a live process");
    return {-1, static_cast<gid_t>(-1), 3, nullptr};
  }
  if (probe == socket_owner::Probe::Unknown) {
    logf("ERROR: socket ownership cannot be verified");
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  const auto lock_path = opt.socket_path + ".lock";
  // A refused socket without a predecessor lock has unverified ownership.
  // Only first startup may create the lock inode used to prove later recovery.
  owner->lock_fd = open(lock_path.c_str(), O_RDWR | O_CLOEXEC | O_NOFOLLOW | O_NONBLOCK |
      (probe == socket_owner::Probe::Missing ? O_CREAT : 0), 0600);
  struct stat lock_stat{}, path_stat{};
  if (owner->lock_fd < 0 || fstat(owner->lock_fd, &lock_stat) != 0 ||
      !S_ISREG(lock_stat.st_mode) || lock_stat.st_uid != geteuid() ||
      lock_stat.st_nlink != 1 || (lock_stat.st_mode & 0022) != 0) {
    logf("ERROR: socket lock ownership cannot be verified");
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  if (flock(owner->lock_fd, LOCK_EX | LOCK_NB) != 0) {
    const bool busy = errno == EWOULDBLOCK || errno == EAGAIN;
    logf(busy ? "socket owned by a live process" : "ERROR: socket owner lock failed");
    return {-1, static_cast<gid_t>(-1), busy ? 3 : 1, nullptr};
  }
  if (lstat(lock_path.c_str(), &path_stat) != 0 || path_stat.st_dev != lock_stat.st_dev ||
      path_stat.st_ino != lock_stat.st_ino) {
    logf("ERROR: socket lock path changed");
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  probe = probe_socket_owner(opt.socket_path);
  const auto decision = socket_owner::decide(probe, true, true);
  if (decision == socket_owner::Decision::LiveOwner || decision == socket_owner::Decision::Refuse) {
    logf(decision == socket_owner::Decision::LiveOwner ? "socket owned by a live process" : "ERROR: socket ownership cannot be verified");
    return {-1, static_cast<gid_t>(-1), decision == socket_owner::Decision::LiveOwner ? 3 : 1, nullptr};
  }
  std::ifstream proc_stat("/proc/self/stat");
  std::string stat_line, start_ticks;
  std::getline(proc_stat, stat_line);
  const auto end_name = stat_line.rfind(')');
  if (end_name != std::string::npos) {
    std::istringstream fields(stat_line.substr(end_name + 1));
    for (int field = 3; field <= 22; ++field) fields >> start_ticks;
    if (!fields) start_ticks.clear();
  }
  const auto identity = "pid=" + std::to_string(getpid()) + " start_time_ticks=" + start_ticks + "\n";
  if (start_ticks.empty() || ftruncate(owner->lock_fd, 0) != 0 ||
      pwrite(owner->lock_fd, identity.data(), identity.size(), 0) != static_cast<ssize_t>(identity.size())) {
    logf("ERROR: cannot publish socket owner identity");
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  if (decision == socket_owner::Decision::Recover && unlink(opt.socket_path.c_str()) != 0) {
    logf("ERROR: cannot remove verified stale socket");
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  result.owner = std::move(owner);

  // Publish only after ownership, so a newcomer cannot replace a live description.
  try {
    const auto description_path = std::filesystem::path(opt.socket_path).parent_path() / "description.json";
    const auto temporary = description_path.string() + ".tmp";
    std::ofstream output(temporary, std::ios::trunc);
    output << describe_core(opt, axis_profiles); output.close();
    if (!output) throw std::runtime_error("description write failed");
    std::filesystem::rename(temporary, description_path);
  } catch (const std::exception& error) {
    logf("ERROR: cannot publish core description: %s", error.what()); return {-1, static_cast<gid_t>(-1), 2, nullptr};
  }

  int& server_fd = result.fd;
  server_fd = socket(AF_UNIX, SOCK_SEQPACKET, 0);
  if (server_fd < 0) {
    logf("ERROR: socket(AF_UNIX, SOCK_SEQPACKET) failed: %s", std::strerror(errno));
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  set_cloexec(server_fd);

  sockaddr_un addr{};
  addr.sun_family = AF_UNIX;
  if (opt.socket_path.size() >= sizeof(addr.sun_path)) {
    logf("ERROR: socket path too long: %s", opt.socket_path.c_str());
    close(server_fd);
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  std::strncpy(addr.sun_path, opt.socket_path.c_str(), sizeof(addr.sun_path) - 1);

  if (bind(server_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
    logf("ERROR: bind(%s) failed: %s", opt.socket_path.c_str(), std::strerror(errno));
    close(server_fd);
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }

  // Best-effort permissions for the IPC socket:
  if (lstat(opt.socket_path.c_str(), &result.owner->bound) != 0) {
    close(server_fd);
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }
  result.owner->owns_socket = true;

  // - chmod 0660 so only owner/group can connect
  // - chown the socket's group to `--ipc-group` (default: `pi`) so the Python
  //   controller (runs unprivileged) can connect.
  gid_t& ipc_gid = result.gid;
  try {
    gid_t desired_gid = static_cast<gid_t>(-1);

    if (!opt.ipc_group.empty()) {
      if (group* g = ::getgrnam(opt.ipc_group.c_str())) {
        desired_gid = g->gr_gid;
      } else {
        logf("WARNING: --ipc-group '%s' not found; falling back to parent directory group",
             opt.ipc_group.c_str());
      }
    }

    std::filesystem::path sock_path(opt.socket_path);
    std::filesystem::path parent = sock_path.parent_path();
    if (!parent.empty()) {
      struct stat st {};
      if (::stat(parent.c_str(), &st) == 0) {
        if (desired_gid == static_cast<gid_t>(-1)) {
          desired_gid = st.st_gid;
        }
      }
    }

    // Keep uid, update gid.
    if (desired_gid != static_cast<gid_t>(-1) &&
        ::chown(opt.socket_path.c_str(), static_cast<uid_t>(-1), desired_gid) != 0) {
      logf("WARNING: chown(%s) failed: %s", opt.socket_path.c_str(), std::strerror(errno));
    }
    ipc_gid = desired_gid;
  } catch (const std::exception& e) {
    logf("WARNING: failed to adjust socket ownership for %s: %s", opt.socket_path.c_str(), e.what());
  }
  chmod(opt.socket_path.c_str(), 0660);

  if (listen(server_fd, 4) != 0) {
    logf("ERROR: listen() failed: %s", std::strerror(errno));
    unlink(opt.socket_path.c_str());
    close(server_fd);
    return {-1, static_cast<gid_t>(-1), 1, nullptr};
  }

  logf("Listening on %s", opt.socket_path.c_str());
#if !GRADIENT_HAVE_ECRT
  logf("NOTE: IgH libecrt headers not found; running IPC-only mode.");
#endif

  return result;
}

enum class AcceptResult { Retry, PollFailed, Connected };

struct ConnectionState {
  // Connection-scoped state (reset on disconnect).
  int controlling_client_fd = -1;
  ShmRegion cmd_shm;
  ShmRegion status_shm;
  ShmRegion fast_shm;
  std::unique_ptr<rosie::rtcore::FastControlRegion> fast_region;
  bool fast_published = false;
  uint64_t next_connection_id = 0;
  int cmd_eventfd = -1;
  int status_eventfd = -1;
  std::atomic<bool> helper_running{false};
  std::thread helper_thread;

  bool adapter_connection = false;
  const uint64_t topology_hash = 0; // TODO(ethercat): compute from live bus
  const uint64_t build_id_hash = 0; // TODO: embed git hash

  bool report_connection_failure(rosie::rtcore::ConnectionFailureLatch& connection_failure) {
    rosie::rtcore::ConnectionFailure failure{};
    if (connection_failure.consume(next_connection_id, failure) !=
        rosie::rtcore::ConnectionFailureConsumption::Matched) return false;
    const char* reason = failure.reason == rosie::rtcore::ConnectionFailureReason::Busy
        ? "busy" : (failure.reason == rosie::rtcore::ConnectionFailureReason::SequenceExhausted
            ? "sequence_exhausted" : "invalid_publisher");
    logf("fast-control observation publication failed: connection_id=%llu reason=%s; retiring IPC state",
         static_cast<unsigned long long>(failure.connection_id), reason);
    return true;
  }

  // Retire RT borrows and join the helper before unmapping connection memory;
  // disconnect must never unmap a descriptor still borrowed by either thread.
  template<class Disconnect>
  void reset(rosie::rtcore::RtMappingHandoff<rosie::rtcore::FastControlRegion>& fast_handoff,
             rosie::rtcore::ConnectionFailureLatch& connection_failure,
             Disconnect&& disconnect_outputs) {
    disconnect_outputs();
    if (fast_published) fast_handoff.detach();
    helper_running.store(false, std::memory_order_relaxed);
    // Make terminal loss visible promptly, even if a mailbox worker has not
    // returned. Its borrowed descriptors/mappings stay alive until it exits.
    if (controlling_client_fd >= 0) shutdown(controlling_client_fd, SHUT_RDWR);
    if (helper_thread.joinable()) {
      helper_thread.join();
    }
    if (controlling_client_fd >= 0) {
      close(controlling_client_fd);
      controlling_client_fd = -1;
    }
    if (cmd_eventfd >= 0) {
      close(cmd_eventfd);
      cmd_eventfd = -1;
    }
    if (status_eventfd >= 0) {
      close(status_eventfd);
      status_eventfd = -1;
    }
    cmd_shm.reset();
    status_shm.reset();
    if (fast_published) {
      // No timeout can authorize munmap. RT acknowledges a later completed
      // borrow scope (or permanent exit), independently of the IPC helper.
      while (fast_handoff.take_retired() == nullptr)
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
      fast_published = false;
    }
    fast_region.reset();
    fast_shm.reset();
    // A failure may have raced an ordinary socket disconnect. The completed
    // RT retirement above proves no later record can arrive for this mapping.
    (void)report_connection_failure(connection_failure);
  }

  // Admit exactly one validated peer and publish its connection before regions
  // are offered. A rejected HELLO closes only the newly accepted descriptor.
  template<class PublishConnection>
  AcceptResult accept_controller(int server_fd, const std::optional<uid_t>& adapter_uid,
                                  PublishConnection&& publish_connection) {
    pollfd pfd{};
    pfd.fd = server_fd;
    pfd.events = POLLIN;
    int pr = poll(&pfd, 1, 10);
    if (pr < 0) {
      if (errno == EINTR) {
        return AcceptResult::Retry;
      }
      logf("ERROR: poll(server) failed: %s", std::strerror(errno));
      return AcceptResult::PollFailed;
    }
    if (pr == 0) {
      return AcceptResult::Retry;
    }

    if (!(pfd.revents & POLLIN)) {
      return AcceptResult::Retry;
    }

    int client_fd = accept(server_fd, nullptr, nullptr);
    if (client_fd < 0) {
      if (errno == EINTR) {
        return AcceptResult::Retry;
      }
      logf("ERROR: accept() failed: %s", std::strerror(errno));
      return AcceptResult::Retry;
    }
    set_cloexec(client_fd);

    if (controlling_client_fd >= 0) {
      logf("Rejecting additional client (single-controller policy).");
      close(client_fd);
      return AcceptResult::Retry;
    }

    // Read HELLO.
    gradient::ipc::v1::HelloV1 hello{};
    pollfd hello_poll{client_fd, POLLIN, 0};
    ssize_t n = poll(&hello_poll, 1, 100) > 0 ? recv(client_fd, &hello, sizeof(hello), MSG_DONTWAIT | MSG_TRUNC) : -1;
    if (n != static_cast<ssize_t>(sizeof(hello))) {
      logf("ERROR: HELLO read failed (got %zd bytes)", n);
      close(client_fd);
      return AcceptResult::Retry;
    }
    bool legacy_oracle_role = false;
#if defined(ROSIE_RT_SIMULATION) || !GRADIENT_ECRT_ENABLED
    legacy_oracle_role = hello.role == gradient::ipc::v1::kRoleController;
#endif
    adapter_connection = hello.role == gradient::ipc::v1::kRoleAdapter;
    if (hello.magic != gradient::ipc::v1::kMagicGipc ||
        hello.ver_major != gradient::ipc::v1::kVerMajor ||
        hello.ver_minor != gradient::ipc::v1::kVerMinor ||
        hello.bytes != sizeof(hello) ||
        (!adapter_connection && !legacy_oracle_role) ||
        std::any_of(std::begin(hello.reserved), std::end(hello.reserved), [](uint64_t v) { return v != 0; })) {
      logf("ERROR: HELLO validation failed (magic/ver/bytes/role mismatch)");
      close(client_fd);
      return AcceptResult::Retry;
    }

    // Bind adapter identity to kernel credentials; only the configured account
    // extends the daemon UID rule, and legacy oracle access remains unchanged.
    ucred peer{};
    socklen_t peer_bytes = sizeof(peer);
    if (getsockopt(client_fd, SOL_SOCKET, SO_PEERCRED, &peer, &peer_bytes) != 0 ||
        peer_bytes != sizeof(peer) ||
        rosie::rtcore::admit_peer(hello.role, hello.pid, peer, geteuid(),
                                  adapter_uid, legacy_oracle_role) !=
            rosie::rtcore::PeerAdmission::Admitted ||
        next_connection_id == std::numeric_limits<uint64_t>::max()) {
      logf("ERROR: native adapter peer credentials/incarnation rejected");
      close(client_fd);
      return AcceptResult::Retry;
    }

    // Connection accepted.
    controlling_client_fd = client_fd;
    ++next_connection_id;
    if (!publish_connection(adapter_connection)) {
      close(controlling_client_fd);
      controlling_client_fd = -1;
      return AcceptResult::Retry;
    }
    logf("Controller connected (pid=%llu)", static_cast<unsigned long long>(hello.pid));

    return AcceptResult::Connected;
  }

  // Publish fixed shared-memory geometry and send WELCOME with borrowed FDs.
  // On failure the caller must reset the connection before accepting a successor.
  bool initialize(const Options& opt, const std::array<uint8_t, 16>& daemon_incarnation,
                  uint32_t configured_mask,
                  rosie::rtcore::RtMappingHandoff<rosie::rtcore::FastControlRegion>& fast_handoff) {
    // Create shared memory regions (memfd) and eventfds.

    const size_t cmd_ring_hdr_aligned = align_up(sizeof(gradient::ipc::v1::RingHeaderV1), 8);
    const size_t cmd_ring_bytes =
        cmd_ring_hdr_aligned +
        static_cast<size_t>(gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY) *
            gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    const size_t cmd_shm_bytes =
        align_up(sizeof(gradient::ipc::v1::ShmHeaderV1) + cmd_ring_bytes, 4096);

    const size_t status_ring_hdr_aligned = align_up(sizeof(gradient::ipc::v1::RingHeaderV1), 8);
    const size_t status_ring_bytes =
        status_ring_hdr_aligned +
        static_cast<size_t>(gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY) *
            gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    const size_t status_shm_bytes =
        align_up(sizeof(gradient::ipc::v1::ShmHeaderV1) + status_ring_bytes, 4096);

    cmd_shm = create_memfd_region("gradient_cmd_shm", cmd_shm_bytes);
    status_shm = create_memfd_region("gradient_status_shm", status_shm_bytes);
    if (cmd_shm.fd < 0 || status_shm.fd < 0) {
      return false;
    }

    if (adapter_connection) {
      fast_shm = create_memfd_region("rosie_fast_control", rosie::rtcore::fast_control_region_bytes);
      rosie::protocol::FastHeader header{};
      header.magic = rosie::rtcore::fast_control_magic;
      header.version_major = rosie::protocol::version_major;
      header.version_minor = rosie::protocol::version_minor;
      header.region_bytes = rosie::rtcore::fast_control_region_bytes;
      header.daemon_incarnation = daemon_incarnation;
      header.connection_id = next_connection_id;
      header.cycle_ns = opt.cycle_ns;
      header.configured_mask = configured_mask;
      bool digest_valid = opt.configuration_hash.size() == 64;
      auto hex_digit = [](char c) -> int {
        if (c >= '0' && c <= '9') return c - '0';
        if (c >= 'a' && c <= 'f') return c - 'a' + 10;
        return -1;
      };
      for (size_t i = 0; digest_valid && i < 32; ++i) {
        const int hi = hex_digit(opt.configuration_hash[2 * i]);
        const int lo = hex_digit(opt.configuration_hash[2 * i + 1]);
        if (hi < 0 || lo < 0) { digest_valid = false; break; }
        (i < 16 ? header.configuration_digest_lo[i] : header.configuration_digest_hi[i - 16]) =
            static_cast<uint8_t>(hi * 16 + lo);
      }
      fast_region = std::make_unique<rosie::rtcore::FastControlRegion>();
      if (fast_shm.fd < 0 || !digest_valid ||
          rosie::rtcore::FastControlRegion::initialize(fast_shm.base, fast_shm.bytes, header, *fast_region) !=
              rosie::rtcore::FastControlStatus::Ok || !fast_handoff.publish(fast_region.get())) {
        logf("ERROR: fast-control mapping initialization/publication failed");
        return false;
      }
      fast_published = true;
    }

    cmd_eventfd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    status_eventfd = eventfd(0, EFD_CLOEXEC | EFD_NONBLOCK);
    if (cmd_eventfd < 0 || status_eventfd < 0) {
      logf("ERROR: eventfd() failed: %s", std::strerror(errno));
      return false;
    }

    // Initialize cmd_shm.
    {
      std::memset(cmd_shm.base, 0, cmd_shm.bytes);
      auto* hdr = reinterpret_cast<gradient::ipc::v1::ShmHeaderV1*>(cmd_shm.base);
      hdr->magic = gradient::ipc::v1::kMagicGshm;
      hdr->ver_major = gradient::ipc::v1::kVerMajor;
      hdr->ver_minor = gradient::ipc::v1::kVerMinor;
      hdr->bytes = sizeof(*hdr);
      hdr->kind = gradient::ipc::v1::kShmKindCmd;
      hdr->num_axes = opt.num_axes;
      hdr->cycle_ns = opt.cycle_ns;
      hdr->topology_hash = topology_hash;
      hdr->ring_offset = sizeof(*hdr);
      hdr->ring_capacity = gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY;
      hdr->ring_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
      hdr->setpoint_offset = 0;

      auto ring = make_ring_view(cmd_shm.base, hdr);
      ring.header->magic = gradient::ipc::v1::kMagicRing;
      ring.header->capacity = hdr->ring_capacity;
      ring.header->msg_bytes = hdr->ring_msg_bytes;
      ring.header->write_idx = 0;
      ring.header->read_idx = 0;
      ring.header->dropped = 0;

    }

    // Initialize status_shm.
    {
      std::memset(status_shm.base, 0, status_shm.bytes);
      auto* hdr = reinterpret_cast<gradient::ipc::v1::ShmHeaderV1*>(status_shm.base);
      hdr->magic = gradient::ipc::v1::kMagicGshm;
      hdr->ver_major = gradient::ipc::v1::kVerMajor;
      hdr->ver_minor = gradient::ipc::v1::kVerMinor;
      hdr->bytes = sizeof(*hdr);
      hdr->kind = gradient::ipc::v1::kShmKindStatus;
      hdr->num_axes = opt.num_axes;
      hdr->cycle_ns = opt.cycle_ns;
      hdr->topology_hash = topology_hash;
      hdr->ring_offset = sizeof(*hdr);
      hdr->ring_capacity = gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY;
      hdr->ring_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
      hdr->setpoint_offset = 0;

      auto ring = make_ring_view(status_shm.base, hdr);
      ring.header->magic = gradient::ipc::v1::kMagicRing;
      ring.header->capacity = hdr->ring_capacity;
      ring.header->msg_bytes = hdr->ring_msg_bytes;
      ring.header->write_idx = 0;
      ring.header->read_idx = 0;
      ring.header->dropped = 0;
    }

    // Send WELCOME + SCM_RIGHTS fds.
    gradient::ipc::v1::WelcomeV1 welcome{};
    welcome.magic = gradient::ipc::v1::kMagicGipc;
    welcome.ver_major = gradient::ipc::v1::kVerMajor;
    welcome.ver_minor = gradient::ipc::v1::kVerMinor;
    welcome.bytes = sizeof(welcome);
    welcome.num_axes = opt.num_axes;
    welcome.cycle_ns = opt.cycle_ns;
    welcome.topology_hash = topology_hash;
    welcome.cmd_ring_capacity = gradient::ipc::v1::GRADIENT_CMD_RING_CAPACITY;
    welcome.cmd_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    welcome.status_ring_capacity = gradient::ipc::v1::GRADIENT_STATUS_RING_CAPACITY;
    welcome.status_msg_bytes = gradient::ipc::v1::GRADIENT_RING_MSG_BYTES;
    welcome.build_id_hash = build_id_hash;
    // Bind both native roles to the ring's daemon identity before descriptor
    // handoff; observation must not depend on an application grant mapping.
    std::memcpy(welcome.daemon_incarnation, daemon_incarnation.data(), daemon_incarnation.size());

    int fds[5] = {cmd_shm.fd, status_shm.fd, cmd_eventfd, status_eventfd, fast_shm.fd};
    const size_t fd_bytes = (adapter_connection ? 5 : 4) * sizeof(int);
    char cmsg_buf[CMSG_SPACE(sizeof(fds))];
    std::memset(cmsg_buf, 0, sizeof(cmsg_buf));

    iovec iov{};
    iov.iov_base = &welcome;
    iov.iov_len = sizeof(welcome);

    msghdr msg{};
    msg.msg_iov = &iov;
    msg.msg_iovlen = 1;
    msg.msg_control = cmsg_buf;
    msg.msg_controllen = CMSG_SPACE(fd_bytes);

    cmsghdr* cmsg = CMSG_FIRSTHDR(&msg);
    cmsg->cmsg_level = SOL_SOCKET;
    cmsg->cmsg_type = SCM_RIGHTS;
    cmsg->cmsg_len = CMSG_LEN(fd_bytes);
    std::memcpy(CMSG_DATA(cmsg), fds, fd_bytes);

    if (sendmsg(controlling_client_fd, &msg, MSG_NOSIGNAL) != static_cast<ssize_t>(sizeof(welcome))) {
      logf("ERROR: sendmsg(WELCOME) failed: %s", std::strerror(errno));
      return false;
    }

    return true;
  }
};
