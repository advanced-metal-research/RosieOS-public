// Private implementation header included once by robot_v4_cartesian_daemon.cpp.
bool daemon_identity_is_semantic_role(const std::string& value);

bool parse_http_url(const std::string& raw_url, HostPort* host_port, std::string* path) {
  constexpr const char* prefix = "http://";
  if (raw_url.compare(0, std::strlen(prefix), prefix) != 0) {
    return false;
  }
  const std::string rest = raw_url.substr(std::strlen(prefix));
  const std::string::size_type authority_end = rest.find_first_of("/?#");
  const std::string authority =
      authority_end == std::string::npos ? rest : rest.substr(0, authority_end);
  if (authority.empty() || authority.find('@') != std::string::npos) {
    return false;
  }
  if (path != nullptr) {
    if (authority_end == std::string::npos) {
      *path = "/";
    } else if (rest[authority_end] == '/') {
      *path = rest.substr(authority_end);
    } else {
      *path = "/" + rest.substr(authority_end);
    }
  }
  return network_authority_host_port(authority, false, "80", host_port) &&
         !daemon_identity_is_semantic_role(host_port->host);
}

bool http_get_text(const std::string& url, std::string* out) {
  HostPort target;
  std::string path;
  if (!parse_http_url(url, &target, &path)) {
    std::cerr << "robot-v4-cartesiand: daemon state URL must be http://, got " << url << "\n";
    return false;
  }
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  addrinfo* results = nullptr;
  const int gai = ::getaddrinfo(target.host.c_str(), target.port.c_str(), &hints, &results);
  if (gai != 0 || results == nullptr) {
    std::cerr << "robot-v4-cartesiand: resolve daemon state " << url << ": "
              << (gai == 0 ? "no results" : gai_strerror(gai)) << "\n";
    return false;
  }
  int fd = -1;
  for (addrinfo* item = results; item != nullptr; item = item->ai_next) {
    fd = ::socket(item->ai_family, item->ai_socktype, item->ai_protocol);
    if (fd < 0) {
      continue;
    }
    // daemon/v1 may keep an HTTP/1.1 connection alive even when the request
    // asks it to close.  Every cold-path NATS publication resolves through
    // this endpoint, so an unbounded recv here leaks one detached publisher
    // thread and socket per plan.  Bound both directions before connect and
    // finish as soon as Content-Length bytes have arrived.
    timeval timeout{};
    timeout.tv_sec = 1;
    timeout.tv_usec = 0;
    (void)::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &timeout,
                       sizeof(timeout));
    (void)::setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &timeout,
                       sizeof(timeout));
#ifdef SO_NOSIGPIPE
    int no_sigpipe = 1;
    (void)::setsockopt(fd, SOL_SOCKET, SO_NOSIGPIPE, &no_sigpipe,
                       sizeof(no_sigpipe));
#endif
    if (::connect(fd, item->ai_addr, item->ai_addrlen) == 0) {
      break;
    }
    ::close(fd);
    fd = -1;
  }
  ::freeaddrinfo(results);
  if (fd < 0) {
    std::cerr << "robot-v4-cartesiand: connect daemon state " << url << " failed\n";
    return false;
  }
  std::ostringstream request;
  request << "GET " << path << " HTTP/1.1\r\n"
          << "Host: " << target.host << "\r\n"
          << "Connection: close\r\n\r\n";
  const std::string request_text = request.str();
  if (!send_all(fd, request_text)) {
    ::close(fd);
    return false;
  }
  std::string response;
  constexpr std::size_t kMaxDaemonStateResponseBytes = 16u * 1024u * 1024u;
  std::size_t expected_response_bytes = std::string::npos;
  char buffer[4096];
  while (true) {
    const ssize_t got = ::recv(fd, buffer, sizeof(buffer), 0);
    if (got < 0 && errno == EINTR) {
      continue;
    }
    if (got <= 0) {
      break;
    }
    response.append(buffer, static_cast<std::size_t>(got));
    if (response.size() > kMaxDaemonStateResponseBytes) {
      response.clear();
      break;
    }
    if (expected_response_bytes == std::string::npos) {
      const std::string::size_type header_end = response.find("\r\n\r\n");
      if (header_end != std::string::npos) {
        std::string headers = response.substr(0, header_end + 2);
        std::transform(headers.begin(), headers.end(), headers.begin(),
                       [](unsigned char ch) {
                         return static_cast<char>(std::tolower(ch));
                       });
        const std::string content_length_header = "\r\ncontent-length:";
        const std::string::size_type length_at =
            headers.find(content_length_header);
        if (length_at != std::string::npos) {
          const char* length_text =
              headers.c_str() + length_at + content_length_header.size();
          char* length_end = nullptr;
          errno = 0;
          const unsigned long long content_length =
              std::strtoull(length_text, &length_end, 10);
          if (errno == 0 && length_end != length_text &&
              content_length <= kMaxDaemonStateResponseBytes) {
            expected_response_bytes =
                header_end + 4 + static_cast<std::size_t>(content_length);
          }
        }
      }
    }
    if (expected_response_bytes != std::string::npos &&
        response.size() >= expected_response_bytes) {
      response.resize(expected_response_bytes);
      break;
    }
  }
  ::close(fd);
  const std::string::size_type header_end = response.find("\r\n\r\n");
  if (header_end == std::string::npos ||
      response.find(" 200 ") == std::string::npos ||
      (expected_response_bytes != std::string::npos &&
       response.size() < expected_response_bytes)) {
    std::cerr << "robot-v4-cartesiand: daemon state " << url << " returned non-200 response\n";
    return false;
  }
  *out = response.substr(header_end + 4);
  return true;
}

bool daemon_identity_is_semantic_role(const std::string& value) {
  const std::string host = transport_identity_host_token(value);
  return host == "manifest" ||
         host == "service" ||
         host == "motion-server" ||
         host == "realtime-core" ||
         host == "steamdeck-teach-pendent" ||
         host == "build-server" ||
         host == "motion-authority" ||
         host == "servo-authority" ||
         host == "human-client-ui" ||
         host == "os-arch-build-server" ||
         host == "builder-dashboard-artifact-node" ||
         host.rfind("robot-v4.", 0) == 0;
}

bool daemon_peer_is_plain_concrete(const std::string& raw_peer) {
  const std::string peer = transport_identity_trim_copy(raw_peer);
  if (peer.empty() || peer.front() == '-' || peer.find("://") != std::string::npos ||
      peer.find_first_of("@/?#=,\\\"' \t\r\n") != std::string::npos) {
    return false;
  }
  if (peer.find(':') == std::string::npos) {
    return true;
  }
  std::string candidate = peer;
  if (candidate.size() >= 2 && candidate.front() == '[' && candidate.back() == ']') {
    candidate = candidate.substr(1, candidate.size() - 2);
  }
  const std::size_t zone = candidate.rfind('%');
  if (zone != std::string::npos) {
    candidate = candidate.substr(0, zone);
  }
  in6_addr address{};
  return ::inet_pton(AF_INET6, candidate.c_str(), &address) == 1;
}

bool daemon_service_ref_parts(const std::string& ref,
                              std::string* peer,
                              std::string* role,
                              std::string* service) {
  constexpr const char* prefix = "daemon-v1://";
  if (ref.compare(0, std::strlen(prefix), prefix) != 0) {
    return false;
  }
  const std::string rest = ref.substr(std::strlen(prefix));
  const std::string::size_type first_split = rest.find('/');
  if (first_split == std::string::npos) {
    return false;
  }
  const std::string::size_type second_split = rest.find('/', first_split + 1);
  if (second_split == std::string::npos ||
      rest.find('/', second_split + 1) != std::string::npos) {
    return false;
  }
  *role = rest.substr(first_split + 1, second_split - first_split - 1);
  *peer = rest.substr(0, first_split);
  *service = rest.substr(second_split + 1);
  std::string peer_lower = *peer;
  std::transform(peer_lower.begin(), peer_lower.end(), peer_lower.begin(), [](unsigned char ch) {
    return static_cast<char>(std::tolower(ch));
  });
  const bool source_decorated = peer_lower.rfind("local:", 0) == 0 ||
                                peer_lower.rfind("iroh-gossip:", 0) == 0;
  return !peer->empty() &&
         !role->empty() &&
         *role != "service" &&
         !source_decorated &&
	     daemon_peer_is_plain_concrete(*peer) &&
         !daemon_identity_is_semantic_role(*peer) &&
         *peer != *role &&
         !service->empty();
}

std::string first_usable_daemon_advertised_address(const std::vector<std::string>& addresses) {
  for (const std::string& address : addresses) {
    std::string host;
    if (daemon_peer_is_plain_concrete(address)) {
      host = transport_identity_host_token(address);
    } else {
      HostPort parsed;
      if (address.find('@') == std::string::npos &&
          network_authority_host_port(address, true, "", &parsed) &&
          daemon_peer_is_plain_concrete(parsed.host)) {
        host = transport_identity_host_token(parsed.host);
      }
    }
    if (!host.empty() &&
        !daemon_identity_is_semantic_role(host) &&
        host.rfind("127.", 0) != 0 &&
        host.rfind("0.0.0.0", 0) != 0 &&
        host.rfind("169.254.", 0) != 0 &&
        host != "localhost" &&
        host != "iroh-gossip") {
      return host;
    }
  }
  return "";
}

bool daemon_service_host_usable(const std::string& host) {
  const std::string normalized = transport_identity_host_token(host);
  return !normalized.empty() &&
         daemon_peer_is_plain_concrete(host) &&
         !daemon_identity_is_semantic_role(normalized) &&
         normalized.rfind("127.", 0) != 0 &&
         normalized.rfind("0.0.0.0", 0) != 0 &&
         normalized.rfind("169.254.", 0) != 0 &&
         normalized != "::" &&
         normalized != "::1" &&
         normalized != "localhost" &&
         normalized != "iroh-gossip";
}

std::string daemon_node_service_host(const std::string& node_object) {
  // Prefer the address observed by this daemon.  In particular, a remote
  // link-local IPv6 advertisement carries the remote host's interface name
  // (for example `%eth1`), which is not a usable scope on this machine.  The
  // source address carries this machine's numeric scope (for example `%4`).
  std::string source_addr;
  if (parse_json_string(node_object, "source_addr", &source_addr) &&
      !source_addr.empty() &&
      source_addr != "local" &&
      source_addr.rfind("iroh-gossip:", 0) != 0) {
    if (source_addr.rfind("local:", 0) == 0) {
      source_addr = source_addr.substr(std::strlen("local:"));
    }
    const std::string observed_host = transport_identity_host_token(source_addr);
    std::string observed_network_address = observed_host;
    const std::size_t zone = observed_network_address.rfind('%');
    if (zone != std::string::npos) {
      observed_network_address = observed_network_address.substr(0, zone);
    }
    in_addr observed_ipv4{};
    in6_addr observed_ipv6{};
    const bool observed_host_is_ip =
        ::inet_pton(AF_INET, observed_network_address.c_str(), &observed_ipv4) == 1 ||
        ::inet_pton(AF_INET6, observed_network_address.c_str(), &observed_ipv6) == 1;
    if (observed_host_is_ip && daemon_service_host_usable(observed_host)) {
      return observed_host;
    }
  }
  const std::string advertised =
      first_usable_daemon_advertised_address(parse_json_string_array(node_object, "advertised_addresses"));
  if (!advertised.empty()) {
    return advertised;
  }
  return "";
}

bool daemon_node_source_is_local(const std::string& node_object) {
  std::string source_addr;
  return parse_json_string(node_object, "source_addr", &source_addr) &&
         (source_addr == "local" ||
          source_addr.rfind("local:", 0) == 0 ||
          source_addr.rfind("local-", 0) == 0);
}

bool daemon_state_local_node_matches_peer(const std::string& state,
                                          const std::string& wanted_peer) {
  // A peer heartbeat can contain that peer's own `source_addr: local`
  // projection.  When the heartbeat is merged into another daemon's state,
  // that projection is still remote and must not resolve to this process's
  // loopback interface.  Only the daemon serving this /api/state response may
  // use the UDP loopback shortcut.
  const std::string node_marker = "\"nodes\"";
  const std::string::size_type nodes_at = state.find(node_marker);
  if (nodes_at == std::string::npos) {
    return false;
  }
  std::string local_node;
  if (!parse_json_string(state.substr(0, nodes_at), "node", &local_node)) {
    return false;
  }
  return transport_identity_host_token(local_node) ==
         transport_identity_host_token(wanted_peer);
}

bool daemon_node_matches_service_ref(const std::string& node_object,
                                     const std::string& wanted_peer,
                                     const std::string& wanted_role) {
  std::string peer_name;
  std::string role_name;
  return parse_json_string(node_object, "peer", &peer_name) &&
         parse_json_string(node_object, "role", &role_name) &&
         transport_identity_host_token(peer_name) ==
             transport_identity_host_token(wanted_peer) &&
         role_name == wanted_role &&
         daemon_peer_is_plain_concrete(peer_name) &&
         !daemon_identity_is_semantic_role(peer_name) &&
         daemon_identity_is_semantic_role(role_name);
}

bool daemon_service_endpoint_matches_ref(const std::string& service_object,
                                         const std::string& ref,
                                         const std::string& wanted_peer,
                                         const std::string& wanted_service) {
  std::string service_name;
  if (!parse_json_string(service_object, "name", &service_name) ||
      service_name != wanted_service) {
    return false;
  }
  std::string advertised_by;
  if (parse_json_string(service_object, "advertised_by", &advertised_by) &&
      !advertised_by.empty() &&
      (!daemon_peer_is_plain_concrete(advertised_by) ||
       transport_identity_host_token(advertised_by) !=
           transport_identity_host_token(wanted_peer))) {
    return false;
  }
  std::string service_url;
  if (!parse_json_string(service_object, "url", &service_url) || service_url.empty()) {
    return true;
  }
  std::string endpoint_peer, endpoint_role, endpoint_service;
  std::string ref_peer, ref_role, ref_service;
  // Peer spelling follows node identity normalization; role and service are
  // exact contract names, not case-insensitive host identities.
  return daemon_service_ref_parts(service_url, &endpoint_peer, &endpoint_role,
                                  &endpoint_service) &&
         daemon_service_ref_parts(ref, &ref_peer, &ref_role, &ref_service) &&
         transport_identity_host_token(endpoint_peer) ==
             transport_identity_host_token(ref_peer) &&
         endpoint_role == ref_role && endpoint_service == ref_service;
}

bool direct_network_target_has_concrete_host(const std::string& target,
                                             const std::string& scheme_prefix,
                                             bool allow_zero_port = false) {
  std::string authority = target;
  if (!scheme_prefix.empty()) {
    if (authority.rfind(scheme_prefix, 0) != 0) {
      return false;
    }
    authority = authority.substr(scheme_prefix.size());
  }
  const std::size_t path = authority.find('/');
  if (path != std::string::npos) {
    authority = authority.substr(0, path);
  }
  HostPort parsed;
  if (!network_authority_host_port(authority, true, "", &parsed,
                                   allow_zero_port)) {
    return false;
  }
  const std::string host = transport_identity_host_token(parsed.host);
  return !host.empty() &&
         daemon_peer_is_plain_concrete(parsed.host) &&
         !daemon_identity_is_semantic_role(host) &&
         host != "iroh-gossip";
}

bool resolve_daemon_udp_service_ref(const std::string& ref,
                                    const std::string& daemon_state_url,
                                    std::string* target) {
  std::string wanted_peer;
  std::string wanted_role;
  std::string wanted_service;
  if (!daemon_service_ref_parts(ref, &wanted_peer, &wanted_role, &wanted_service)) {
    if (ref == "udp-source") {
      *target = ref;
      return true;
    }
    if (ref.rfind("daemon-v1://", 0) == 0 ||
        !direct_network_target_has_concrete_host(ref, "")) {
      return false;
    }
    *target = ref;
    return true;
  }
  std::string state;
  if (!http_get_text(daemon_state_url, &state)) {
    return false;
  }
  if (daemon_state_local_node_matches_peer(state, wanted_peer)) {
    for (const std::string& node_object :
         json_object_slices_for_array_key(state, "nodes")) {
      if (!daemon_node_matches_service_ref(node_object, wanted_peer,
                                           wanted_role) ||
          !daemon_node_source_is_local(node_object)) {
        continue;
      }
      for (const std::string& service_object :
           json_object_slices_for_array_key(node_object,
                                            "service_endpoints")) {
        std::string protocol;
        std::string listen;
        if (!daemon_service_endpoint_matches_ref(
                service_object, ref, wanted_peer, wanted_service) ||
            !parse_json_string(service_object, "listen", &listen)) {
          continue;
        }
        (void)parse_json_string(service_object, "protocol", &protocol);
        if (!protocol.empty() && protocol != "udp") {
          continue;
        }
        HostPort listen_host_port;
        if (!split_host_port(listen, &listen_host_port)) {
          continue;
        }
        *target = "127.0.0.1:" + listen_host_port.port;
        return true;
      }
    }
  }
  for (const std::string& node_object : json_object_slices_for_array_key(state, "nodes")) {
    if (!daemon_node_matches_service_ref(node_object, wanted_peer, wanted_role)) {
      continue;
    }
    const std::string host = daemon_node_service_host(node_object);
    if (host.empty()) {
      continue;
    }
    for (const std::string& service_object :
         json_object_slices_for_array_key(node_object, "service_endpoints")) {
      std::string protocol;
      std::string listen;
      if (!daemon_service_endpoint_matches_ref(service_object, ref, wanted_peer, wanted_service) ||
          !parse_json_string(service_object, "listen", &listen)) {
        continue;
      }
      (void)parse_json_string(service_object, "protocol", &protocol);
      if (!protocol.empty() && protocol != "udp") {
        continue;
      }
      HostPort listen_host_port;
      if (!split_host_port(listen, &listen_host_port)) {
        continue;
      }
      const std::string target_host =
          host.find(':') == std::string::npos ? host : "[" + host + "]";
      *target = target_host + ":" + listen_host_port.port;
      return true;
    }
  }
  return false;
}

bool resolve_daemon_nats_service_ref_uncached(const std::string& ref,
                                     const std::string& daemon_state_url,
                                     std::string* target) {
  std::string wanted_peer;
  std::string wanted_role;
  std::string wanted_service;
  if (!daemon_service_ref_parts(ref, &wanted_peer, &wanted_role, &wanted_service)) {
    if (ref.rfind("daemon-v1://", 0) == 0 ||
        !direct_network_target_has_concrete_host(ref, "nats://")) {
      return false;
    }
    *target = ref;
    return true;
  }
  std::string state;
  // Service resolution only needs nodes. The full state includes unbounded
  // catalog payloads and can block synchronous lease acknowledgements.
  std::string discovery_url = daemon_state_url;
  const std::string state_suffix = "/api/state";
  if (discovery_url.size() >= state_suffix.size() &&
      discovery_url.compare(discovery_url.size() - state_suffix.size(),
                            state_suffix.size(), state_suffix) == 0) {
    discovery_url.replace(discovery_url.size() - state_suffix.size(),
                          state_suffix.size(), "/nodes");
  }
  if (!http_get_text(discovery_url, &state)) {
    return false;
  }
  for (const std::string& node_object : json_object_slices_for_array_key(state, "nodes")) {
    const bool role_projection =
        daemon_node_matches_service_ref(node_object, wanted_peer, wanted_role);
    std::string raw_peer, raw_role;
    // /nodes returns physical-host rows, whereas /api/state also expands
    // role aliases. Accept a raw host only with an exact provider-owned
    // service URL below; never infer a role from a service name alone.
    const bool physical_host =
        parse_json_string(node_object, "peer", &raw_peer) &&
        parse_json_string(node_object, "role", &raw_role) &&
        transport_identity_host_token(raw_peer) ==
            transport_identity_host_token(wanted_peer) &&
        daemon_peer_is_plain_concrete(raw_role) &&
        transport_identity_host_token(raw_role) ==
            transport_identity_host_token(raw_peer) &&
        daemon_peer_is_plain_concrete(raw_peer) &&
        !daemon_identity_is_semantic_role(raw_peer);
    if (!role_projection && !physical_host) {
      continue;
    }
    const std::string host = daemon_node_service_host(node_object);
    if (host.empty()) {
      return false;
    }
    for (const std::string& service_object :
         json_object_slices_for_array_key(node_object, "service_endpoints")) {
      if (!role_projection) {
        // Physical rows require explicit ownership; the shared matcher below
        // compares peer identities while retaining exact URL role/service.
        std::string service_url, provider;
        if (!parse_json_string(service_object, "url", &service_url) ||
            service_url.empty() ||
            !parse_json_string(service_object, "advertised_by", &provider) ||
            provider.empty()) {
          continue;
        }
      }
      std::string protocol;
      std::string listen;
      if (!daemon_service_endpoint_matches_ref(service_object, ref, wanted_peer, wanted_service) ||
          !parse_json_string(service_object, "listen", &listen)) {
        continue;
      }
      (void)parse_json_string(service_object, "protocol", &protocol);
      if (!protocol.empty() && protocol != "nats") {
        continue;
      }
      HostPort listen_host_port;
      if (!split_host_port(listen, &listen_host_port)) {
        continue;
      }
      const std::string url_host =
          host.find(':') == std::string::npos ? host : "[" + host + "]";
      *target = "nats://" + url_host + ":" + listen_host_port.port;
      return true;
    }
  }
  return false;
}

bool resolve_daemon_nats_service_ref(const std::string& ref,
                                     const std::string& daemon_state_url,
                                     std::string* target) {
  if (ref.rfind("daemon-v1://", 0) != 0) {
    return resolve_daemon_nats_service_ref_uncached(ref, daemon_state_url, target);
  }
  struct Entry {
    std::string url;
    std::chrono::steady_clock::time_point resolved_at{};
    bool refreshing = false;
  };
  struct Cache { std::mutex mutex; std::map<std::string, Entry> entries; };
  static const auto cache = std::make_shared<Cache>();
  const std::string key = ref + "\n" + daemon_state_url;
  const auto now = std::chrono::steady_clock::now();
  bool refresh = false;
  bool cached = false;
  {
    std::lock_guard<std::mutex> lock(cache->mutex);
    auto& entry = cache->entries[key];
    if (!entry.url.empty()) {
      const auto age = now - entry.resolved_at;
      if (age < std::chrono::seconds(15)) {
        *target = entry.url;
        cached = true;
      }
      if (age >= std::chrono::seconds(5) && !entry.refreshing) {
        entry.refreshing = true;
        refresh = true;
      }
      if (!refresh) return cached;
    } else if (entry.refreshing) {
      return false;
    } else {
      entry.refreshing = true;
    }
  }
  const auto update = [shared = cache, key, ref, daemon_state_url]() {
    std::string resolved;
    const bool ok = resolve_daemon_nats_service_ref_uncached(ref, daemon_state_url, &resolved);
    std::lock_guard<std::mutex> lock(shared->mutex);
    auto& entry = shared->entries[key];
    if (ok) {
      entry.url = resolved;
      entry.resolved_at = std::chrono::steady_clock::now();
    }
    entry.refreshing = false;
    return ok;
  };
  if (refresh) {
    // Never let discovery HTTP delay active lease/status handling. Expired
    // routing fails closed after 15 s; RTCore authority still expires normally.
    std::thread([update]() { (void)update(); }).detach();
    return cached;
  }
  // First resolution occurs at startup. No prior route is invented.
  if (!update()) return false;
  std::lock_guard<std::mutex> lock(cache->mutex);
  *target = cache->entries[key].url;
  return !target->empty();
}

bool normalize_xyz(double values[3]) {
  const double length = std::sqrt(values[0] * values[0] + values[1] * values[1] +
                                  values[2] * values[2]);
  if (!std::isfinite(length) || length <= 1.0e-9) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    values[axis] /= length;
  }
  return true;
}

bool parse_xyz_field(const std::string& raw, const std::string& key, double values[3]) {
  const std::vector<double> parsed = parse_json_number_array(raw, key);
  if (parsed.size() < 3) {
    return false;
  }
  for (int axis = 0; axis < 3; ++axis) {
    values[axis] = parsed[static_cast<std::size_t>(axis)];
  }
  return normalize_xyz(values);
}

SpreadsheetToolAxisConstraint parse_spreadsheet_tool_axis_constraint(const std::string& raw) {
  SpreadsheetToolAxisConstraint constraint;
  constraint.present = raw.find("\"tool_axis_constraint\"") != std::string::npos;
  if (!constraint.present) {
    return constraint;
  }
  (void)parse_json_string(raw, "tool_frame", &constraint.tool_frame);
  (void)parse_json_string(raw, "tip_link", &constraint.tip_link);
  (void)parse_json_string(raw, "world_frame", &constraint.world_frame);
  const bool tool_axis_valid = parse_xyz_field(raw, "tool_axis_xyz", constraint.tool_axis_xyz);
  const bool world_axis_valid = parse_xyz_field(raw, "world_axis_xyz", constraint.world_axis_xyz);
  if (!parse_xyz_field(raw, "offset_reference_axis_xyz", constraint.offset_reference_axis_xyz)) {
    constraint.offset_reference_axis_xyz[0] = 1.0;
    constraint.offset_reference_axis_xyz[1] = 0.0;
    constraint.offset_reference_axis_xyz[2] = 0.0;
  }
  (void)parse_json_double(raw, "offset_deg", &constraint.offset_deg);
  double legacy_tolerance_deg = constraint.tolerance_plus_deg;
  if (parse_json_double(raw, "angle_tolerance_deg", &legacy_tolerance_deg)) {
    constraint.tolerance_minus_deg = legacy_tolerance_deg;
    constraint.tolerance_plus_deg = legacy_tolerance_deg;
  }
  (void)parse_json_double(raw, "tolerance_minus_deg", &constraint.tolerance_minus_deg);
  (void)parse_json_double(raw, "tolerance_plus_deg", &constraint.tolerance_plus_deg);
  (void)parse_json_double(raw, "weight", &constraint.weight);
  if (!tool_axis_valid || !world_axis_valid) {
    constraint.validation_error = "axis_vector_invalid";
  } else if (constraint.tool_frame.empty() || constraint.tip_link.empty() ||
             constraint.world_frame.empty()) {
    constraint.validation_error = "frame_name_missing";
  } else if (constraint.tool_frame != "end_effector" || constraint.tip_link != "tool0" ||
             constraint.world_frame != "world") {
    // The current FK terminates at the URDF tool0/end_effector transform.
    // Fail closed until arbitrary frame-chain resolution is implemented.
    constraint.validation_error = "unsupported_frame";
  } else if (!std::isfinite(constraint.offset_deg) || constraint.offset_deg < -180.0 ||
             constraint.offset_deg > 180.0) {
    constraint.validation_error = "offset_out_of_range";
  } else if (!std::isfinite(constraint.tolerance_minus_deg) ||
             !std::isfinite(constraint.tolerance_plus_deg) ||
             constraint.tolerance_minus_deg < 0.0 || constraint.tolerance_minus_deg > 180.0 ||
             constraint.tolerance_plus_deg < 0.0 || constraint.tolerance_plus_deg > 180.0) {
    constraint.validation_error = "tolerance_out_of_range";
  } else if (!std::isfinite(constraint.weight) || constraint.weight <= 0.0 ||
             constraint.weight > 10.0) {
    constraint.validation_error = "weight_out_of_range";
  } else {
    constraint.valid = true;
  }
  return constraint;
}

SpreadsheetHarmoniousAxisPolicy parse_spreadsheet_harmonious_axis_policy(
    const std::string& raw) {
  SpreadsheetHarmoniousAxisPolicy policy;
  // Legacy requests carried only joint/center/amplitude/phase metadata. That
  // is not enough to identify a safe fixture side or workplane, so it must not
  // accidentally activate (and then fail) the modern coordinated-axis gate.
  policy.present = raw.find("\"harmonious_external_axis\"") != std::string::npos &&
                   raw.find("\"active_side\"") != std::string::npos &&
                   raw.find("\"selector_joint\"") != std::string::npos &&
                   raw.find("\"work_frame\"") != std::string::npos;
  if (!policy.present) {
    return policy;
  }
  (void)parse_json_string(raw, "joint", &policy.joint);
  (void)parse_json_string(raw, "active_side", &policy.active_side);
  (void)parse_json_string(raw, "selector_joint", &policy.selector_joint);
  (void)parse_json_double(raw, "selector_rad", &policy.selector_rad);
  (void)parse_json_double(raw, "selector_tolerance_rad", &policy.selector_tolerance_rad);
  (void)parse_json_double(raw, "center_rad", &policy.center_rad);
  (void)parse_json_double(raw, "amplitude_rad", &policy.amplitude_rad);
  (void)parse_json_double(raw, "phase_deg", &policy.phase_deg);
  (void)parse_json_string(raw, "trajectory_source", &policy.trajectory_source);
  (void)parse_json_string(raw, "work_frame", &policy.work_frame);
  (void)parse_json_bool(raw, "workplane_relative", &policy.workplane_relative);
  (void)parse_json_double(raw, "reference_rad", &policy.reference_rad);
  (void)parse_json_bool(raw, "tool_axis_follows_workplane",
                        &policy.tool_axis_follows_workplane);
  const std::vector<double> pivot = parse_json_number_array(raw, "workplane_pivot_m");
  if (pivot.size() >= 3) {
    for (int axis = 0; axis < 3; ++axis) {
      policy.workplane_pivot_m[axis] = pivot[static_cast<std::size_t>(axis)];
    }
  }
  const bool workplane_axis_valid =
      parse_xyz_field(raw, "workplane_axis_xyz", policy.workplane_axis_xyz);
  const bool side_a = policy.active_side == "A" && policy.joint == "J7" &&
                      policy.work_frame == "positioner_table_a" &&
                      std::abs(policy.selector_rad) <= policy.selector_tolerance_rad;
  const bool side_b = policy.active_side == "B" && policy.joint == "J8" &&
                      policy.work_frame == "positioner_table_b" &&
                      std::abs(std::abs(policy.selector_rad) - kPi) <=
                          policy.selector_tolerance_rad;
  if (policy.selector_joint != "J9") {
    policy.validation_error = "selector_must_be_J9";
  } else if (!std::isfinite(policy.selector_rad) ||
             !std::isfinite(policy.selector_tolerance_rad) ||
             policy.selector_tolerance_rad <= 0.0 ||
             policy.selector_tolerance_rad > 0.10) {
    policy.validation_error = "selector_contract_invalid";
  } else if (!side_a && !side_b) {
    policy.validation_error = "active_side_workplane_mismatch";
  } else if (!std::isfinite(policy.center_rad) || !std::isfinite(policy.amplitude_rad) ||
             !std::isfinite(policy.phase_deg) || policy.amplitude_rad < 0.0 ||
             policy.amplitude_rad > kPi || std::abs(policy.center_rad) + policy.amplitude_rad > kPi) {
    policy.validation_error = "harmonious_axis_range_invalid";
  } else if (policy.trajectory_source != "sinusoidal_progress" &&
             policy.trajectory_source != "section_sinusoidal" &&
             policy.trajectory_source != "row_targets") {
    policy.validation_error = "harmonious_trajectory_source_invalid";
  } else if (policy.workplane_relative &&
             ((!side_a && !side_b) || pivot.size() < 3 ||
              !workplane_axis_valid || !std::isfinite(policy.reference_rad) ||
              !std::isfinite(policy.workplane_pivot_m[0]) ||
              !std::isfinite(policy.workplane_pivot_m[1]) ||
              !std::isfinite(policy.workplane_pivot_m[2]))) {
    policy.validation_error = "workplane_contract_invalid";
  } else {
    policy.valid = true;
  }
  return policy;
}

double harmonious_axis_position_rad(const SpreadsheetHarmoniousAxisPolicy& policy,
                                    double normalized_progress) {
  const double phase_rad = policy.phase_deg * kPi / 180.0;
  const double cycle = 2.0 * kPi * std::clamp(normalized_progress, 0.0, 1.0) + phase_rad;
  return policy.center_rad + policy.amplitude_rad * std::sin(cycle);
}

std::array<double, 3> rotate_about_normalized_axis(const double values[3],
                                                   const double axis[3],
                                                   double radians) {
  const std::array<double, 3> v{{values[0], values[1], values[2]}};
  const std::array<double, 3> a{{axis[0], axis[1], axis[2]}};
  const double c = std::cos(radians);
  const double s = std::sin(radians);
  const double dot = dot3(a, v);
  const std::array<double, 3> cross = cross3(a, v);
  return {{v[0] * c + cross[0] * s + a[0] * dot * (1.0 - c),
           v[1] * c + cross[1] * s + a[1] * dot * (1.0 - c),
           v[2] * c + cross[2] * s + a[2] * dot * (1.0 - c)}};
}

void apply_harmonious_workplane_transform(const SpreadsheetHarmoniousAxisPolicy& policy,
                                          double positioner_rad,
                                          double tcp_pose[6]) {
  if (!policy.workplane_relative || tcp_pose == nullptr) {
    return;
  }
  const double relative[3] = {
      tcp_pose[0] - policy.workplane_pivot_m[0],
      tcp_pose[1] - policy.workplane_pivot_m[1],
      tcp_pose[2] - policy.workplane_pivot_m[2],
  };
  const std::array<double, 3> rotated = rotate_about_normalized_axis(
      relative, policy.workplane_axis_xyz, positioner_rad - policy.reference_rad);
  for (int axis = 0; axis < 3; ++axis) {
    tcp_pose[axis] = policy.workplane_pivot_m[axis] + rotated[static_cast<std::size_t>(axis)];
  }
}

SpreadsheetToolAxisConstraint harmonious_sample_tool_constraint(
    const SpreadsheetToolAxisConstraint& constraint,
    const SpreadsheetHarmoniousAxisPolicy& policy,
    double positioner_rad) {
  SpreadsheetToolAxisConstraint sample = constraint;
  if (!policy.workplane_relative || !policy.tool_axis_follows_workplane) {
    return sample;
  }
  const double delta = positioner_rad - policy.reference_rad;
  const std::array<double, 3> world = rotate_about_normalized_axis(
      constraint.world_axis_xyz, policy.workplane_axis_xyz, delta);
  const std::array<double, 3> reference = rotate_about_normalized_axis(
      constraint.offset_reference_axis_xyz, policy.workplane_axis_xyz, delta);
  for (int axis = 0; axis < 3; ++axis) {
    sample.world_axis_xyz[axis] = world[static_cast<std::size_t>(axis)];
    sample.offset_reference_axis_xyz[axis] = reference[static_cast<std::size_t>(axis)];
  }
  return sample;
}

void fill_default_spreadsheet_plan_rows(std::vector<SpreadsheetPlanRow>* rows) {
  if (rows == nullptr) {
    return;
  }
  rows->clear();
  constexpr double side = 0.5;
  constexpr double front = 0.5;
  const std::array<std::array<double, 6>, 4> tcp = {{
      {{front + side, 0.0, 0.0, 0.0, 0.0, 0.0}},
      {{front + side, side, 0.0, 0.0, 0.0, 0.0}},
      {{front, side, 0.0, 0.0, 0.0, 0.0}},
      {{front, 0.0, 0.0, 0.0, 0.0, 0.0}},
  }};
  for (int index = 0; index < static_cast<int>(tcp.size()); ++index) {
    SpreadsheetPlanRow row;
    row.row_index = index;
    row.row_id = "row-" + std::to_string(index + 1);
    std::copy(tcp[static_cast<std::size_t>(index)].begin(),
              tcp[static_cast<std::size_t>(index)].end(),
              row.tcp_pose);
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      row.joints_rad[axis] = 0.05 * static_cast<double>(index) * static_cast<double>(axis + 1);
    }
    row.tcp_velocity_m_s = kSpreadsheetPlannerDefaultSpeedMPS;
    row.playback_step_ms = static_cast<int>(std::ceil((side / row.tcp_velocity_m_s) * 1000.0));
    rows->push_back(row);
  }
}

// Keep motion-server row validation aligned with the Steam Deck Row View.
// A literal zero is retained as "not configured" so velocity-timed segments
// can fail closed. Preserve supported authored low speeds exactly; reject
// positive sub-UI values rather than silently accelerating them. Existing
// maximum caps remain unchanged.
constexpr double kSpreadsheetMinimumTcpVelocityMPS = 0.001 / 1000.0;
constexpr double kSpreadsheetMaximumTcpVelocityMPS = 0.500;
// Match Python math.radians(.001) operation order at the UI boundary.
constexpr double kSpreadsheetMinimumJointVelocityRadS = 0.001 * (kPi / 180.0);
constexpr double kSpreadsheetMaximumJointVelocityRadS = 1.50;

bool parse_spreadsheet_row_velocity(const std::string& object,
                                    const std::string& key,
                                    double minimum,
                                    double maximum,
                                    double* value) {
  if (value == nullptr) {
    return false;
  }
  if (object.find("\"" + key + "\"") == std::string::npos) {
    return true;
  }
  double parsed = 0.0;
  if (!parse_json_double(object, key, &parsed) || !std::isfinite(parsed) ||
      parsed < 0.0 || (parsed > 0.0 && parsed < minimum)) {
    return false;
  }
  *value = std::min(parsed, maximum);
  return true;
}

bool parse_spreadsheet_plan_rows(const std::string& raw, std::vector<SpreadsheetPlanRow>* rows) {
  if (rows == nullptr) {
    return false;
  }
  rows->clear();
  const std::vector<std::string> objects = json_object_slices_for_array_key(raw, "rows");
  std::set<int> row_indices;
  std::set<std::string> row_ids;
  for (const std::string& object : objects) {
    SpreadsheetPlanRow row;
    if (!parse_json_int(object, "row_index", &row.row_index) || row.row_index < 0) {
      return false;
    }
    (void)parse_json_int(object, "revision", &row.revision);
    (void)parse_json_bool(object, "enabled", &row.enabled);
    (void)parse_json_string(object, "row_id", &row.row_id);
    (void)parse_json_string(object, "row_hash", &row.row_hash);
    (void)parse_json_string(object, "label", &row.label);
    const bool explicit_row_type =
        object.find("\"row_type\"") != std::string::npos;
    (void)parse_json_string(object, "row_type", &row.row_type);
    (void)parse_json_string(object, "command", &row.command);
    (void)parse_json_int(object, "loop_target_row", &row.loop_target_row);
    (void)parse_json_string(object, "loop_target_row_id", &row.loop_target_row_id);
    if (!parse_json_int(object, "loop_max_iterations", &row.loop_max_iterations)) {
      (void)parse_json_int(object, "loop_iterations", &row.loop_max_iterations);
    }
    if (!parse_json_bool(object, "loop_return_home", &row.loop_return_home)) {
      (void)parse_json_bool(object, "return_home_before_repeat", &row.loop_return_home);
    }
    if (!explicit_row_type && !row.command.empty() &&
        row.row_type == "motion") {
      // Accept the concise command form, but normalize it immediately so the
      // planner and report have one unambiguous schema. An explicit row_type
      // always wins: shared Offline/SteamDeck motion rows also use their
      // Command cell for MOVE/HOME operator semantics.
      row.row_type = "command";
    }
    (void)parse_json_string(object, "move_type", &row.move_type);
    (void)parse_json_string(object, "target_frame_type", &row.target_frame_type);
    (void)parse_json_string(object, "motion_phase", &row.motion_phase);
    (void)parse_json_int(object, "calibration_section", &row.calibration_section);
    (void)parse_json_double(object, "harmonious_phase_deg", &row.harmonious_phase_deg);
    (void)parse_json_string(object, "tool_frame", &row.tool_frame);
    (void)parse_json_string(object, "work_frame", &row.work_frame);
    (void)parse_json_string(object, "fixture_side", &row.fixture_side);
    int playback_step_ms = 0;
    if (parse_json_int(object, "playback_step_ms", &playback_step_ms) &&
        playback_step_ms >= 0) {
      row.playback_step_ms = playback_step_ms;
    }
    const std::vector<double> tcp_pose = parse_json_number_array(object, "target_tcp_m");
    const std::vector<double> current_tcp_pose = parse_json_number_array(object, "current_tcp_pose");
    const std::vector<double> joints = parse_json_number_array(object, "target_joints_rad");
    const std::vector<double> current_joints = parse_json_number_array(object, "current_joints_rad");
    const std::vector<double>& tcp_source = tcp_pose.size() >= 6 ? tcp_pose : current_tcp_pose;
    const std::vector<double>& joints_source = joints.size() >= kLiveAxisCount ? joints : current_joints;
    if (tcp_source.size() >= 6) {
      for (int axis = 0; axis < 6; ++axis) {
        row.tcp_pose[axis] = tcp_source[static_cast<std::size_t>(axis)];
      }
    }
    if (joints_source.size() >= kLiveAxisCount) {
      for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
        row.joints_rad[axis] = joints_source[axis];
      }
    }
    if (!parse_spreadsheet_row_velocity(
            object, "tcp_velocity_m_s", kSpreadsheetMinimumTcpVelocityMPS,
            kSpreadsheetMaximumTcpVelocityMPS, &row.tcp_velocity_m_s) ||
        !parse_spreadsheet_row_velocity(
            object, "joint_velocity_rad_s",
            kSpreadsheetMinimumJointVelocityRadS,
            kSpreadsheetMaximumJointVelocityRadS,
            &row.joint_velocity_rad_s)) {
      return false;
    }
    (void)parse_json_bool(object, "weld_enable", &row.weld_enable);
    (void)parse_json_string(object, "arc_state", &row.arc_state);
    (void)parse_json_double(object, "wire_feed_m_per_min",
                            &row.wire_feed_m_per_min);
    (void)parse_json_double(object, "voltage_v", &row.voltage_v);
    (void)parse_json_string(object, "revpi_io_action", &row.revpi_io_action);
    (void)parse_json_int(object, "revpi_io_slot", &row.revpi_io_slot);
    if ((row.row_type == "motion" && row.move_type != "none") ||
        row.row_type == "command") {
      if (row.row_id.empty() || !row_indices.insert(row.row_index).second ||
          !row_ids.insert(row.row_id).second || row.revision < 0 ||
          (row.row_type == "motion" &&
           ((row.move_type == "joint_ptp" && joints_source.size() < kLiveAxisCount) ||
            (row.move_type != "joint_ptp" && tcp_source.size() < 6)))) {
        return false;
      }
      rows->push_back(row);
    }
  }
  for (SpreadsheetPlanRow& row : *rows) {
    if (row.row_type != "command" || row.loop_target_row >= 0 ||
        row.loop_target_row_id.empty()) {
      continue;
    }
    const auto target = std::find_if(
        rows->begin(), rows->end(), [&row](const SpreadsheetPlanRow& candidate) {
          return candidate.row_id == row.loop_target_row_id;
        });
    if (target == rows->end()) {
      return false;
    }
    row.loop_target_row = target->row_index;
  }
  int motion_row_count = 0;
  for (const SpreadsheetPlanRow& row : *rows) {
    if (row.enabled && row.row_type == "motion" &&
        row.move_type != "none") {
      motion_row_count++;
    }
  }
  return motion_row_count >= 2;
}

constexpr const char* kCalibrationTourBoxProgramID = "robot-v4-tour-calibration-box";
constexpr const char* kCalibrationTourBoxLegacyProgramID = "robot-v4-calibration-tour-box";
constexpr int kCalibrationTourBoxProgramRevision = 4;
constexpr double kCalibrationTourBoxCoordinatedAxisRad = kPi / 4.0;
constexpr const char* kRosie1420CalibrationBoxProgramID =
    "robot-v4-tour-calibration-box-rosie-1420-v1";
constexpr int kRosie1420CalibrationBoxProgramRevision = 1;
constexpr double kRosie1420CalibrationBoxHalfSpanM = 0.02;
constexpr double kRosie1420CalibrationBoxInwardOffsetM = 0.04;
constexpr double kRosie1420CalibrationBoxLiftM = 0.02;
constexpr double kRosie1420CalibrationBoxMaxHomeJointDeltaRad = 0.25;
constexpr int kRosie1420CalibrationBoxLoopIterations = 3;
constexpr const char* kCollisionRejectionDemoProgramID =
    "robot-v4-tour-collision-rejection-demo";

double tcp_distance_m(const double a[6], const double b[6]);
double max_joint_delta_rad(const double a[kLiveAxisCount],
                           const double b[kLiveAxisCount]);
void spreadsheet_fk_pose_from_joints(const double joints[kLiveAxisCount],
                                      double tcp_pose[6]);
bool spreadsheet_row_is_line_tcp(const SpreadsheetPlanRow& row);

std::string spreadsheet_path_parent_for_work_frame(
    const std::string& work_frame) {
  if (work_frame == "positioner_table_a") {
    return "table_a";
  }
  if (work_frame == "positioner_table_b") {
    return "table_b";
  }
  if (work_frame == "h_frame") {
    return "h_frame";
  }
  return "world";
}

std::string spreadsheet_path_parent_for_row(
    const SpreadsheetPlanRow& row) {
  // A J9/side-select stroke belongs to the H-frame itself. Table-local weld
  // and approach strokes inherit their individual J7/J8 child instead.
  if (row.motion_phase == "reconfigure" ||
      row.motion_phase == "side_reconfiguration" ||
      row.motion_phase.find("j9_") == 0) {
    return "h_frame";
  }
  return spreadsheet_path_parent_for_work_frame(row.work_frame);
}

void stamp_spreadsheet_path_metadata(
    SpreadsheetPlanPoint* point,
    const SpreadsheetPlanRow& row) {
  if (point == nullptr) {
    return;
  }
  point->work_frame = row.work_frame;
  point->fixture_side = row.fixture_side;
  point->path_parent = spreadsheet_path_parent_for_row(row);
  point->path_segment = !row.motion_phase.empty()
                            ? row.motion_phase
                            : (!row.row_id.empty() ? row.row_id
                                                   : row.label);
  if (row.calibration_section > 0) {
    point->path_segment += ":section_" +
                           std::to_string(row.calibration_section);
  }
}

bool is_calibration_tour_box_program_id(const std::string& program_id) {
  return program_id == kCalibrationTourBoxProgramID ||
         program_id == kCalibrationTourBoxLegacyProgramID ||
         program_id == "tour-calibration-box" ||
         program_id == "calibration-tour-box";
}

bool is_rosie_1420_calibration_box_program_id(
    const std::string& program_id) {
  return program_id == kRosie1420CalibrationBoxProgramID ||
         program_id == "tour-calibration-box-rosie-1420-v1";
}

bool is_collision_rejection_demo_program_id(const std::string& program_id) {
  return program_id == kCollisionRejectionDemoProgramID;
}

bool spreadsheet_program_is_simulation_only(const std::string& program_id) {
  return is_collision_rejection_demo_program_id(program_id);
}

void fill_calibration_tour_box_program_rows(
    std::vector<SpreadsheetPlanRow>* rows,
    const SpreadsheetHarmoniousAxisPolicy& policy) {
  if (rows == nullptr) {
    return;
  }
  rows->clear();
  constexpr double kVelocityMPS = 0.03;
  // Keep the tool +X axis down while rolling 30 degrees around that axis.
  // This selects the collision-free wrist branch represented by the
  // production row anchors without changing the weld approach direction.
  constexpr double kToolDownRollRad = -kPi / 6.0;
  constexpr double kToolDownPitchRad = kPi / 2.0;
  constexpr std::array<std::array<double, 3>, 4> kCorners = {{
      {{1.0256, -0.6220, 0.2700625}},  // C1
      {{1.2256, -0.6220, 0.2700625}},  // C2
      {{1.2256,  0.6220, 0.2700625}},  // C3
      {{1.0256,  0.6220, 0.2700625}},  // C4
  }};
  constexpr std::array<double, kLiveAxisCount> kHome = {{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  }};
  constexpr std::array<double, kLiveAxisCount> kRetract = {{
      0.0, -0.35, 0.45, 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
  }};
  constexpr std::array<double, kLiveAxisCount> kSectionAStart = {{
      -0.834828, 0.493850, -0.048588, 0.878764, -0.459107,
      1.756972, 0.0, 0.0, 0.0,
  }};
  constexpr std::array<double, kLiveAxisCount> kSectionAEnd = {{
      -0.709040, 0.725066, -0.403800, 0.840574, -0.492339,
      1.541364, -0.000170, 0.0, 0.0,
  }};
  // Exact C3 IK anchor for the rolled, tool-down orientation. Reusing it at
  // both section boundaries prevents a hidden Cartesian seek after the
  // Home-bounded reconfiguration.
  constexpr std::array<double, kLiveAxisCount> kSectionBMid = {{
      0.450685, 0.413255, 0.066518, 0.172995, -1.261521,
      0.216443, 0.0, 0.0, 0.0,
  }};
  constexpr std::array<double, kLiveAxisCount> kSectionBEnd = {{
      0.510098, 0.141241, 0.410505, 0.346903, -1.319211,
      0.421494, 0.000023, 0.0, 0.0,
  }};
  constexpr std::array<double, kLiveAxisCount> kSectionCEnd = {{
      -0.841750, 0.588021, -0.188687, 0.836380, -0.312688,
      1.828253, -0.000027, 0.0, 0.0,
  }};

  const auto append_joint_row = [&](const std::string& label,
                                    const std::string& phase,
                                    int section,
                                    double harmonious_phase_deg,
                                    const std::array<double, kLiveAxisCount>& joints,
                                    const std::array<double, 3>* authored_tcp) {
    SpreadsheetPlanRow row;
    row.row_index = static_cast<int>(rows->size());
    row.row_id = "row-" + std::to_string(row.row_index + 1);
    row.label = label;
    row.row_type = "motion";
    row.move_type = "joint_ptp";
    row.target_frame_type = "joint_positions";
    row.motion_phase = phase;
    row.calibration_section = section;
    row.harmonious_phase_deg = harmonious_phase_deg;
    row.work_frame = policy.work_frame;
    row.fixture_side = policy.active_side;
    std::copy(joints.begin(), joints.end(), row.joints_rad);
    row.joints_rad[8] = policy.selector_rad;
    spreadsheet_fk_pose_from_joints(row.joints_rad, row.tcp_pose);
    if (authored_tcp != nullptr) {
      row.tcp_pose[0] = (*authored_tcp)[0];
      row.tcp_pose[1] = (*authored_tcp)[1];
      row.tcp_pose[2] = (*authored_tcp)[2];
      row.tcp_pose[3] = kToolDownRollRad;
      row.tcp_pose[4] = kToolDownPitchRad;
      row.tcp_pose[5] = 0.0;
    }
    row.tcp_velocity_m_s = kVelocityMPS;
    row.playback_step_ms = 1000;
    row.weld_enable = false;
    row.arc_state = "off";
    row.revpi_io_slot = RTCORE_WELD_IO_SLOT;
    row.revpi_io_action = "none";
    rows->push_back(row);
  };

  const auto append_trace_row = [&](const std::string& label,
                                    int section,
                                    double harmonious_phase_deg,
                                    const std::array<double, 3>& tcp,
                                    const std::array<double, kLiveAxisCount>& joints,
                                    bool final_in_section) {
    SpreadsheetPlanRow row;
    row.row_index = static_cast<int>(rows->size());
    row.row_id = "row-" + std::to_string(row.row_index + 1);
    row.label = label;
    row.row_type = "motion";
    row.move_type = "line_tcp";
    row.target_frame_type = "tcp_cartesian";
    row.motion_phase = "calibration";
    row.calibration_section = section;
    row.harmonious_phase_deg = harmonious_phase_deg;
    row.work_frame = policy.work_frame;
    row.fixture_side = policy.active_side;
    row.tcp_pose[0] = tcp[0];
    row.tcp_pose[1] = tcp[1];
    row.tcp_pose[2] = tcp[2];
    row.tcp_pose[3] = kToolDownRollRad;
    row.tcp_pose[4] = kToolDownPitchRad;
    std::copy(joints.begin(), joints.end(), row.joints_rad);
    row.joints_rad[8] = policy.selector_rad;
    row.tcp_velocity_m_s = kVelocityMPS;
    row.playback_step_ms = final_in_section ? 1 : 1000;
    row.weld_enable = true;
    row.arc_state = final_in_section ? "end" : "hold";
    row.revpi_io_slot = RTCORE_WELD_IO_SLOT;
    row.revpi_io_action = final_in_section ? "weld_off" : "none";
    rows->push_back(row);
  };

  if (policy.active_side != "A" || policy.joint != "J7") {
    // The segmented branches below are qualified for Table A/J7 only. Return
    // a structurally incomplete, non-playable reference for any other side;
    // never mirror a branch onto Table B without separate Tesseract evidence.
    append_joint_row("StartUnsupportedSide", "home", 0, 0.0, kHome, nullptr);
    append_joint_row("ReturnStartUnsupportedSide", "return_home", 0, 0.0, kHome, nullptr);
  } else {
    append_joint_row("Start", "home", 0, 0.0, kHome, nullptr);
    append_joint_row("RetractBeforeL1", "reconfigure", 0, 0.0,
                     kRetract, nullptr);
    append_joint_row("TableCorner1", "calibration", 1, 0.0,
                     kSectionAStart, &kCorners[0]);
    rows->back().weld_enable = true;
    rows->back().arc_state = "start";
    rows->back().revpi_io_action = "weld_on";
    std::array<double, kLiveAxisCount> section_a_corner_2 = kSectionAEnd;
    section_a_corner_2[6] = kCalibrationTourBoxCoordinatedAxisRad;
    append_trace_row("TableCorner2", 1, 0.0, kCorners[1],
                     section_a_corner_2, false);
    std::array<double, kLiveAxisCount> section_a_corner_3 = kSectionBMid;
    section_a_corner_3[6] = 0.0;
    append_trace_row("TableCorner3", 1, 0.0, kCorners[2],
                     section_a_corner_3, true);
    append_joint_row("L1SettleJ7Zero", "section_settle", 1, 0.0,
                     section_a_corner_3, &kCorners[2]);

    append_joint_row("RetractAfterL1", "reconfigure", 0, 0.0,
                     kRetract, nullptr);
    append_joint_row("ReturnHomeAfterL1", "section_home", 0, 0.0,
                     kHome, nullptr);
    append_joint_row("RetractBeforeL2", "reconfigure", 0, 0.0,
                     kRetract, nullptr);
    append_joint_row("SectionBStartC3", "calibration", 2, 0.0,
                     section_a_corner_3, &kCorners[2]);
    rows->back().weld_enable = true;
    rows->back().arc_state = "start";
    rows->back().revpi_io_action = "weld_on";
    std::array<double, kLiveAxisCount> section_b_corner_4 = kSectionBEnd;
    section_b_corner_4[6] = kCalibrationTourBoxCoordinatedAxisRad;
    append_trace_row("TableCorner4", 2, 0.0, kCorners[3],
                     section_b_corner_4, false);
    std::array<double, kLiveAxisCount> section_b_corner_1 = kSectionCEnd;
    section_b_corner_1[6] = 0.0;
    append_trace_row("SectionBEndC1", 2, 0.0, kCorners[0],
                     section_b_corner_1, true);
    append_joint_row("L2SettleJ7Zero", "section_settle", 2, 0.0,
                     section_b_corner_1, &kCorners[0]);
    append_joint_row("RetractAfterL2", "reconfigure", 0, 0.0,
                     kRetract, nullptr);
    append_joint_row("ReturnStart", "return_home", 0, 0.0, kHome, nullptr);

    // Row duration belongs to the segment departing that row.
    for (std::size_t index = 0; index + 1 < rows->size(); ++index) {
      SpreadsheetPlanRow& from = (*rows)[index];
      const SpreadsheetPlanRow& to = (*rows)[index + 1];
      if (spreadsheet_row_is_line_tcp(to)) {
        from.playback_step_ms = std::max(
            1000, static_cast<int>(std::lround(
                      tcp_distance_m(from.tcp_pose, to.tcp_pose) /
                      kVelocityMPS * 1000.0)));
      } else if (to.motion_phase == "section_settle") {
        from.playback_step_ms = 1;
      } else {
        constexpr double kNominalJointPTPSpeedRadS = 0.25;
        from.playback_step_ms = std::max(
            1000, static_cast<int>(std::lround(
                      max_joint_delta_rad(from.joints_rad, to.joints_rad) /
                      kNominalJointPTPSpeedRadS * 1000.0)));
      }
    }
  }
  SpreadsheetPlanRow loop;
  loop.row_index = static_cast<int>(rows->size());
  loop.row_id = "row-" + std::to_string(loop.row_index + 1);
  loop.label = "LOOP";
  loop.row_type = "command";
  loop.command = "LOOP";
  loop.move_type = "none";
  loop.target_frame_type = "none";
  loop.motion_phase = "loop";
  loop.calibration_section = 0;
  loop.work_frame = policy.work_frame;
  loop.fixture_side = policy.active_side;
  loop.loop_target_row = 0;
  loop.loop_max_iterations = 1;
  loop.loop_return_home = true;
  loop.playback_step_ms = 0;
  loop.arc_state = "off";
  loop.revpi_io_action = "none";
  rows->push_back(loop);
}

void fill_rosie_1420_calibration_box_program_rows(
    std::vector<SpreadsheetPlanRow>* rows) {
  if (rows == nullptr) {
    return;
  }
  rows->clear();
  constexpr double kVelocityMPS = 0.03;
  constexpr std::array<double, kLiveAxisCount> kHome = {{
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
  }};
  double home_tcp[6]{};
  spreadsheet_fk_pose_from_joints(kHome.data(), home_tcp);
  const double center_x =
      home_tcp[0] - kRosie1420CalibrationBoxInwardOffsetM;
  const double center_y = home_tcp[1];
  const double center_z = home_tcp[2] + kRosie1420CalibrationBoxLiftM;
  const std::array<std::array<double, 3>, 4> corners = {{
      {{center_x, center_y - kRosie1420CalibrationBoxHalfSpanM,
        center_z - kRosie1420CalibrationBoxHalfSpanM}},
      {{center_x, center_y + kRosie1420CalibrationBoxHalfSpanM,
        center_z - kRosie1420CalibrationBoxHalfSpanM}},
      {{center_x, center_y + kRosie1420CalibrationBoxHalfSpanM,
        center_z + kRosie1420CalibrationBoxHalfSpanM}},
      {{center_x, center_y - kRosie1420CalibrationBoxHalfSpanM,
        center_z + kRosie1420CalibrationBoxHalfSpanM}},
  }};

  const auto append_joint_row = [&](const std::string& label,
                                    const std::string& phase) {
    SpreadsheetPlanRow row;
    row.row_index = static_cast<int>(rows->size());
    row.row_id = "row-" + std::to_string(row.row_index + 1);
    row.label = label;
    row.row_type = "motion";
    row.move_type = "joint_ptp";
    row.target_frame_type = "joint_positions";
    row.motion_phase = phase;
    row.calibration_section = 0;
    row.work_frame = "world";
    row.fixture_side = "arm";
    std::copy(kHome.begin(), kHome.end(), row.joints_rad);
    spreadsheet_fk_pose_from_joints(row.joints_rad, row.tcp_pose);
    row.tcp_velocity_m_s = kVelocityMPS;
    row.joint_velocity_rad_s = 0.1;
    row.playback_step_ms = 1000;
    row.weld_enable = false;
    row.arc_state = "off";
    row.revpi_io_slot = RTCORE_WELD_IO_SLOT;
    row.revpi_io_action = "none";
    rows->push_back(row);
  };
  const auto append_corner_row = [&](const std::string& label,
                                     const std::array<double, 3>& corner,
                                     double roll_offset_rad) {
    SpreadsheetPlanRow row;
    row.row_index = static_cast<int>(rows->size());
    row.row_id = "row-" + std::to_string(row.row_index + 1);
    row.label = label;
    row.row_type = "motion";
    row.move_type = "line_tcp";
    row.target_frame_type = "tcp_cartesian";
    row.motion_phase = "calibration";
    row.calibration_section = 1;
    row.work_frame = "world";
    row.fixture_side = "arm";
    row.tcp_pose[0] = corner[0];
    row.tcp_pose[1] = corner[1];
    row.tcp_pose[2] = corner[2];
    row.tcp_pose[3] = home_tcp[3] + roll_offset_rad;
    row.tcp_pose[4] = home_tcp[4];
    row.tcp_pose[5] = home_tcp[5];
    std::copy(kHome.begin(), kHome.end(), row.joints_rad);
    row.tcp_velocity_m_s = kVelocityMPS;
    row.joint_velocity_rad_s = 0.1;
    row.playback_step_ms = 1000;
    row.weld_enable = false;
    row.arc_state = "off";
    row.revpi_io_slot = RTCORE_WELD_IO_SLOT;
    row.revpi_io_action = "none";
    rows->push_back(row);
  };

  append_joint_row("Start", "home");
  append_corner_row("NearHomeCorner1", corners[0], 0.0);
  append_corner_row("NearHomeCorner2", corners[1], 0.02);
  append_corner_row("NearHomeCorner3", corners[2], 0.02);
  append_corner_row("NearHomeCorner4", corners[3], -0.02);
  append_corner_row("CloseNearHomeBox", corners[0], 0.0);
  append_joint_row("ReturnStart", "return_home");
  for (std::size_t index = 0; index + 1 < rows->size(); ++index) {
    SpreadsheetPlanRow& from = (*rows)[index];
    const SpreadsheetPlanRow& to = (*rows)[index + 1];
    if (spreadsheet_row_is_line_tcp(to)) {
      from.playback_step_ms = std::max(
          500, static_cast<int>(std::lround(
                   tcp_distance_m(from.tcp_pose, to.tcp_pose) /
                   kVelocityMPS * 1000.0)));
    } else {
      from.playback_step_ms = std::max(
          500, static_cast<int>(std::lround(
                   max_joint_delta_rad(from.joints_rad, to.joints_rad) /
                   0.1 * 1000.0)));
    }
  }

  SpreadsheetPlanRow loop;
  loop.row_index = static_cast<int>(rows->size());
  loop.row_id = "row-" + std::to_string(loop.row_index + 1);
  loop.label = "LOOP";
  loop.row_type = "command";
  loop.command = "LOOP";
  loop.move_type = "none";
  loop.target_frame_type = "none";
  loop.motion_phase = "loop";
  loop.work_frame = "world";
  loop.fixture_side = "arm";
  loop.loop_target_row = 0;
  loop.loop_max_iterations = kRosie1420CalibrationBoxLoopIterations;
  loop.loop_return_home = true;
  loop.playback_step_ms = 0;
  loop.arc_state = "off";
  loop.revpi_io_action = "none";
  rows->push_back(loop);
}

// Intentionally unsafe planner fixture used only to demonstrate that
// Tesseract identifies a collision and the motion-server refuses to install
// or play the program.  The tour may render its planned geometry and contact
// points, but this program is never a preload candidate, including in
// isolated simulation.
void fill_collision_rejection_demo_program_rows(
    std::vector<SpreadsheetPlanRow>* rows) {
  if (rows == nullptr) {
    return;
  }
  rows->clear();
  constexpr std::array<std::array<double, 3>, 2> kTargets = {{
      {{0.90, 0.0, 0.0}},
      {{0.18, 0.0, 0.0}},
  }};
  constexpr std::array<const char*, 2> kLabels = {{
      "CollisionDemoStart",
      "CollisionDemoIntoBase",
  }};
  for (std::size_t index = 0; index < kTargets.size(); ++index) {
    SpreadsheetPlanRow row;
    row.row_index = static_cast<int>(index);
    row.row_id = "collision-demo-row-00" + std::to_string(index + 1);
    row.label = kLabels[index];
    row.row_type = "motion";
    row.move_type = "line_tcp";
    row.target_frame_type = "tcp_cartesian";
    row.work_frame = "world";
    row.tcp_pose[0] = kTargets[index][0];
    row.tcp_pose[1] = kTargets[index][1];
    row.tcp_pose[2] = kTargets[index][2];
    row.tcp_velocity_m_s = 0.05;
    row.playback_step_ms = index == 0 ? 1000 : 14400;
    row.weld_enable = false;
    row.arc_state = "off";
    row.revpi_io_action = "none";
    rows->push_back(row);
  }
}

SpreadsheetToolAxisConstraint calibration_tour_box_default_tool_constraint() {
  SpreadsheetToolAxisConstraint constraint;
  constraint.present = true;
  constraint.valid = true;
  return constraint;
}

SpreadsheetHarmoniousAxisPolicy calibration_tour_box_default_harmonious_axis() {
  SpreadsheetHarmoniousAxisPolicy policy;
  policy.present = true;
  policy.valid = true;
  policy.joint = "J7";
  policy.active_side = "A";
  policy.selector_joint = "J9";
  policy.selector_rad = 0.0;
  policy.selector_tolerance_rad = 0.02;
  policy.center_rad = 0.0;
  policy.amplitude_rad = kCalibrationTourBoxCoordinatedAxisRad;
  policy.phase_deg = 0.0;
  policy.trajectory_source = "row_targets";
  policy.work_frame = "positioner_table_a";
  policy.workplane_relative = true;
  policy.workplane_pivot_m[0] = 1.1256;
  policy.workplane_pivot_m[1] = 0.62200026;
  policy.workplane_pivot_m[2] = 0.0700413;
  policy.workplane_axis_xyz[0] = 0.0;
  policy.workplane_axis_xyz[1] = 1.0;
  policy.workplane_axis_xyz[2] = 0.0;
  policy.reference_rad = 0.0;
  // Rotate the Table-A-owned position path into a world-space curve while the
  // qualified torch axis remains world-down. Following J7 with the torch axis
  // made the full +100 mm rectangle unreachable/colliding in Tesseract.
  policy.tool_axis_follows_workplane = false;
  return policy;
}

std::array<double, kLiveAxisCount> spreadsheet_plan_joint_speed_limits(
    const std::string& raw, const RTCorePairConfigBinding* authority = nullptr) {
  // The network planner passes its fresh native snapshot. Client JSON cannot
  // select a cell or raise authority. Legacy/offline calls retain 0.2.
  std::array<double, kLiveAxisCount> limits{};
  if (authority != nullptr) {
    limits = plan_playback_joint_limits(*authority, authority->mapped_axis_mask);
    for (std::size_t axis = 0; axis < limits.size(); ++axis)
      if ((authority->mapped_axis_mask & (1u << axis)) == 0u)
        limits[axis] = authority->valid ? PLAN_PLAYBACK_MAX_ABS_QD_RAD_S : 0.0;
  } else {
    limits.fill(PLAN_PLAYBACK_MAX_ABS_QD_RAD_S);
  }
  const std::vector<double> requested =
      parse_json_number_array(raw, "global_joint_speed_limits_rad_s");
  for (std::size_t axis = 0; axis < requested.size() && axis < limits.size(); ++axis) {
    // Positive padding is required by the nine-slot retimer. Unmapped output
    // remains forbidden by the exact native command/plan authority gates.
    if (authority != nullptr && authority->valid &&
        (authority->mapped_axis_mask & (1u << axis)) == 0u) continue;
    if (!std::isfinite(requested[axis]) || requested[axis] <= 0.0)
      limits[axis] = 0.0;
    else
      limits[axis] = std::min(requested[axis], limits[axis]);
  }
  return limits;
}

double spreadsheet_plan_singularity_threshold(const std::string& raw,
                                              const std::string& key,
                                              double fallback) {
  double value = fallback;
  if (parse_json_double(raw, key, &value) && std::isfinite(value) &&
      value >= 0.0 && value <= 1.0) {
    return value;
  }
  return fallback;
}

bool spreadsheet_authoritative_calibration_rows_valid(
    const std::vector<SpreadsheetPlanRow>& rows,
    int program_revision,
    int base_program_revision) {
  if (program_revision < 1 || base_program_revision < 1 ||
      program_revision < base_program_revision || rows.size() < 3) {
    return false;
  }
  int command_count = 0;
  for (std::size_t index = 0; index < rows.size(); ++index) {
    const SpreadsheetPlanRow& row = rows[index];
    if (row.row_index != static_cast<int>(index) || row.revision < 1 ||
        row.revision > program_revision || row.row_hash.empty()) {
      return false;
    }
    if (row.row_type == "command") {
      command_count++;
      if (index + 1 != rows.size() ||
          (row.command != "LOOP" && row.command != "loop") ||
          row.move_type != "none" || row.loop_target_row != 0 ||
          row.loop_max_iterations < 1 || row.loop_max_iterations > 8 ||
          !row.loop_return_home) {
        return false;
      }
      continue;
    }
    if (row.row_type != "motion" || !row.command.empty() ||
        (row.move_type != "line_tcp" && row.move_type != "curve_tcp" &&
         row.move_type != "joint_ptp") ||
        row.tool_frame.empty() || row.work_frame.empty() ||
        !std::isfinite(row.harmonious_phase_deg) ||
        row.calibration_section < 0) {
      return false;
    }
  }
  return command_count == 1;
}

bool resolve_spreadsheet_plan_request(const std::string& raw,
                                      std::vector<SpreadsheetPlanRow>* rows,
                                      SpreadsheetToolAxisConstraint* tool_axis_constraint,
                                      SpreadsheetHarmoniousAxisPolicy* harmonious_axis_policy,
                                      std::string* program_id,
                                      std::string* program_source,
                                      int* program_revision) {
  if (rows == nullptr || tool_axis_constraint == nullptr ||
      harmonious_axis_policy == nullptr || program_id == nullptr ||
      program_source == nullptr || program_revision == nullptr) {
    return false;
  }
  *program_id = "";
  (void)parse_json_string(raw, "program_id", program_id);
  *program_revision = 1;
  if (!parse_json_int(raw, "program_revision", program_revision)) {
    (void)parse_json_int(raw, "revision", program_revision);
  }
  if (is_rosie_1420_calibration_box_program_id(*program_id)) {
    *program_id = kRosie1420CalibrationBoxProgramID;
    *tool_axis_constraint = SpreadsheetToolAxisConstraint{};
    *harmonious_axis_policy = SpreadsheetHarmoniousAxisPolicy{};
    bool program_reference_only = false;
    bool rows_omitted_by_client = false;
    const bool reference_flag_present =
        parse_json_bool(raw, "program_reference_only",
                        &program_reference_only);
    const bool omitted_flag_present =
        parse_json_bool(raw, "rows_omitted_by_client",
                        &rows_omitted_by_client);
    const bool request_has_rows =
        !json_object_slices_for_array_key(raw, "rows").empty();
    const bool reference_request =
        (!request_has_rows && !reference_flag_present &&
         !omitted_flag_present) ||
        (program_reference_only && rows_omitted_by_client &&
         !request_has_rows);
    if (!reference_request) {
      return false;
    }
    *program_source = "motion-server-builtin-program-database";
    *program_revision = kRosie1420CalibrationBoxProgramRevision;
    fill_rosie_1420_calibration_box_program_rows(rows);
    return rows->size() >= 2;
  }
  if (is_calibration_tour_box_program_id(*program_id)) {
    *program_id = kCalibrationTourBoxProgramID;
    *tool_axis_constraint = parse_spreadsheet_tool_axis_constraint(raw);
    if (!tool_axis_constraint->present) {
      *tool_axis_constraint = calibration_tour_box_default_tool_constraint();
    }
    *harmonious_axis_policy = parse_spreadsheet_harmonious_axis_policy(raw);
    if (!harmonious_axis_policy->present) {
      *harmonious_axis_policy = calibration_tour_box_default_harmonious_axis();
    }
    bool program_reference_only = false;
    bool rows_omitted_by_client = false;
    const bool reference_flag_present =
        parse_json_bool(raw, "program_reference_only", &program_reference_only);
    const bool omitted_flag_present =
        parse_json_bool(raw, "rows_omitted_by_client", &rows_omitted_by_client);
    const bool request_has_rows =
        !json_object_slices_for_array_key(raw, "rows").empty();
    const bool reference_request =
        (!request_has_rows && !reference_flag_present && !omitted_flag_present) ||
        (program_reference_only && rows_omitted_by_client && !request_has_rows);
    if (reference_request) {
      *program_source = "motion-server-builtin-program-database";
      *program_revision = kCalibrationTourBoxProgramRevision;
      harmonious_axis_policy->center_rad = 0.0;
      harmonious_axis_policy->amplitude_rad =
          kCalibrationTourBoxCoordinatedAxisRad;
      harmonious_axis_policy->trajectory_source = "row_targets";
      harmonious_axis_policy->phase_deg = 0.0;
      fill_calibration_tour_box_program_rows(rows, *harmonious_axis_policy);
      return rows->size() >= 2;
    }
    if (!request_has_rows || program_reference_only || rows_omitted_by_client) {
      return false;
    }
    int base_program_revision = 0;
    if (!parse_json_int(raw, "base_program_revision", &base_program_revision) ||
        !parse_spreadsheet_plan_rows(raw, rows) ||
        !spreadsheet_authoritative_calibration_rows_valid(
            *rows, *program_revision, base_program_revision)) {
      return false;
    }
    // Once hydrated, the spreadsheet's J7/J8 targets are part of the exact
    // optimistic row update. Do not regenerate a sinusoid over edited rows.
    harmonious_axis_policy->trajectory_source = "row_targets";
    *program_source = "motion-server-authoritative-client-edited-program-rows";
    return true;
  }
  if (is_collision_rejection_demo_program_id(*program_id)) {
    *program_source = "motion-server-builtin-simulation-only-program-database";
    *tool_axis_constraint = SpreadsheetToolAxisConstraint{};
    *harmonious_axis_policy = SpreadsheetHarmoniousAxisPolicy{};
    fill_collision_rejection_demo_program_rows(rows);
    return rows->size() >= 2;
  }
  *program_source = "client-authored-program-rows";
  *tool_axis_constraint = parse_spreadsheet_tool_axis_constraint(raw);
  *harmonious_axis_policy = parse_spreadsheet_harmonious_axis_policy(raw);
  return parse_spreadsheet_plan_rows(raw, rows);
}

double tcp_distance_m(const double a[6], const double b[6]) {
  const double dx = a[0] - b[0];
  const double dy = a[1] - b[1];
  const double dz = a[2] - b[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool spreadsheet_row_is_motion(const SpreadsheetPlanRow& row) {
  return row.enabled && row.row_type == "motion" && row.move_type != "none";
}

bool spreadsheet_row_is_joint_ptp(const SpreadsheetPlanRow& row) {
  return spreadsheet_row_is_motion(row) && row.move_type == "joint_ptp";
}

bool spreadsheet_row_is_line_tcp(const SpreadsheetPlanRow& row) {
  return spreadsheet_row_is_motion(row) && row.move_type == "line_tcp";
}

bool spreadsheet_joints_are_policy_home(
    const double joints[kLiveAxisCount],
    const SpreadsheetHarmoniousAxisPolicy& policy,
    double tolerance_rad = 1.0e-9) {
  if (joints == nullptr) {
    return false;
  }
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    const double expected = axis == 8u && policy.present ? policy.selector_rad : 0.0;
    if (!std::isfinite(joints[axis]) ||
        std::abs(joints[axis] - expected) > tolerance_rad) {
      return false;
    }
  }
  return true;
}

void spreadsheet_fk_pose_from_joints(const double joints[kLiveAxisCount],
                                      double tcp_pose[6]) {
  double arm_joints[6]{};
  for (int axis = 0; axis < 6; ++axis) {
    arm_joints[axis] = joints[axis];
  }
  const Mat4 transform = urdf_forward_kinematics_j1_base(arm_joints);
  tcp_pose[0] = transform[0][3];
  tcp_pose[1] = transform[1][3];
  tcp_pose[2] = transform[2][3];
  const double pitch = std::asin(std::clamp(-transform[2][0], -1.0, 1.0));
  const double cos_pitch = std::cos(pitch);
  if (std::abs(cos_pitch) > 1.0e-9) {
    tcp_pose[3] = std::atan2(transform[2][1], transform[2][2]);
    tcp_pose[5] = std::atan2(transform[1][0], transform[0][0]);
  } else {
    tcp_pose[3] = std::atan2(-transform[1][2], transform[1][1]);
    tcp_pose[5] = 0.0;
  }
  tcp_pose[4] = pitch;
}

constexpr double kSpreadsheetFKTranslationToleranceM = 0.005;
constexpr double kSpreadsheetFKOrientationToleranceRad = 0.05;

void spreadsheet_rpy_rotation_matrix(const double pose[6],
                                     double rotation[3][3]) {
  const double cr = std::cos(pose[3]);
  const double sr = std::sin(pose[3]);
  const double cp = std::cos(pose[4]);
  const double sp = std::sin(pose[4]);
  const double cy = std::cos(pose[5]);
  const double sy = std::sin(pose[5]);
  rotation[0][0] = cy * cp;
  rotation[0][1] = cy * sp * sr - sy * cr;
  rotation[0][2] = cy * sp * cr + sy * sr;
  rotation[1][0] = sy * cp;
  rotation[1][1] = sy * sp * sr + cy * cr;
  rotation[1][2] = sy * sp * cr - cy * sr;
  rotation[2][0] = -sp;
  rotation[2][1] = cp * sr;
  rotation[2][2] = cp * cr;
}

double spreadsheet_pose_translation_residual_m(const double expected[6],
                                               const double actual[6]) {
  const double dx = expected[0] - actual[0];
  const double dy = expected[1] - actual[1];
  const double dz = expected[2] - actual[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double spreadsheet_pose_orientation_residual_rad(const double expected[6],
                                                 const double actual[6]) {
  double expected_rotation[3][3]{};
  double actual_rotation[3][3]{};
  spreadsheet_rpy_rotation_matrix(expected, expected_rotation);
  spreadsheet_rpy_rotation_matrix(actual, actual_rotation);
  double relative_trace = 0.0;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      relative_trace +=
          expected_rotation[row][column] * actual_rotation[row][column];
    }
  }
  return std::acos(std::clamp((relative_trace - 1.0) / 2.0, -1.0, 1.0));
}

double max_joint_delta_rad(const double a[kLiveAxisCount],
                           const double b[kLiveAxisCount]) {
  double max_delta = 0.0;
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    max_delta = std::max(max_delta, std::abs(b[axis] - a[axis]));
  }
  return max_delta;
}

bool spreadsheet_segment_velocity_timing_valid(
    const SpreadsheetPlanRow& from,
    const SpreadsheetPlanRow& to) {
  if (from.playback_step_ms > 0) {
    return true;
  }
  if (spreadsheet_row_is_joint_ptp(to)) {
    return std::isfinite(from.joint_velocity_rad_s) &&
           from.joint_velocity_rad_s >= kSpreadsheetMinimumJointVelocityRadS &&
           from.joint_velocity_rad_s <= kSpreadsheetMaximumJointVelocityRadS;
  }
  return std::isfinite(from.tcp_velocity_m_s) &&
         from.tcp_velocity_m_s >= kSpreadsheetMinimumTcpVelocityMPS &&
         from.tcp_velocity_m_s <= kSpreadsheetMaximumTcpVelocityMPS;
}

double spreadsheet_segment_duration_s(const SpreadsheetPlanRow& from,
                                      const SpreadsheetPlanRow& to) {
  constexpr double kMinimumSegmentDurationS = 0.001;
  if (from.playback_step_ms > 0) {
    return std::max(
        kMinimumSegmentDurationS,
        static_cast<double>(from.playback_step_ms) / 1000.0);
  }
  if (!spreadsheet_segment_velocity_timing_valid(from, to)) {
    return std::numeric_limits<double>::infinity();
  }
  if (spreadsheet_row_is_joint_ptp(to)) {
    return std::max(
        kMinimumSegmentDurationS,
        max_joint_delta_rad(from.joints_rad, to.joints_rad) /
            from.joint_velocity_rad_s);
  }
  return std::max(
      kMinimumSegmentDurationS,
      tcp_distance_m(from.tcp_pose, to.tcp_pose) / from.tcp_velocity_m_s);
}

SpreadsheetLoopCommand validate_spreadsheet_loop_command(
    const std::vector<SpreadsheetPlanRow>& program_rows) {
  SpreadsheetLoopCommand result;
  const SpreadsheetPlanRow* first_motion = nullptr;
  const SpreadsheetPlanRow* last_motion = nullptr;
  for (std::size_t index = 0; index < program_rows.size(); ++index) {
    const SpreadsheetPlanRow& row = program_rows[index];
    if (spreadsheet_row_is_motion(row)) {
      if (first_motion == nullptr) {
        first_motion = &row;
      }
      last_motion = &row;
      continue;
    }
    if (!row.enabled || row.row_type != "command") {
      continue;
    }
    if (result.present) {
      result.valid = false;
      result.validation_error = "multiple_command_rows";
      continue;
    }
    result.present = true;
    result.row_index = row.row_index;
    result.target_row = row.loop_target_row;
    result.max_iterations = row.loop_max_iterations;
    result.return_home = row.loop_return_home;
    if (index + 1 != program_rows.size()) {
      result.valid = false;
      result.validation_error = "loop_command_must_be_terminal";
    } else if ((row.command != "LOOP" && row.command != "loop") ||
               row.move_type != "none") {
      result.valid = false;
      result.validation_error = "unsupported_terminal_command";
    } else if (row.loop_max_iterations < 1 || row.loop_max_iterations > 8) {
      result.valid = false;
      result.validation_error = "loop_iterations_must_be_bounded_1_to_8";
    } else if (!row.loop_return_home) {
      result.valid = false;
      result.validation_error = "loop_requires_return_home";
    }
  }
  if (!result.present || !result.valid) {
    return result;
  }
  if (first_motion == nullptr || last_motion == nullptr ||
      result.target_row != first_motion->row_index) {
    result.valid = false;
    result.validation_error = "loop_target_must_be_first_motion_row";
  } else if (first_motion->work_frame != last_motion->work_frame ||
             first_motion->fixture_side != last_motion->fixture_side ||
             tcp_distance_m(first_motion->tcp_pose, last_motion->tcp_pose) > 1.0e-6 ||
             max_joint_delta_rad(first_motion->joints_rad, last_motion->joints_rad) > 1.0e-9) {
    result.valid = false;
    result.validation_error = "loop_requires_explicit_return_home_motion_row";
  }
  return result;
}

// Resolve a repo-relative file (e.g. the planning URDF or the collision
// worker) by walking up from the current working directory. Deployed daemons
// that do not run from a repo checkout can override via environment.
std::string resolve_repo_relative_file(const std::string& relative) {
  std::string prefix;
  for (int depth = 0; depth < 8; ++depth) {
    const std::string candidate = prefix + relative;
    if (::access(candidate.c_str(), R_OK) == 0) {
      return candidate;
    }
    prefix += "../";
  }
  return "";
}

std::string resolve_spreadsheet_planner_urdf_path() {
  if (g_async_planning_kinematics) return g_async_planning_kinematics->urdf_path;
  const char* override_path = std::getenv("ROBOT_V4_TESSERACT_URDF_PATH");
  if (override_path != nullptr && override_path[0] != '\0') {
    if (::access(override_path, R_OK) == 0) {
      return override_path;
    }
    const std::string revision_path =
        motion_server_authoritative_urdf_path();
    if (!revision_path.empty()) {
      return revision_path;
    }
    // A deployed motion server is explicitly bound to the Jetson's active
    // persisted revision. Never substitute a repo fixture if that authority
    // is absent or unreadable.
    return "";
  }
  return resolve_repo_relative_file(kMotionServerTesseractURDFSource);
}

// Parse the revolute joint limits for the planner axes straight from the same
// selected-pair URDF the collision checker loads, so the limit check and the
// mesh model can never drift apart. The shared trajectory envelope has nine
// positions; rosie_1420_v1 owns only J1-J6, so absent J7-J9 are represented by
// exact zero limits and can never acquire motion.
SpreadsheetJointLimits load_spreadsheet_joint_limits() {
  SpreadsheetJointLimits limits;
  const std::string urdf_path = resolve_spreadsheet_planner_urdf_path();
  if (urdf_path.empty()) {
    return limits;
  }
  std::string urdf;
  if (!read_text_file(urdf_path, &urdf)) {
    return limits;
  }
  const auto parse_attribute = [](const std::string& text, std::string::size_type from,
                                  std::string::size_type until, const std::string& name,
                                  double* out) {
    const std::string marker = name + "=\"";
    const std::string::size_type pos = text.find(marker, from);
    if (pos == std::string::npos || pos >= until) {
      return false;
    }
    const std::string::size_type value_start = pos + marker.size();
    const std::string::size_type value_end = text.find('"', value_start);
    if (value_end == std::string::npos) {
      return false;
    }
    try {
      *out = std::stod(text.substr(value_start, value_end - value_start));
    } catch (...) {
      return false;
    }
    return true;
  };
  const bool rosie_1420 =
      urdf.find("<robot name=\"rosie_1420_v1\"") !=
      std::string::npos;
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    const std::string joint_marker =
        std::string("<joint name=\"") + spreadsheet_planner_joint_name(axis) + "\"";
    const std::string::size_type joint_pos = urdf.find(joint_marker);
    if (joint_pos == std::string::npos) {
      if (rosie_1420 && axis >= 6u) {
        limits.lower_rad[axis] = 0.0;
        limits.upper_rad[axis] = 0.0;
        continue;
      }
      return limits;
    }
    const std::string::size_type joint_end = urdf.find("</joint>", joint_pos);
    if (joint_end == std::string::npos) {
      return limits;
    }
    if (!parse_attribute(urdf, joint_pos, joint_end, "lower", &limits.lower_rad[axis]) ||
        !parse_attribute(urdf, joint_pos, joint_end, "upper", &limits.upper_rad[axis])) {
      return limits;
    }
  }
  limits.loaded = true;
  limits.source = urdf_path;
  return limits;
}

double nearest_equivalent_joint_angle_within_limits(double value_rad,
                                                     double reference_rad,
                                                     double lower_rad,
                                                     double upper_rad,
                                                     bool* wrapped) {
  double best = value_rad;
  double best_distance = std::numeric_limits<double>::infinity();
  for (int turns = -3; turns <= 3; ++turns) {
    const double candidate = value_rad + static_cast<double>(turns) * 2.0 * kPi;
    if (candidate < lower_rad || candidate > upper_rad) {
      continue;
    }
    const double distance = std::abs(candidate - reference_rad);
    if (distance < best_distance) {
      best = candidate;
      best_distance = distance;
    }
  }
  if (wrapped != nullptr) {
    *wrapped = std::abs(best - value_rad) > 1.0e-9;
  }
  return best;
}

std::string shell_quote_path(const std::string& path) {
  return "'" + path + "'";
}

bool spreadsheet_collision_pair_identity(
    const std::string& urdf_path,
    std::string* robot_model,
    std::string* mesh_package_root) {
  if (robot_model == nullptr || mesh_package_root == nullptr) {
    return false;
  }
  std::string urdf;
  if (!read_text_file(urdf_path, &urdf)) {
    return false;
  }
  const std::size_t robot_begin = urdf.find("<robot");
  const std::size_t robot_end =
      robot_begin == std::string::npos
          ? std::string::npos
          : urdf.find('>', robot_begin);
  std::string model;
  if (robot_begin == std::string::npos ||
      robot_end == std::string::npos ||
      !extract_xml_attribute(
          urdf.substr(robot_begin, robot_end - robot_begin + 1),
          "name", &model) ||
      (model != "rosie_1400_v3" &&
       model != "rosie_1420_v1")) {
    return false;
  }
  const std::string canonical_urdf = resolve_repo_relative_file(
      "robot_description/robots/" + model + "/robot.urdf");
  if (canonical_urdf.empty()) {
    return false;
  }
  *robot_model = model;
  *mesh_package_root = parent_directory(canonical_urdf);
  return !mesh_package_root->empty();
}

bool spreadsheet_collision_server_request(
    const std::string& address,
    const std::string& request,
    std::string* response,
    std::string* error) {
  constexpr std::size_t kMaxFrameBytes = 64u * 1024u * 1024u;
  const std::string prefix = "127.0.0.1:";
  if (response == nullptr || address.rfind(prefix, 0) != 0) {
    if (error != nullptr) *error = "collision server must use 127.0.0.1:PORT";
    return false;
  }
  const std::string port_text = address.substr(prefix.size());
  char* end = nullptr;
  const long port = std::strtol(port_text.c_str(), &end, 10);
  if (end == port_text.c_str() || *end != '\0' || port < 1 || port > 65535) {
    if (error != nullptr) *error = "collision server port is invalid";
    return false;
  }
  const int fd = ::socket(AF_INET, SOCK_STREAM, 0);
  if (fd < 0) {
    if (error != nullptr) *error = std::strerror(errno);
    return false;
  }
  const auto close_fd = [&]() { (void)::close(fd); };
  const auto wait_for_socket = [&](bool writing) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(240);
    while (keep_running.load()) {
      fd_set ready;
      FD_ZERO(&ready);
      FD_SET(fd, &ready);
      timeval poll_timeout{0, 50000};
      const int selected = ::select(fd + 1, writing ? nullptr : &ready,
                                    writing ? &ready : nullptr, nullptr, &poll_timeout);
      if (selected > 0) return true;
      if (selected < 0 && errno != EINTR) return false;
      if (std::chrono::steady_clock::now() >= deadline) return false;
    }
    return false;
  };
  const int flags = ::fcntl(fd, F_GETFL, 0);
  if (flags < 0 || ::fcntl(fd, F_SETFL, flags | O_NONBLOCK) != 0) {
    close_fd();
    return false;
  }
  sockaddr_in target{};
  target.sin_family = AF_INET;
  target.sin_port = htons(static_cast<uint16_t>(port));
  target.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
  int connected = ::connect(fd, reinterpret_cast<sockaddr*>(&target), sizeof(target));
  if (connected != 0 && errno == EINPROGRESS && wait_for_socket(true)) {
    int socket_error = 0;
    socklen_t length = sizeof(socket_error);
    if (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &socket_error, &length) == 0) {
      errno = socket_error;
      connected = socket_error == 0 ? 0 : -1;
    }
  }
  if (connected != 0) {
    if (error != nullptr) *error = std::strerror(errno);
    close_fd();
    return false;
  }
  const auto send_all = [&](const uint8_t* data, std::size_t size) {
    std::size_t sent = 0;
    while (sent < size) {
      if (!wait_for_socket(true)) return false;
      const ssize_t count = ::send(fd, data + sent, size - sent, MSG_DONTWAIT);
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (count <= 0) return false;
      sent += static_cast<std::size_t>(count);
    }
    return true;
  };
  const auto receive_all = [&](uint8_t* data, std::size_t size) {
    std::size_t received = 0;
    while (received < size) {
      if (!wait_for_socket(false)) return false;
      const ssize_t count = ::recv(fd, data + received, size - received, MSG_DONTWAIT);
      if (count < 0 && errno == EINTR) continue;
      if (count < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) continue;
      if (count <= 0) return false;
      received += static_cast<std::size_t>(count);
    }
    return true;
  };
  uint8_t header[8]{};
  uint64_t request_size = static_cast<uint64_t>(request.size());
  for (int index = 7; index >= 0; --index) {
    header[index] = static_cast<uint8_t>(request_size & 0xffu);
    request_size >>= 8u;
  }
  if (!send_all(header, sizeof(header)) ||
      !send_all(reinterpret_cast<const uint8_t*>(request.data()), request.size()) ||
      !receive_all(header, sizeof(header))) {
    if (error != nullptr) *error = "collision server frame transfer failed";
    close_fd();
    return false;
  }
  uint64_t response_size = 0;
  for (uint8_t byte : header) response_size = (response_size << 8u) | byte;
  if (response_size == 0 || response_size > kMaxFrameBytes) {
    if (error != nullptr) *error = "collision server response exceeds frame bounds";
    close_fd();
    return false;
  }
  response->resize(static_cast<std::size_t>(response_size));
  const bool received = receive_all(
      reinterpret_cast<uint8_t*>(response->data()), response->size());
  close_fd();
  if (!received && error != nullptr) *error = "collision server response was incomplete";
  return received;
}

// Run the genuine tesseract_collision discrete contact check (Bullet BVH
// manager via the tesseract-robotics-nanobind worker) over every planned
// waypoint. Robot self-collision uses an adjacent-link ACM; static geometry
// from the URDF (floor, H-frame, positioner tables) is checked against the
// arm as well.
int run_collision_worker_cancellable(const std::string& command) {
  posix_spawnattr_t attributes;
  if (::posix_spawnattr_init(&attributes) != 0) return -1;
  const int configured = ::posix_spawnattr_setflags(&attributes, POSIX_SPAWN_SETPGROUP);
  const int group_configured = ::posix_spawnattr_setpgroup(&attributes, 0);
  if (configured != 0 || group_configured != 0) {
    ::posix_spawnattr_destroy(&attributes);
    return -1;
  }
  char* argv[] = {const_cast<char*>("sh"), const_cast<char*>("-c"),
                  const_cast<char*>(command.c_str()), nullptr};
  pid_t child = 0;
  const int spawned = ::posix_spawn(&child, "/bin/sh", nullptr, &attributes, argv, ::environ);
  ::posix_spawnattr_destroy(&attributes);
  if (spawned != 0) return -1;
  int status = 0;
  auto stopping_at = std::chrono::steady_clock::time_point{};
  for (;;) {
    const pid_t waited = ::waitpid(child, &status, WNOHANG);
    if (waited == child) {
      if (!keep_running.load()) (void)::kill(-child, SIGKILL);
      return status;
    }
    if (waited < 0 && errno != EINTR) return -1;
    if (!keep_running.load()) {
      const auto now = std::chrono::steady_clock::now();
      if (stopping_at == std::chrono::steady_clock::time_point{}) {
        stopping_at = now;
        (void)::kill(-child, SIGTERM);
      } else if (now - stopping_at >= std::chrono::milliseconds(500)) {
        (void)::kill(-child, SIGKILL);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}

void require_planning_running() {
  if (!keep_running.load()) throw std::runtime_error("planning cancelled by shutdown");
}

SpreadsheetCollisionCheck run_spreadsheet_plan_collision_check(
    const SpreadsheetMotionServerPlan& plan) {
  SpreadsheetCollisionCheck check;
  if (plan.trajectory.empty()) {
    check.state = "skipped_empty_trajectory";
    return check;
  }
  const std::string urdf_path = resolve_spreadsheet_planner_urdf_path();
  if (urdf_path.empty()) {
    check.state = "unavailable_urdf_not_found";
    check.error = std::string("cannot resolve ") + kMotionServerTesseractURDFSource;
    return check;
  }
  std::string robot_model;
  std::string mesh_package_root;
  if (!spreadsheet_collision_pair_identity(
          urdf_path, &robot_model, &mesh_package_root)) {
    check.state = "unavailable_pair_identity";
    check.error =
        "planner URDF does not resolve to a supported model and mesh package";
    return check;
  }
  std::string worker_command;
  std::string server_address;
  const char* server_override =
      std::getenv("ROBOT_V4_TESSERACT_COLLISION_SERVER");
  if (server_override != nullptr && server_override[0] != '\0') {
    server_address = server_override;
  }
  const char* worker_override = std::getenv("ROBOT_V4_TESSERACT_COLLISION_WORKER");
  if (!server_address.empty()) {
    // The persistent loopback server owns the already-loaded collision world.
  } else if (worker_override != nullptr && worker_override[0] != '\0') {
    worker_command = worker_override;
  } else {
    const std::string python = resolve_repo_relative_file(kSpreadsheetCollisionWorkerPython);
    const std::string script = resolve_repo_relative_file(kSpreadsheetCollisionWorkerScript);
    if (python.empty() || script.empty()) {
      check.state = "unavailable_worker_not_provisioned";
      check.error = std::string("missing ") + kSpreadsheetCollisionWorkerPython +
                    " (run `pixi install` under tesseract/v1) or " +
                    kSpreadsheetCollisionWorkerScript;
      return check;
    }
    worker_command = shell_quote_path(python) + " " + shell_quote_path(script);
  }

  std::ostringstream request;
  request << "{\"schema\":\"robot-v4.tesseract-collision-check.v1\","
          << "\"urdf_path\":\"" << json_escape(urdf_path) << "\","
          << "\"robot_model\":\"" << json_escape(robot_model) << "\","
          << "\"mesh_package_root\":\""
          << json_escape(mesh_package_root) << "\","
          << "\"contact_distance_m\":0.0,"
          << "\"joint_names\":[";
  const std::size_t collision_axis_count =
      robot_model == "rosie_1420_v1" ? 6u : kLiveAxisCount;
  for (std::size_t axis = 0; axis < collision_axis_count; ++axis) {
    if (axis > 0) {
      request << ",";
    }
    request << "\"" << spreadsheet_planner_joint_name(axis) << "\"";
  }
  request << "],\"waypoints\":[";
  for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
    require_planning_running();
    const SpreadsheetPlanPoint& point = plan.trajectory[index];
    if (index > 0) {
      request << ",";
    }
    request << "{\"waypoint_index\":" << index
            << ",\"time_s\":" << std::fixed << std::setprecision(6) << point.time_s
            << ",\"positions_rad\":"
            << double_array_json_fixed(
                   point.joints_rad,
                   static_cast<int>(collision_axis_count))
            << "}";
  }
  request << "]}";

  const std::string temp_base =
      "/tmp/robot-v4-collision-check-" + std::to_string(static_cast<long>(::getpid()));
  const std::string input_path = temp_base + "-input.json";
  const std::string output_path = temp_base + "-output.json";
  const std::string log_path = temp_base + "-worker.log";
  const auto started_at = std::chrono::steady_clock::now();
  int exit_code = 0;
  std::string raw;
  if (!server_address.empty()) {
    std::string server_error;
    if (!spreadsheet_collision_server_request(
            server_address, request.str(), &raw, &server_error)) {
      check.state = "failed_persistent_worker_request";
      check.error = server_error;
      return check;
    }
  } else {
    if (!write_text_file(input_path, request.str())) {
      check.state = "failed_request_write";
      check.error = "cannot write " + input_path;
      return check;
    }
    const std::string command = worker_command + " --input " + shell_quote_path(input_path) +
                                " --output " + shell_quote_path(output_path) + " >" +
                                shell_quote_path(log_path) + " 2>&1";
    exit_code = run_collision_worker_cancellable(command);
    if (!keep_running.load()) {
      check.state = "cancelled_shutdown";
      return check;
    }
    if (::access(output_path.c_str(), R_OK) != 0 || !read_text_file(output_path, &raw)) {
      check.state = "failed_worker_no_output";
      check.error = "worker exit " + std::to_string(exit_code) + ", no output; see " + log_path;
      (void)::unlink(input_path.c_str());
      return check;
    }
  }
  const double wall_s =
      std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
  bool worker_ok = false;
  (void)parse_json_bool(raw, "ok", &worker_ok);
  if (!worker_ok) {
    std::string worker_error;
    (void)parse_json_string(raw, "error", &worker_error);
    check.state = "failed_worker_error";
    check.error = worker_error.empty() ? ("worker exit " + std::to_string(exit_code)) : worker_error;
    (void)::unlink(input_path.c_str());
    (void)::unlink(output_path.c_str());
    return check;
  }
  (void)parse_json_string(raw, "checker", &check.checker);
  (void)parse_json_string(raw, "tesseract_nanobind_version", &check.library_version);
  (void)parse_json_int(raw, "checked_waypoint_count", &check.checked_waypoint_count);
  (void)parse_json_int(raw, "cache_hit_waypoint_count", &check.cache_hit_waypoint_count);
  check.worker_wall_time_s = wall_s;
  if (!parse_json_double(raw, "check_duration_s", &check.check_duration_s)) {
    check.check_duration_s = wall_s;
  }
  for (const std::string& contact_object : json_object_slices_for_array_key(raw, "contacts")) {
    SpreadsheetPlanCollisionPoint contact;
    (void)parse_json_int(contact_object, "waypoint_index", &contact.waypoint_index);
    (void)parse_json_double(contact_object, "time_s", &contact.time_s);
    (void)parse_json_string(contact_object, "link_a", &contact.link_a);
    (void)parse_json_string(contact_object, "link_b", &contact.link_b);
    contact.affected_joints =
        parse_json_string_array(contact_object, "affected_joints");
    (void)parse_json_double(contact_object, "depth_m", &contact.depth_m);
    if (contact.waypoint_index >= 0 &&
        contact.waypoint_index < static_cast<int>(plan.trajectory.size())) {
      const SpreadsheetPlanPoint& point =
          plan.trajectory[static_cast<std::size_t>(contact.waypoint_index)];
      contact.row_index = point.row_index;
      contact.time_s = point.time_s;
      std::copy(point.tcp_pose, point.tcp_pose + 6, contact.tcp_pose);
    }
    check.contacts.push_back(contact);
  }
  check.ran = true;
  check.state = "tesseract_discrete_contact_checked";
  (void)::unlink(input_path.c_str());
  (void)::unlink(output_path.c_str());
  (void)::unlink(log_path.c_str());
  return check;
}

SpreadsheetPlanPoint interpolate_spreadsheet_plan_point(const SpreadsheetPlanRow& from,
                                                        const SpreadsheetPlanRow& to,
                                                        int row_index,
                                                        double time_s,
                                                        double alpha) {
  SpreadsheetPlanPoint point;
  point.row_index = row_index;
  point.time_s = time_s;
  point.move_type = to.move_type;
  point.motion_phase = to.motion_phase;
  point.calibration_section = to.calibration_section;
  point.tool_axis_constraint_active = spreadsheet_row_is_line_tcp(to);
  point.harmonious_workplane_active = spreadsheet_row_is_line_tcp(to);
  // Translation interpolates componentwise, exactly as before: x/y/z are a
  // genuine vector space, so the straight line between row targets IS the
  // intended path. The expression is kept character-for-character so every
  // sampled position stays bit-identical to the previous planner.
  for (int axis = 0; axis < 3; ++axis) {
    point.tcp_pose[axis] = from.tcp_pose[axis] + (to.tcp_pose[axis] - from.tcp_pose[axis]) * alpha;
  }
  // Orientation does NOT interpolate componentwise. Row RPY is URDF fixed-axis
  // Rz(yaw)*Ry(pitch)*Rx(roll); lerping those three angles independently leaves
  // the SO(3) geodesic and swings the torch off the authored path by up to
  // 21.22 deg mid-segment on a true 177.33 deg inter-seam reorientation (measured
  // Dog Box pair 24; see tests/SO3_INTERP_NOTES.md). Every interpolated sample
  // here is a hard IK target, so that excursion is real torch motion. Walk the
  // shortest-arc geodesic in SO(3) instead.
  robot_v4::motion_server::so3_interpolate_rpy(
      &from.tcp_pose[3], &to.tcp_pose[3], alpha, &point.tcp_pose[3]);
  for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
    point.joints_rad[axis] = from.joints_rad[axis] + (to.joints_rad[axis] - from.joints_rad[axis]) * alpha;
  }
  return point;
}

SpreadsheetMotionServerPlan plan_motion_server_tesseract_spreadsheet_path(
    const std::vector<SpreadsheetPlanRow>& program_rows,
    int samples_per_segment,
    const SpreadsheetToolAxisConstraint& constraint,
    const SpreadsheetHarmoniousAxisPolicy& harmonious_axis_policy,
    const std::array<double, kLiveAxisCount>& joint_speed_limits_rad_s,
    double singularity_warning_threshold,
    double singularity_hard_stop_threshold) {
  SpreadsheetMotionServerPlan plan;
  plan.tool_axis_constraint = constraint;
  plan.harmonious_axis_policy = harmonious_axis_policy;
  plan.joint_speed_limits_rad_s = joint_speed_limits_rad_s;
  plan.singularity_warning_threshold = singularity_warning_threshold;
  plan.singularity_hard_stop_threshold = singularity_hard_stop_threshold;
  plan.loop_command = validate_spreadsheet_loop_command(program_rows);
  std::vector<SpreadsheetPlanRow> rows;
  rows.reserve(program_rows.size());
  for (const SpreadsheetPlanRow& row : program_rows) {
    if (spreadsheet_row_is_motion(row)) {
      rows.push_back(row);
    }
  }
  plan.motion_row_count = static_cast<int>(rows.size());
  if (rows.size() < 2) {
    return plan;
  }
  for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
    if (!spreadsheet_segment_velocity_timing_valid(
            rows[row_index - 1], rows[row_index])) {
      return plan;
    }
  }
  std::set<int> calibration_sections;
  for (const SpreadsheetPlanRow& row : rows) {
    if (row.calibration_section > 0) {
      calibration_sections.insert(row.calibration_section);
    }
    if (row.motion_phase == "reconfigure") {
      plan.reconfiguration_row_count++;
    }
  }
  plan.calibration_section_count = static_cast<int>(calibration_sections.size());
  for (const SpreadsheetPlanRow& row : rows) {
    if (spreadsheet_row_is_joint_ptp(row) &&
        spreadsheet_joints_are_policy_home(row.joints_rad,
                                           harmonious_axis_policy)) {
      plan.home_row_count++;
    }
  }
  plan.starts_at_home = spreadsheet_row_is_joint_ptp(rows.front()) &&
                        spreadsheet_joints_are_policy_home(
                            rows.front().joints_rad, harmonious_axis_policy);
  plan.returns_home = spreadsheet_row_is_joint_ptp(rows.back()) &&
                      spreadsheet_joints_are_policy_home(
                          rows.back().joints_rad, harmonious_axis_policy);
  samples_per_segment = std::max(1, samples_per_segment);
  const SpreadsheetJointLimits limits = load_spreadsheet_joint_limits();
  plan.joint_limits_checked = limits.loaded;
  plan.joint_limits_source = limits.source;
  plan.tool_axis_constraint_checked = constraint.present;
  plan.tool_axis_constraint_satisfied = !constraint.present;
  plan.min_measured_tool_axis_offset_deg = std::numeric_limits<double>::infinity();
  plan.max_measured_tool_axis_offset_deg = -std::numeric_limits<double>::infinity();
  if (plan.harmonious_axis_policy.present && plan.harmonious_axis_policy.valid) {
    for (const SpreadsheetPlanRow& row : rows) {
      if (row.fixture_side != plan.harmonious_axis_policy.active_side ||
          row.work_frame != plan.harmonious_axis_policy.work_frame ||
          std::abs(row.joints_rad[8] - plan.harmonious_axis_policy.selector_rad) >
              plan.harmonious_axis_policy.selector_tolerance_rad) {
        plan.harmonious_axis_policy.valid = false;
        plan.harmonious_axis_policy.validation_error =
            "row_active_side_workplane_mismatch";
        break;
      }
    }
  }
  const bool harmonious_enabled = plan.harmonious_axis_policy.present &&
                                  plan.harmonious_axis_policy.valid;
  const bool harmonious_from_row_targets =
      harmonious_enabled &&
      plan.harmonious_axis_policy.trajectory_source == "row_targets";
  const bool harmonious_per_calibration_section =
      harmonious_enabled &&
      plan.harmonious_axis_policy.trajectory_source == "section_sinusoidal";
  const std::size_t harmonious_axis_index =
      harmonious_axis_policy.joint == "J8" ? 7u : 6u;
  if (harmonious_enabled) {
    plan.harmonious_axis_applied = true;
    plan.harmonious_axis_min_rad = std::numeric_limits<double>::infinity();
    plan.harmonious_axis_max_rad = -std::numeric_limits<double>::infinity();
  }
  // The active J7/J8 curve is parameterized by normalized path progress, not by the
  // final retimed clock. This keeps the same table pose attached to each TCP
  // sample even when joint-speed limits stretch individual sample intervals.
  int maximum_calibration_section = 0;
  for (const SpreadsheetPlanRow& row : rows) {
    maximum_calibration_section =
        std::max(maximum_calibration_section, row.calibration_section);
  }
  std::vector<double> section_start_s(
      static_cast<std::size_t>(maximum_calibration_section + 1),
      std::numeric_limits<double>::infinity());
  std::vector<double> section_end_s(
      static_cast<std::size_t>(maximum_calibration_section + 1), 0.0);
  std::vector<double> section_phase_deg(
      static_cast<std::size_t>(maximum_calibration_section + 1), 0.0);
  std::vector<bool> section_phase_set(
      static_cast<std::size_t>(maximum_calibration_section + 1), false);
  double nominal_duration_s = 0.0;
  for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
    const SpreadsheetPlanRow& from = rows[row_index - 1];
    const SpreadsheetPlanRow& to = rows[row_index];
    const double segment_s = spreadsheet_segment_duration_s(from, to);
    if (spreadsheet_row_is_line_tcp(to) && to.calibration_section > 0 &&
        to.calibration_section <= maximum_calibration_section) {
      const std::size_t section = static_cast<std::size_t>(to.calibration_section);
      section_start_s[section] = std::min(section_start_s[section], nominal_duration_s);
      section_end_s[section] = std::max(section_end_s[section], nominal_duration_s + segment_s);
      if (!section_phase_set[section]) {
        section_phase_deg[section] = to.harmonious_phase_deg;
        section_phase_set[section] = true;
      } else if (std::abs(section_phase_deg[section] - to.harmonious_phase_deg) > 1.0e-9) {
        plan.harmonious_axis_policy.valid = false;
        plan.harmonious_axis_policy.validation_error =
            "section_harmonious_phase_mismatch";
      }
    } else if (spreadsheet_row_is_joint_ptp(to) &&
               to.motion_phase == "section_settle" &&
               to.calibration_section > 0 &&
               to.calibration_section <= maximum_calibration_section) {
      // Preserve the qualified section waveform through the explicit J7/J8
      // zero-settle row. The joint-space settle itself is still constraint-
      // free and collision checked; only its duration completes the section
      // phase used by the preceding Cartesian samples.
      const std::size_t section = static_cast<std::size_t>(to.calibration_section);
      section_end_s[section] =
          std::max(section_end_s[section], nominal_duration_s + segment_s);
    }
    nominal_duration_s += segment_s;
  }
  nominal_duration_s +=
      std::max(0.001, static_cast<double>(std::max(1, rows.back().playback_step_ms)) / 1000.0);
  // Weld timeline: a row's weld edge fires at row start, i.e. when the
  // trajectory departs that row's position, and the arc state holds until the
  // next explicit edge (STEAMDECK_REVPI_IO timing "row_start").
  bool weld_state = false;
  const auto apply_weld_edge = [&weld_state](const SpreadsheetPlanRow& row) {
    const int edge = spreadsheet_row_weld_edge(row);
    if (edge >= 0) {
      weld_state = edge == 1;
    }
  };
  double previous_tcp[6]{};
  double previous_joints[kLiveAxisCount]{};
  // Evaluate every sampled point even after a failure so the report carries a
  // complete per-waypoint reachability map. Unreachable samples hold the last
  // solved joints (the plan stays unplayable; validation goes invalid).
  const auto record_unreachable = [&](const SpreadsheetPlanPoint& point,
                                      const std::string& reason,
                                      const std::string& joint,
                                      double joint_value_rad) {
    SpreadsheetPlanUnreachablePoint unreachable;
    unreachable.row_index = point.row_index;
    unreachable.waypoint_index = static_cast<int>(plan.trajectory.size());
    unreachable.time_s = point.time_s;
    std::copy(point.tcp_pose, point.tcp_pose + 6, unreachable.tcp_pose);
    unreachable.reason = reason;
    unreachable.joint = joint;
    unreachable.joint_value_rad = joint_value_rad;
    plan.unreachable_points.push_back(unreachable);
  };
  const auto check_joint_limits = [&](const SpreadsheetPlanPoint& point) {
    if (!limits.loaded) {
      return;
    }
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      const double value = point.joints_rad[axis];
      if (value < limits.lower_rad[axis] || value > limits.upper_rad[axis]) {
        record_unreachable(point, "joint_limit", spreadsheet_planner_joint_name(axis), value);
        return;
      }
    }
  };
  const auto solve_point = [&](SpreadsheetPlanPoint* point) {
    if (point->move_type == "joint_ptp") {
      point->tool_axis_constraint_active = false;
      point->harmonious_workplane_active = false;
      spreadsheet_fk_pose_from_joints(point->joints_rad, point->tcp_pose);
      if (harmonious_enabled) {
        const double sample_positioner_rad = point->joints_rad[harmonious_axis_index];
        plan.harmonious_axis_min_rad =
            std::min(plan.harmonious_axis_min_rad, sample_positioner_rad);
        plan.harmonious_axis_max_rad =
            std::max(plan.harmonious_axis_max_rad, sample_positioner_rad);
      }
      const std::size_t unreachable_before = plan.unreachable_points.size();
      check_joint_limits(*point);
      return plan.unreachable_points.size() == unreachable_before;
    }
    point->tool_axis_constraint_active = constraint.present;
    point->harmonious_workplane_active = harmonious_enabled;
    double sample_positioner_rad = previous_joints[harmonious_axis_index];
    double reference_tcp[3] = {point->tcp_pose[0], point->tcp_pose[1], point->tcp_pose[2]};
    SpreadsheetToolAxisConstraint sample_constraint = constraint;
    if (harmonious_enabled) {
      if (harmonious_per_calibration_section && point->calibration_section > 0 &&
          point->calibration_section <= maximum_calibration_section) {
        const std::size_t section =
            static_cast<std::size_t>(point->calibration_section);
        const double section_duration_s = section_end_s[section] - section_start_s[section];
        const double progress = section_duration_s > 0.0
                                    ? (point->time_s - section_start_s[section]) /
                                          section_duration_s
                                    : 0.0;
        SpreadsheetHarmoniousAxisPolicy section_policy = harmonious_axis_policy;
        section_policy.phase_deg = section_phase_deg[section];
        sample_positioner_rad =
            harmonious_axis_position_rad(section_policy, progress);
      } else if (harmonious_from_row_targets) {
        sample_positioner_rad = point->joints_rad[harmonious_axis_index];
      } else {
        const double progress = nominal_duration_s > 0.0 ? point->time_s / nominal_duration_s : 0.0;
        sample_positioner_rad = harmonious_axis_position_rad(harmonious_axis_policy, progress);
      }
      apply_harmonious_workplane_transform(harmonious_axis_policy, sample_positioner_rad,
                                           point->tcp_pose);
      sample_constraint = harmonious_sample_tool_constraint(
          constraint, harmonious_axis_policy, sample_positioner_rad);
      plan.harmonious_axis_min_rad =
          std::min(plan.harmonious_axis_min_rad, sample_positioner_rad);
      plan.harmonious_axis_max_rad =
          std::max(plan.harmonious_axis_max_rad, sample_positioner_rad);
      if (harmonious_axis_policy.workplane_relative) {
        double reconstructed[6] = {point->tcp_pose[0], point->tcp_pose[1], point->tcp_pose[2], 0, 0, 0};
        SpreadsheetHarmoniousAxisPolicy inverse = harmonious_axis_policy;
        inverse.reference_rad = sample_positioner_rad;
        apply_harmonious_workplane_transform(inverse,
                                             harmonious_axis_policy.reference_rad,
                                             reconstructed);
        const double dx = reconstructed[0] - reference_tcp[0];
        const double dy = reconstructed[1] - reference_tcp[1];
        const double dz = reconstructed[2] - reference_tcp[2];
        plan.max_workplane_relative_error_m = std::max(
            plan.max_workplane_relative_error_m,
            std::sqrt(dx * dx + dy * dy + dz * dz));
      }
    }
    if (constraint.present && !constraint.valid) {
      record_unreachable(*point, "tool_axis_constraint_invalid", "", 0.0);
      std::copy(previous_joints, previous_joints + kLiveAxisCount, point->joints_rad);
      if (harmonious_enabled) {
        point->joints_rad[harmonious_axis_index] = sample_positioner_rad;
        point->joints_rad[8] = harmonious_axis_policy.selector_rad;
      }
      return false;
    }
    const SpreadsheetToolAxisConstraint* active_constraint =
        constraint.present ? &sample_constraint : nullptr;
    double sample_seed[kLiveAxisCount]{};
    std::copy(previous_joints, previous_joints + kLiveAxisCount, sample_seed);
    if (harmonious_enabled) {
      sample_seed[harmonious_axis_index] = sample_positioner_rad;
      sample_seed[8] = harmonious_axis_policy.selector_rad;
    }
    if (solve_spreadsheet_tcp_position_ik(
            point->tcp_pose, sample_seed, active_constraint, point->joints_rad)) {
      // The arm IK owns J1..J6 only. Preserve the selected positioner and
      // J9 face selector as an inseparable part of every nine-axis sample.
      if (harmonious_enabled) {
        point->joints_rad[harmonious_axis_index] = sample_positioner_rad;
        point->joints_rad[8] = harmonious_axis_policy.selector_rad;
      }
      // IK Euler extraction can return the same physical revolute pose on
      // opposite sides of +/-pi. Keep each solved sample on the nearest
      // equivalent branch that remains inside the URDF limit; otherwise a
      // small Cartesian step becomes a false 2*pi joint revolution that the
      // speed retimer and playback executor must crawl through.
      if (limits.loaded) {
        for (std::size_t axis = 0; axis < 6; ++axis) {
          bool wrapped = false;
          point->joints_rad[axis] = nearest_equivalent_joint_angle_within_limits(
              point->joints_rad[axis], previous_joints[axis],
              limits.lower_rad[axis], limits.upper_rad[axis], &wrapped);
          if (wrapped) {
            plan.joint_angle_wraps_resolved++;
          }
        }
      }
      plan.max_tcp_fk_error_m =
          std::max(plan.max_tcp_fk_error_m,
                   spreadsheet_plan_tcp_fk_error_m(point->tcp_pose, point->joints_rad));
      check_joint_limits(*point);
      if (constraint.present) {
        double q[6]{};
        for (int axis = 0; axis < 6; ++axis) {
          q[axis] = point->joints_rad[axis];
        }
        const std::array<double, 3> actual_axis = normalized3(
            rotate_direction(urdf_forward_kinematics_j1_base(q), sample_constraint.tool_axis_xyz));
        const double measured_offset_deg =
            spreadsheet_constraint_measured_offset_deg(sample_constraint, actual_axis);
        plan.min_measured_tool_axis_offset_deg =
            std::min(plan.min_measured_tool_axis_offset_deg, measured_offset_deg);
        plan.max_measured_tool_axis_offset_deg =
            std::max(plan.max_measured_tool_axis_offset_deg, measured_offset_deg);
        plan.max_tool_axis_error_deg = std::max(
            plan.max_tool_axis_error_deg,
            spreadsheet_constraint_axis_error_deg(sample_constraint, actual_axis));
        const bool sample_satisfied =
            spreadsheet_constraint_sample_satisfied(sample_constraint, actual_axis);
        plan.tool_axis_constraint_satisfied =
            plan.tool_axis_constraint_satisfied || sample_satisfied;
        if (!sample_satisfied) {
          record_unreachable(*point, "tool_axis_constraint", "", 0.0);
          return false;
        }
      }
      return true;
    }
    record_unreachable(*point, "ik_no_solution", "", 0.0);
    std::copy(previous_joints, previous_joints + kLiveAxisCount, point->joints_rad);
    if (harmonious_enabled) {
      point->joints_rad[harmonious_axis_index] = sample_positioner_rad;
      point->joints_rad[8] = harmonious_axis_policy.selector_rad;
    }
    return false;
  };

  SpreadsheetPlanPoint first;
  first.row_index = rows.front().row_index;
  first.move_type = rows.front().move_type;
  first.motion_phase = rows.front().motion_phase;
  first.calibration_section = rows.front().calibration_section;
  stamp_spreadsheet_path_metadata(&first, rows.front());
  first.tool_axis_constraint_active = spreadsheet_row_is_line_tcp(rows.front());
  first.harmonious_workplane_active = spreadsheet_row_is_line_tcp(rows.front());
  apply_weld_edge(rows.front());
  first.weld_on = weld_state;
  std::copy(rows.front().tcp_pose, rows.front().tcp_pose + 6, first.tcp_pose);
  std::copy(rows.front().joints_rad,
            rows.front().joints_rad + kLiveAxisCount,
            first.joints_rad);
  std::copy(rows.front().joints_rad, rows.front().joints_rad + kLiveAxisCount, previous_joints);
  (void)solve_point(&first);
  plan.trajectory.push_back(first);
  double time_s = 0.0;
  std::copy(first.tcp_pose, first.tcp_pose + 6, previous_tcp);
  std::copy(first.joints_rad, first.joints_rad + kLiveAxisCount, previous_joints);

  auto append_segment = [&](const SpreadsheetPlanRow& from,
                            const SpreadsheetPlanRow& to,
                            double segment_s) {
    apply_weld_edge(from);
    SpreadsheetPlanRow interpolation_from = from;
    if (spreadsheet_row_is_joint_ptp(to)) {
      std::copy(previous_joints, previous_joints + kLiveAxisCount,
                interpolation_from.joints_rad);
      std::copy(previous_tcp, previous_tcp + 6, interpolation_from.tcp_pose);
    }
    constexpr double kMaximumJointPTPSampleStepRad = 0.05;
    constexpr double kMaximumLineTCPSampleStepM = 0.05;
    int segment_samples = samples_per_segment;
    if (spreadsheet_row_is_joint_ptp(to)) {
      segment_samples = std::max(
          segment_samples,
          static_cast<int>(std::ceil(
              max_joint_delta_rad(previous_joints, to.joints_rad) /
              kMaximumJointPTPSampleStepRad)));
    } else {
      segment_samples = std::max(
          segment_samples,
          static_cast<int>(std::ceil(tcp_distance_m(from.tcp_pose, to.tcp_pose) /
                                     kMaximumLineTCPSampleStepM)));
      if (to.calibration_section > 0) {
        // Keep the rolled wrist branch below the 0.25 rad continuity gate
        // while retaining dense collision/FK evidence along every trace.
        segment_samples = std::max(segment_samples, 32);
      }
    }
    segment_samples = std::clamp(segment_samples, 1, 512);
    for (int sample = 1; sample <= segment_samples; ++sample) {
      require_planning_running();
      const double alpha = static_cast<double>(sample) / static_cast<double>(segment_samples);
      const double sample_time_s = time_s + segment_s * alpha;
      const int sample_row_index = sample == segment_samples ? to.row_index : from.row_index;
      SpreadsheetPlanPoint point =
          interpolate_spreadsheet_plan_point(
              interpolation_from, to, sample_row_index, sample_time_s, alpha);
      // The interpolated stroke approaches `to`; its frame owner must match
      // the target row for every dense sample, not switch only at the final
      // endpoint. Cross-fixture programs therefore require an explicit
      // h_frame reconfiguration row between Table A and Table B strokes.
      stamp_spreadsheet_path_metadata(&point, to);
      point.weld_on = weld_state;
      const bool solved = solve_point(&point);
      const double tcp_step_m = tcp_distance_m(previous_tcp, point.tcp_pose);
      plan.tool_path_length_m += tcp_step_m;
      if (solved) {
        const double joint_step_rad = max_joint_delta_rad(previous_joints, point.joints_rad);
        plan.max_tcp_step_m = std::max(plan.max_tcp_step_m, tcp_step_m);
        plan.max_joint_step_rad = std::max(plan.max_joint_step_rad, joint_step_rad);
        if (point.move_type == "joint_ptp") {
          plan.max_joint_ptp_step_rad =
              std::max(plan.max_joint_ptp_step_rad, joint_step_rad);
        } else {
          plan.max_line_tcp_joint_step_rad =
              std::max(plan.max_line_tcp_joint_step_rad, joint_step_rad);
        }
      }
      std::copy(point.tcp_pose, point.tcp_pose + 6, previous_tcp);
      std::copy(point.joints_rad, point.joints_rad + kLiveAxisCount, previous_joints);
      plan.trajectory.push_back(point);
    }
    time_s += segment_s;
  };

  for (std::size_t row_index = 1; row_index < rows.size(); ++row_index) {
    const SpreadsheetPlanRow& from = rows[row_index - 1];
    const SpreadsheetPlanRow& to = rows[row_index];
    const double segment_s = spreadsheet_segment_duration_s(from, to);
    append_segment(from, to, segment_s);
  }
  const SpreadsheetPlanRow& last = rows.back();
  const SpreadsheetPlanRow& first_row = rows.front();
  const double final_segment_s =
      std::max(0.001, static_cast<double>(std::max(1, last.playback_step_ms)) / 1000.0);
  if (plan.starts_at_home && plan.returns_home &&
      max_joint_delta_rad(last.joints_rad, first_row.joints_rad) <= 1.0e-9) {
    // A segmented program reaches the explicit ReturnHome row before LOOP.
    // Do not append an invisible closure seek or hold: the last planned
    // sample is already the exact all-axis loop boundary.
  } else if (plan.loop_command.present && plan.loop_command.valid &&
             tcp_distance_m(last.tcp_pose, first_row.tcp_pose) <= 1.0e-6) {
    // The final zero-distance segment returns the harmonious positioner to
    // the exact start branch while remaining semantically owned by the
    // explicit ReturnHome row (not by the LOOP target row).
    SpreadsheetPlanRow home_target = first_row;
    home_target.row_index = last.row_index;
    append_segment(last, home_target, final_segment_s);
    // Close the pass on the exact first solved joint branch, not merely an
    // IK-equivalent TCP pose. This makes the LOOP boundary a true home
    // boundary for every commanded axis (including J7), so resetting the
    // plan clock cannot introduce an implicit seek move.
    if (!plan.trajectory.empty()) {
      std::copy(first.joints_rad, first.joints_rad + kLiveAxisCount,
                plan.trajectory.back().joints_rad);
    }
  } else {
    time_s += final_segment_s;
    SpreadsheetPlanPoint hold = plan.trajectory.back();
    hold.row_index = last.row_index;
    hold.time_s = time_s;
    apply_weld_edge(last);
    hold.weld_on = weld_state;
    plan.trajectory.push_back(hold);
  }
  plan.duration_s = time_s;
  if (constraint.present && constraint.valid && !plan.trajectory.empty()) {
    plan.tool_axis_constraint_satisfied = true;
    for (const SpreadsheetPlanUnreachablePoint& point : plan.unreachable_points) {
      if (point.reason == "tool_axis_constraint" ||
          point.reason == "tool_axis_constraint_invalid") {
        plan.tool_axis_constraint_satisfied = false;
      }
    }
  }
  if (!std::isfinite(plan.min_measured_tool_axis_offset_deg)) {
    plan.min_measured_tool_axis_offset_deg = 0.0;
    plan.max_measured_tool_axis_offset_deg = 0.0;
  }
  if (plan.trajectory.size() >= 2) {
    std::vector<double> original_times_s;
    original_times_s.reserve(plan.trajectory.size());
    for (const SpreadsheetPlanPoint& point : plan.trajectory) {
      original_times_s.push_back(point.time_s);
    }
    double retimed_s = 0.0;
    plan.trajectory.front().time_s = 0.0;
    plan.max_joint_speeds_rad_s.fill(0.0);
    plan.joint_speed_limits_applied = true;
    plan.singularity_analysis_available = true;
    plan.singularity_hard_stop_clear = true;
    plan.minimum_singularity_score = 1.0;
    plan.minimum_applied_velocity_scale = 1.0;
    for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
      SpreadsheetPlanPoint& point = plan.trajectory[index];
      double arm_joints[6]{};
      for (int axis = 0; axis < 6; ++axis) {
        arm_joints[axis] = point.joints_rad[axis];
      }
      const RobotV4SingularityMetrics metrics =
          robot_v4_singularity_metrics(finite_difference_jacobian(arm_joints));
      for (int axis = 0; axis < 6; ++axis) {
        point.jacobian_singular_values[axis] = metrics.singular_values[axis];
      }
      point.jacobian_condition_number =
          std::isfinite(metrics.condition_number) ? metrics.condition_number : 0.0;
      point.jacobian_manipulability = metrics.manipulability;
      point.singularity_score = metrics.score;
      point.singular_direction_joint_index = metrics.singular_direction_joint_index;
      if (point.move_type == "joint_ptp") {
        // Cartesian Jacobian singularity is not an execution gate for an
        // explicitly authored joint-space reconfiguration. Joint limits,
        // speed limits, and Tesseract collision checks remain mandatory.
        point.singularity_scale = 1.0;
        point.applied_velocity_scale = 1.0;
        point.singularity_class = "not_applicable_joint_ptp";
        continue;
      }
      point.singularity_scale = robot_v4_singularity_velocity_scale(
          metrics, singularity_warning_threshold, singularity_hard_stop_threshold);
      if (index == 0) {
        point.applied_velocity_scale = point.singularity_scale;
      }
      if (!metrics.valid) {
        point.singularity_class = "unavailable";
        plan.singularity_analysis_available = false;
        plan.singularity_hard_stop_clear = false;
      } else if (metrics.score <= singularity_hard_stop_threshold) {
        point.singularity_class = "hard_stop";
        plan.singularity_hard_stop_point_count++;
        plan.singularity_hard_stop_clear = false;
      } else if (metrics.score < singularity_warning_threshold) {
        point.singularity_class = "warning";
        plan.singularity_warning_point_count++;
      } else {
        point.singularity_class = "clear";
      }
      plan.minimum_singularity_score =
          std::min(plan.minimum_singularity_score, metrics.score);
      plan.minimum_applied_velocity_scale =
          std::min(plan.minimum_applied_velocity_scale, point.applied_velocity_scale);
    }
    for (std::size_t index = 1; index < plan.trajectory.size(); ++index) {
      SpreadsheetPlanPoint& point = plan.trajectory[index];
      const SpreadsheetPlanPoint& previous = plan.trajectory[index - 1];
      const double requested_dt_s =
          std::max(0.001, original_times_s[index] - original_times_s[index - 1]);
      for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
        point.requested_velocities_rad_s[axis] =
            (point.joints_rad[axis] - previous.joints_rad[axis]) / requested_dt_s;
      }
      point.joint_limit_scale = robot_v4_common_joint_limit_scale(
          point.requested_velocities_rad_s, joint_speed_limits_rad_s.data(),
          kLiveAxisCount, &point.limiting_joint_index);
      if (point.joint_limit_scale <= 0.0) {
        plan.joint_speed_limits_applied = false;
      }
      point.applied_velocity_scale =
          std::min(point.joint_limit_scale, point.singularity_scale);
      plan.minimum_applied_velocity_scale =
          std::min(plan.minimum_applied_velocity_scale, point.applied_velocity_scale);
      const double required_dt_s = point.applied_velocity_scale > 0.0
                                       ? requested_dt_s / point.applied_velocity_scale
                                       : requested_dt_s;
      retimed_s += required_dt_s;
      point.time_s = retimed_s;
      for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
        point.scaled_velocities_rad_s[axis] =
            point.requested_velocities_rad_s[axis] * point.applied_velocity_scale;
        plan.max_joint_speeds_rad_s[axis] = std::max(
            plan.max_joint_speeds_rad_s[axis],
            std::abs(point.scaled_velocities_rad_s[axis]));
      }
    }
    plan.duration_s = retimed_s;
    for (SpreadsheetPlanUnreachablePoint& point : plan.unreachable_points) {
      if (point.waypoint_index >= 0 &&
          point.waypoint_index < static_cast<int>(plan.trajectory.size())) {
        point.time_s = plan.trajectory[static_cast<std::size_t>(point.waypoint_index)].time_s;
      }
    }
  }
  plan.planned = plan.trajectory.size() >= 2;
  return plan;
}

// Table-corner work-frame calibration (fit + URDF overlay + record) for the
// Steam Deck calibrate chord; see the file header for the honest scope.
#include "robot_v4_table_calibration.hpp"

// True when the plan is fully proven: every waypoint IK-reachable, inside the
// URDF joint limits, and cleared by the Tesseract discrete contact check.
bool rosie_1420_calibration_box_axis_bounds_valid(
    const SpreadsheetMotionServerPlan& plan) {
  if (plan.trajectory.empty()) {
    return false;
  }
  std::array<double, 6> minimum{};
  std::array<double, 6> maximum{};
  for (std::size_t axis = 0; axis < minimum.size(); ++axis) {
    minimum[axis] = std::numeric_limits<double>::infinity();
    maximum[axis] = -std::numeric_limits<double>::infinity();
  }
  for (const SpreadsheetPlanPoint& point : plan.trajectory) {
    for (std::size_t axis = 0; axis < minimum.size(); ++axis) {
      const double value = point.joints_rad[axis];
      if (!std::isfinite(value) ||
          std::abs(value) >
              kRosie1420CalibrationBoxMaxHomeJointDeltaRad) {
        return false;
      }
      minimum[axis] = std::min(minimum[axis], value);
      maximum[axis] = std::max(maximum[axis], value);
    }
    for (std::size_t axis = 6u; axis < kLiveAxisCount; ++axis) {
      if (!std::isfinite(point.joints_rad[axis]) ||
          std::abs(point.joints_rad[axis]) > 1.0e-9) {
        return false;
      }
    }
  }
  for (std::size_t axis = 0; axis < minimum.size(); ++axis) {
    if (maximum[axis] - minimum[axis] <= 1.0e-5) {
      return false;
    }
  }
  return true;
}

bool spreadsheet_program_structure_valid(const SpreadsheetMotionServerPlan& plan) {
  if (is_rosie_1420_calibration_box_program_id(plan.program_id)) {
    return plan.starts_at_home && plan.returns_home &&
           plan.calibration_section_count == 1 &&
           plan.reconfiguration_row_count == 0 &&
           plan.home_row_count == 2 &&
           plan.max_joint_ptp_step_rad <=
               kRosie1420CalibrationBoxMaxHomeJointDeltaRad &&
           rosie_1420_calibration_box_axis_bounds_valid(plan) &&
           plan.loop_command.present && plan.loop_command.valid &&
           plan.loop_command.target_row == 0 &&
           plan.loop_command.max_iterations ==
               kRosie1420CalibrationBoxLoopIterations &&
           plan.loop_command.return_home;
  }
  if (!is_calibration_tour_box_program_id(plan.program_id)) {
    return true;
  }
  return plan.starts_at_home && plan.returns_home &&
         plan.calibration_section_count >= 2 &&
         plan.reconfiguration_row_count >= 1 &&
         plan.home_row_count == 3 &&
         plan.max_joint_ptp_step_rad <= 0.051 &&
         plan.loop_command.present && plan.loop_command.valid &&
         plan.loop_command.target_row == 0 &&
         plan.loop_command.max_iterations >= 1 &&
         plan.loop_command.max_iterations <= 8 &&
         plan.loop_command.return_home;
}

bool spreadsheet_plan_validated(const SpreadsheetMotionServerPlan& plan,
                                const SpreadsheetCollisionCheck& collision) {
  return plan.planned && plan.unreachable_points.empty() && plan.joint_limits_checked &&
         plan.joint_speed_limits_applied && plan.singularity_analysis_available &&
         plan.singularity_hard_stop_clear &&
         spreadsheet_program_structure_valid(plan) &&
         (!plan.loop_command.present || plan.loop_command.valid) &&
         collision.ran && collision.contacts.empty() &&
         (!plan.tool_axis_constraint.present ||
          (plan.tool_axis_constraint.valid && plan.tool_axis_constraint_checked &&
           plan.tool_axis_constraint_satisfied)) &&
         (!plan.harmonious_axis_policy.present ||
          (plan.harmonious_axis_policy.valid && plan.harmonious_axis_applied &&
           (!plan.harmonious_axis_policy.workplane_relative ||
            plan.max_workplane_relative_error_m <= 1.0e-9)));
}

// Runtime preload gate: only the same fully validated plan advertised as
// play-enabled may stream to the realtime core. Collision contacts or missing
// collision evidence fail closed here while the report remains available for
// simulation-only inspection through the plan endpoint.
bool spreadsheet_plan_playable(const SpreadsheetMotionServerPlan& plan,
                               const SpreadsheetCollisionCheck& collision,
                               bool isolated_simulation_preview = false) {
  (void)isolated_simulation_preview;
  if (spreadsheet_program_is_simulation_only(plan.program_id)) {
    return false;
  }
  return spreadsheet_plan_validated(plan, collision);
}

std::string spreadsheet_plan_validation_state(const SpreadsheetMotionServerPlan& plan,
                                              const SpreadsheetCollisionCheck& collision) {
  if (!plan.planned || !plan.unreachable_points.empty() || !collision.contacts.empty() ||
      (plan.loop_command.present && !plan.loop_command.valid) ||
      (plan.singularity_analysis_available && !plan.singularity_hard_stop_clear)) {
    return "invalid";
  }
  if (spreadsheet_plan_validated(plan, collision)) {
    return "valid";
  }
  return "checks_unavailable";
}

std::string spreadsheet_plan_limit_check_state(const SpreadsheetMotionServerPlan& plan) {
  if (!plan.joint_limits_checked) {
    return "urdf_limits_unavailable";
  }
  for (const SpreadsheetPlanUnreachablePoint& point : plan.unreachable_points) {
    if (point.reason == "joint_limit") {
      return "joint_limit_violation";
    }
  }
  return "clear";
}

void spreadsheet_program_hash_u64(uint64_t* hash, uint64_t value) {
  if (hash == nullptr) {
    return;
  }
  for (int byte = 0; byte < 8; ++byte) {
    *hash ^= static_cast<uint8_t>((value >> (byte * 8)) & 0xffu);
    *hash *= 1099511628211ull;
  }
}

void spreadsheet_program_hash_double(uint64_t* hash, double value) {
  constexpr double kSpreadsheetProgramHashUnitsPerWhole = 1000000.0;
  uint64_t canonical = 0;
  if (std::isfinite(value)) {
    // Program rows are serialized to six decimal places. Hash the same
    // semantic precision instead of architecture-sensitive IEEE-754 results
    // from FK/trigonometric calculations. This also canonicalizes signed zero.
    const double maximum =
        static_cast<double>(std::numeric_limits<int64_t>::max()) /
        kSpreadsheetProgramHashUnitsPerWhole;
    if (value >= -maximum && value <= maximum) {
      const int64_t quantized = static_cast<int64_t>(
          std::llround(value * kSpreadsheetProgramHashUnitsPerWhole));
      canonical = static_cast<uint64_t>(quantized);
    } else {
      canonical = value < 0.0
                      ? static_cast<uint64_t>(std::numeric_limits<int64_t>::min())
                      : static_cast<uint64_t>(std::numeric_limits<int64_t>::max());
    }
  }
  spreadsheet_program_hash_u64(hash, canonical);
}

void spreadsheet_program_hash_string(uint64_t* hash, const std::string& value) {
  if (hash == nullptr) {
    return;
  }
  for (char ch : value) {
    *hash ^= static_cast<uint8_t>(ch);
    *hash *= 1099511628211ull;
  }
  spreadsheet_program_hash_u64(hash, static_cast<uint64_t>(value.size()));
}

std::string spreadsheet_program_hash(const SpreadsheetMotionServerPlan& plan,
                                     const std::vector<SpreadsheetPlanRow>& rows) {
  uint64_t hash = 14695981039346656037ull;
  spreadsheet_program_hash_string(&hash, plan.program_id);
  spreadsheet_program_hash_string(&hash, plan.program_source);
  spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(std::max(0, plan.program_revision)));
  spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(rows.size()));
  for (const SpreadsheetPlanRow& row : rows) {
    spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(std::max(0, row.row_index)));
    spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(std::max(0, row.revision)));
    spreadsheet_program_hash_u64(&hash, row.enabled ? 1u : 0u);
    spreadsheet_program_hash_string(&hash, row.row_id);
    spreadsheet_program_hash_string(&hash, row.row_hash);
    spreadsheet_program_hash_string(&hash, row.label);
    spreadsheet_program_hash_string(&hash, row.row_type);
    spreadsheet_program_hash_string(&hash, row.command);
    spreadsheet_program_hash_u64(
        &hash, static_cast<uint64_t>(std::max(-1, row.loop_target_row) + 1));
    spreadsheet_program_hash_u64(
        &hash, static_cast<uint64_t>(std::max(0, row.loop_max_iterations)));
    spreadsheet_program_hash_u64(&hash, row.loop_return_home ? 1u : 0u);
    spreadsheet_program_hash_string(&hash, row.move_type);
    spreadsheet_program_hash_string(&hash, row.target_frame_type);
    spreadsheet_program_hash_string(&hash, row.motion_phase);
    spreadsheet_program_hash_u64(
        &hash, static_cast<uint64_t>(std::max(0, row.calibration_section)));
    spreadsheet_program_hash_double(&hash, row.harmonious_phase_deg);
    spreadsheet_program_hash_string(&hash, row.tool_frame);
    spreadsheet_program_hash_string(&hash, row.work_frame);
    spreadsheet_program_hash_string(&hash, row.fixture_side);
    for (double value : row.tcp_pose) {
      spreadsheet_program_hash_double(&hash, value);
    }
    for (double value : row.joints_rad) {
      spreadsheet_program_hash_double(&hash, value);
    }
    spreadsheet_program_hash_double(&hash, row.tcp_velocity_m_s);
    spreadsheet_program_hash_double(&hash, row.joint_velocity_rad_s);
    spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(std::max(0, row.playback_step_ms)));
    spreadsheet_program_hash_u64(&hash, row.weld_enable ? 1u : 0u);
    spreadsheet_program_hash_string(&hash, row.arc_state);
    spreadsheet_program_hash_double(&hash, row.wire_feed_m_per_min);
    spreadsheet_program_hash_double(&hash, row.voltage_v);
    spreadsheet_program_hash_string(&hash, row.revpi_io_action);
    spreadsheet_program_hash_u64(&hash, static_cast<uint64_t>(std::max(0, row.revpi_io_slot)));
  }
  spreadsheet_program_hash_string(&hash, plan.harmonious_axis_policy.joint);
  spreadsheet_program_hash_string(&hash, plan.harmonious_axis_policy.active_side);
  spreadsheet_program_hash_string(&hash, plan.harmonious_axis_policy.selector_joint);
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.selector_rad);
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.selector_tolerance_rad);
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.center_rad);
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.amplitude_rad);
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.phase_deg);
  spreadsheet_program_hash_string(&hash, plan.harmonious_axis_policy.trajectory_source);
  spreadsheet_program_hash_string(&hash, plan.harmonious_axis_policy.work_frame);
  spreadsheet_program_hash_u64(&hash, plan.harmonious_axis_policy.workplane_relative ? 1u : 0u);
  for (double value : plan.harmonious_axis_policy.workplane_pivot_m) {
    spreadsheet_program_hash_double(&hash, value);
  }
  for (double value : plan.harmonious_axis_policy.workplane_axis_xyz) {
    spreadsheet_program_hash_double(&hash, value);
  }
  spreadsheet_program_hash_double(&hash, plan.harmonious_axis_policy.reference_rad);
  spreadsheet_program_hash_u64(&hash,
                               plan.harmonious_axis_policy.tool_axis_follows_workplane ? 1u : 0u);
  std::ostringstream encoded;
  encoded << "fnv1a64:" << std::hex << std::setfill('0') << std::setw(16) << hash;
  return encoded.str();
}

std::string spreadsheet_plan_round_trip_double_json(double value) {
  std::ostringstream out;
  out << std::defaultfloat
      << std::setprecision(std::numeric_limits<double>::max_digits10)
      << value;
  return out.str();
}

std::string spreadsheet_plan_round_trip_array_json(const double* values,
                                                   int count) {
  std::ostringstream out;
  out << "[";
  for (int index = 0; index < count; ++index) {
    if (index > 0) {
      out << ",";
    }
    out << spreadsheet_plan_round_trip_double_json(values[index]);
  }
  out << "]";
  return out.str();
}

std::string render_spreadsheet_tesseract_plan_report(const std::vector<SpreadsheetPlanRow>& rows,
                                                     const SpreadsheetMotionServerPlan& plan,
                                                     int samples_per_segment,
                                                     const SpreadsheetCollisionCheck& collision,
                                                     const std::string& planner_request_subject) {
  const double max_sample_gap_s =
      plan.trajectory.size() < 2 ? 0.0 : plan.duration_s / static_cast<double>(plan.trajectory.size() - 1);
  const std::string validation_state = spreadsheet_plan_validation_state(plan, collision);
  const std::string limit_check_state = spreadsheet_plan_limit_check_state(plan);
  const std::string program_hash = spreadsheet_program_hash(plan, rows);
  const SpreadsheetJointLimits playback_limits =
      load_spreadsheet_joint_limits();
  const std::string planner_urdf_path =
      resolve_spreadsheet_planner_urdf_path();
  std::string planner_robot_model = kMotionServerRobotModel;
  std::string planner_mesh_package_root;
  if (!spreadsheet_collision_pair_identity(
          planner_urdf_path, &planner_robot_model,
          &planner_mesh_package_root)) {
    planner_robot_model = kMotionServerRobotModel;
    planner_mesh_package_root.clear();
  }
  double max_arm_home_delta_rad = 0.0;
  double max_external_axis_abs_rad = 0.0;
  std::array<double, 6> arm_minimum{};
  std::array<double, 6> arm_maximum{};
  for (std::size_t axis = 0; axis < arm_minimum.size(); ++axis) {
    arm_minimum[axis] = std::numeric_limits<double>::infinity();
    arm_maximum[axis] = -std::numeric_limits<double>::infinity();
  }
  for (const SpreadsheetPlanPoint& point : plan.trajectory) {
    for (std::size_t axis = 0; axis < arm_minimum.size(); ++axis) {
      max_arm_home_delta_rad =
          std::max(max_arm_home_delta_rad,
                   std::abs(point.joints_rad[axis]));
      arm_minimum[axis] =
          std::min(arm_minimum[axis], point.joints_rad[axis]);
      arm_maximum[axis] =
          std::max(arm_maximum[axis], point.joints_rad[axis]);
    }
    for (std::size_t axis = 6u; axis < kLiveAxisCount; ++axis) {
      max_external_axis_abs_rad =
          std::max(max_external_axis_abs_rad,
                   std::abs(point.joints_rad[axis]));
    }
  }
  int moving_arm_axis_count = 0;
  for (std::size_t axis = 0; axis < arm_minimum.size(); ++axis) {
    if (arm_maximum[axis] - arm_minimum[axis] > 1.0e-5) {
      moving_arm_axis_count++;
    }
  }
  const bool external_axes_stationary =
      max_external_axis_abs_rad <= 1.0e-9;
  const bool near_home_joint_bound_satisfied =
      max_arm_home_delta_rad <=
      kRosie1420CalibrationBoxMaxHomeJointDeltaRad;
  double max_fk_translation_residual_m = 0.0;
  double max_fk_orientation_residual_rad = 0.0;
  for (const SpreadsheetPlanPoint& point : plan.trajectory) {
    double fk_tcp_pose[6]{};
    spreadsheet_fk_pose_from_joints(point.joints_rad, fk_tcp_pose);
    max_fk_translation_residual_m =
        std::max(max_fk_translation_residual_m,
                 spreadsheet_pose_translation_residual_m(point.tcp_pose,
                                                         fk_tcp_pose));
    max_fk_orientation_residual_rad =
        std::max(max_fk_orientation_residual_rad,
                 spreadsheet_pose_orientation_residual_rad(point.tcp_pose,
                                                           fk_tcp_pose));
  }
  const bool all_sample_fk_within_tolerance =
      max_fk_translation_residual_m <= kSpreadsheetFKTranslationToleranceM &&
      max_fk_orientation_residual_rad <= kSpreadsheetFKOrientationToleranceRad;
  double playback_wrapped_equivalence_axis[kLiveAxisCount]{};
  if (playback_limits.loaded) {
    for (std::size_t axis = 0; axis < kLiveAxisCount; ++axis) {
      playback_wrapped_equivalence_axis[axis] =
          playback_limits.upper_rad[axis] -
                      playback_limits.lower_rad[axis] >
                  2.0 * kPi
              ? 1.0
              : 0.0;
    }
  }
  const bool planning_contract_accepted =
      plan.planned && plan.singularity_analysis_available &&
      plan.singularity_hard_stop_clear &&
      spreadsheet_program_structure_valid(plan) &&
      (!plan.loop_command.present || plan.loop_command.valid) &&
      (!plan.harmonious_axis_policy.present ||
       (plan.harmonious_axis_policy.valid && plan.harmonious_axis_applied &&
        (!plan.harmonious_axis_policy.workplane_relative ||
         plan.max_workplane_relative_error_m <= 1.0e-9))) &&
      (!plan.tool_axis_constraint.present ||
       (plan.tool_axis_constraint.valid && plan.tool_axis_constraint_checked &&
        plan.tool_axis_constraint_satisfied));
  std::ostringstream report;
  report << "{"
         << "\"name\":\"robot-v4-cartesiand\","
         << "\"self_test\":\"spreadsheet-tesseract-plan\","
         << "\"planner_owner\":\"" << kMotionServerTesseractPlannerOwner << "\","
         << "\"planner_backend\":\"" << kMotionServerTesseractPlannerBackend << "\","
         << "\"trajectory_method\":\"" << kMotionServerTesseractTrajectoryMethod << "\","
         << "\"planner_request_subject\":\"" << json_escape(planner_request_subject) << "\","
         << "\"integration\":\"motion_server_owned_planner_for_spreadsheet_playback\","
         << "\"program_id\":\"" << json_escape(plan.program_id) << "\","
         << "\"program_source\":\"" << json_escape(plan.program_source) << "\","
         << "\"program_revision\":" << plan.program_revision << ","
         << "\"accepted_program_revision\":" << plan.program_revision << ","
         << "\"program_revision_semantics\":\"optimistic-client-base\","
         << "\"program_hash\":\"" << program_hash << "\","
         << "\"simulation_only_program\":"
         << (spreadsheet_program_is_simulation_only(plan.program_id) ? "true" : "false")
         << ","
         << "\"execution_policy\":\""
         << (spreadsheet_program_is_simulation_only(plan.program_id)
                 ? "simulation-preview-only"
                 : "validated-runtime-or-simulation")
         << "\","
         << "\"mapped_axis_mask\":"
         << (planner_robot_model == "rosie_1420_v1" ? 63 : 511)
         << ","
         << "\"arm_axis_count\":6,"
         << "\"moving_arm_axis_count\":" << moving_arm_axis_count << ","
         << "\"external_axis_count\":"
         << (planner_robot_model == "rosie_1420_v1" ? 0 : 3)
         << ","
         << "\"external_axes_stationary\":"
         << (external_axes_stationary ? "true" : "false") << ","
         << "\"max_external_axis_abs_rad\":"
         << std::fixed << std::setprecision(9)
         << max_external_axis_abs_rad << ","
         << "\"max_arm_home_delta_rad\":"
         << std::fixed << std::setprecision(9)
         << max_arm_home_delta_rad << ","
         << "\"near_home_joint_bound_rad\":"
         << std::fixed << std::setprecision(9)
         << kRosie1420CalibrationBoxMaxHomeJointDeltaRad << ","
         << "\"near_home_joint_bound_satisfied\":"
         << (near_home_joint_bound_satisfied ? "true" : "false")
         << ","
         << "\"program_row_count\":" << rows.size() << ","
         << "\"motion_row_count\":" << plan.motion_row_count << ","
         << "\"calibration_section_count\":" << plan.calibration_section_count << ","
         << "\"reconfiguration_row_count\":" << plan.reconfiguration_row_count << ","
         << "\"home_row_count\":" << plan.home_row_count << ","
         << "\"starts_at_home\":" << (plan.starts_at_home ? "true" : "false") << ","
         << "\"returns_home\":" << (plan.returns_home ? "true" : "false") << ","
         << "\"program_structure_valid\":"
         << (spreadsheet_program_structure_valid(plan) ? "true" : "false") << ","
         << "\"command_row_count\":"
         << (static_cast<int>(rows.size()) - plan.motion_row_count) << ","
         << "\"loop_command_present\":"
         << (plan.loop_command.present ? "true" : "false") << ","
         << "\"loop_command_valid\":"
         << (plan.loop_command.valid ? "true" : "false") << ","
         << "\"loop_command_validation_error\":\""
         << json_escape(plan.loop_command.validation_error) << "\","
         << "\"loop_command_row_index\":" << plan.loop_command.row_index << ","
         << "\"loop_target_row\":" << plan.loop_command.target_row << ","
         << "\"loop_target_row_id\":\""
         << (plan.loop_command.present && plan.loop_command.target_row >= 0 &&
                     plan.loop_command.target_row < static_cast<int>(rows.size())
                 ? json_escape(rows[static_cast<std::size_t>(plan.loop_command.target_row)].row_id)
                 : "")
         << "\","
         << "\"loop_max_iterations\":" << plan.loop_command.max_iterations << ","
         << "\"loop_iterations\":" << plan.loop_command.max_iterations << ","
         << "\"loop_return_home\":"
         << (plan.loop_command.return_home ? "true" : "false") << ","
         << "\"return_home_before_repeat\":"
         << (plan.loop_command.return_home ? "true" : "false") << ","
         << "\"constraint_owner\":\"motion-server\","
         << "\"constraint_schema\":\"robot.v4.tool-axis-constraint.v1\","
         << "\"tool_axis_constraint_present\":"
         << (plan.tool_axis_constraint.present ? "true" : "false") << ","
         << "\"tool_axis_constraint_valid\":"
         << (plan.tool_axis_constraint.valid ? "true" : "false") << ","
         << "\"tool_axis_constraint_validation_error\":\""
         << json_escape(plan.tool_axis_constraint.validation_error) << "\","
         << "\"tool_axis_constraint_checked\":"
         << (plan.tool_axis_constraint_checked ? "true" : "false") << ","
         << "\"tool_axis_constraint_satisfied\":"
         << (plan.tool_axis_constraint_satisfied ? "true" : "false") << ","
         << "\"tool_frame\":\"" << json_escape(plan.tool_axis_constraint.tool_frame) << "\","
         << "\"tip_link\":\"" << json_escape(plan.tool_axis_constraint.tip_link) << "\","
         << "\"world_frame\":\"" << json_escape(plan.tool_axis_constraint.world_frame) << "\","
         << "\"tool_axis_xyz\":"
         << double_array_json_fixed(plan.tool_axis_constraint.tool_axis_xyz, 3) << ","
         << "\"world_axis_xyz\":"
         << double_array_json_fixed(plan.tool_axis_constraint.world_axis_xyz, 3) << ","
         << "\"offset_reference_axis_xyz\":"
         << double_array_json_fixed(plan.tool_axis_constraint.offset_reference_axis_xyz, 3) << ","
         << "\"constraint_offset_deg\":" << std::fixed << std::setprecision(6)
         << plan.tool_axis_constraint.offset_deg << ","
         << "\"constraint_tolerance_minus_deg\":" << std::fixed << std::setprecision(6)
         << plan.tool_axis_constraint.tolerance_minus_deg << ","
         << "\"constraint_tolerance_plus_deg\":" << std::fixed << std::setprecision(6)
         << plan.tool_axis_constraint.tolerance_plus_deg << ","
         << "\"max_tool_axis_error_deg\":" << std::fixed << std::setprecision(6)
         << plan.max_tool_axis_error_deg << ","
         << "\"min_measured_tool_axis_offset_deg\":" << std::fixed << std::setprecision(6)
         << plan.min_measured_tool_axis_offset_deg << ","
         << "\"max_measured_tool_axis_offset_deg\":" << std::fixed << std::setprecision(6)
         << plan.max_measured_tool_axis_offset_deg << ","
         << "\"harmonious_external_axis_present\":"
         << (plan.harmonious_axis_policy.present ? "true" : "false") << ","
         << "\"harmonious_external_axis_valid\":"
         << (plan.harmonious_axis_policy.valid ? "true" : "false") << ","
         << "\"harmonious_external_axis_validation_error\":\""
         << json_escape(plan.harmonious_axis_policy.validation_error) << "\","
         << "\"harmonious_external_axis_joint\":\""
         << json_escape(plan.harmonious_axis_policy.joint) << "\","
         << "\"harmonious_active_side\":\""
         << json_escape(plan.harmonious_axis_policy.active_side) << "\","
         << "\"harmonious_selector_joint\":\""
         << json_escape(plan.harmonious_axis_policy.selector_joint) << "\","
         << "\"harmonious_selector_rad\":" << std::fixed << std::setprecision(6)
         << plan.harmonious_axis_policy.selector_rad << ","
         << "\"harmonious_selector_tolerance_rad\":" << std::fixed
         << std::setprecision(6)
         << plan.harmonious_axis_policy.selector_tolerance_rad << ","
         << "\"harmonious_external_axis_applied\":"
         << (plan.harmonious_axis_applied ? "true" : "false") << ","
         << "\"harmonious_external_axis_min_rad\":" << std::fixed << std::setprecision(6)
         << plan.harmonious_axis_min_rad << ","
         << "\"harmonious_external_axis_max_rad\":" << std::fixed << std::setprecision(6)
         << plan.harmonious_axis_max_rad << ","
         << "\"harmonious_trajectory_source\":\""
         << json_escape(plan.harmonious_axis_policy.trajectory_source) << "\","
         << "\"harmonious_workplane_relative\":"
         << (plan.harmonious_axis_policy.workplane_relative ? "true" : "false") << ","
         << "\"harmonious_work_frame\":\""
         << json_escape(plan.harmonious_axis_policy.work_frame) << "\","
         << "\"harmonious_workplane_pivot_m\":"
         << double_array_json_fixed(plan.harmonious_axis_policy.workplane_pivot_m, 3) << ","
         << "\"harmonious_workplane_axis_xyz\":"
         << double_array_json_fixed(plan.harmonious_axis_policy.workplane_axis_xyz, 3) << ","
         << "\"harmonious_reference_rad\":" << std::fixed << std::setprecision(6)
         << plan.harmonious_axis_policy.reference_rad << ","
         << "\"harmonious_tool_axis_follows_workplane\":"
         << (plan.harmonious_axis_policy.tool_axis_follows_workplane ? "true" : "false") << ","
         << "\"jogging_solver\":\"direct_motion_server_cartesian_or_joint_solver\","
         << "\"playback_solver\":\"tesseract_dense_trajectory\","
         << "\"database_schema_version\":1,"
         << "\"robotic-cell-type\":\""
         << json_escape(planner_robot_model) << "\","
         << "\"robot_model\":\""
         << json_escape(planner_robot_model) << "\","
         << "\"tesseract_urdf_source\":\""
         << json_escape(
                planner_robot_model == kMotionServerRobotModel
                    ? std::string(kMotionServerTesseractURDFSource)
                    : planner_urdf_path)
         << "\","
         << "\"tesseract_collision_geometry\":\"" << kMotionServerTesseractCollisionGeometry << "\","
         << "\"tesseract_collision_mesh_package\":\"package://"
         << json_escape(planner_robot_model) << "/meshes\","
         << "\"program_state\":\"planned\","
         << "\"planner_status\":\"planned\","
         << "\"validation_state\":\"" << validation_state << "\","
         << "\"collision_gate_clear\":"
         << (collision.ran && collision.contacts.empty() ? "true" : "false") << ","
         << "\"play_enabled\":"
         << (spreadsheet_plan_validated(plan, collision) ? "true" : "false") << ","
         << "\"runtime_preload_enabled\":"
         << (spreadsheet_plan_playable(plan, collision) ? "true" : "false") << ","
         << "\"isolated_simulation_preload_enabled\":"
         << (spreadsheet_plan_playable(plan, collision, true) ? "true" : "false") << ","
         << "\"simulation_preview_enabled\":" << (plan.planned ? "true" : "false") << ","
         << "\"collision_check_state\":\"" << json_escape(collision.state) << "\","
         << "\"collision_checker\":\"" << json_escape(collision.checker) << "\","
         << "\"collision_checker_library_version\":\"" << json_escape(collision.library_version) << "\","
         << "\"collision_check_error\":\"" << json_escape(collision.error) << "\","
         << "\"collision_checked_waypoint_count\":" << collision.checked_waypoint_count << ","
         << "\"collision_cache_hit_waypoint_count\":"
         << collision.cache_hit_waypoint_count << ","
         << "\"collision_check_time_s\":" << std::fixed << std::setprecision(6)
         << collision.check_duration_s << ","
         << "\"collision_worker_wall_time_s\":" << std::fixed << std::setprecision(6)
         << collision.worker_wall_time_s << ","
         << "\"collision_joint_mapping\":\"urdf_joints_equal_plan_positions_rad\","
         << "\"collision_point_count\":" << collision.contacts.size() << ","
         << "\"motion_safety_evidence_schema\":\"robot.v4.motion-safety-evidence.v1\","
         << "\"motion_allowed\":"
         << (spreadsheet_plan_validated(plan, collision) ? "true" : "false") << ","
         << "\"singularity_check_state\":\""
         << (plan.singularity_analysis_available ? "jacobian_svd_checked" : "unavailable")
         << "\","
         << "\"singularity_analysis_available\":"
         << (plan.singularity_analysis_available ? "true" : "false") << ","
         << "\"singularity_gate_clear\":"
         << (plan.singularity_hard_stop_clear ? "true" : "false") << ","
         << "\"singularity_warning_threshold\":" << std::fixed << std::setprecision(6)
         << plan.singularity_warning_threshold << ","
         << "\"singularity_hard_stop_threshold\":" << std::fixed << std::setprecision(6)
         << plan.singularity_hard_stop_threshold << ","
         << "\"singularity_warning_point_count\":"
         << plan.singularity_warning_point_count << ","
         << "\"singularity_hard_stop_point_count\":"
         << plan.singularity_hard_stop_point_count << ","
         << "\"minimum_singularity_score\":" << std::fixed << std::setprecision(9)
         << plan.minimum_singularity_score << ","
         << "\"minimum_applied_velocity_scale\":" << std::fixed << std::setprecision(9)
         << plan.minimum_applied_velocity_scale << ","
         << "\"velocity_scale_invariant\":\"scaled_velocity_i=requested_velocity_i*applied_velocity_scale\","
         << "\"limit_check_state\":\"" << limit_check_state << "\","
         << "\"limit_check_source\":\"" << json_escape(plan.joint_limits_source) << "\","
         << "\"unreachable_point_count\":" << plan.unreachable_points.size() << ","
         << "\"planning_time_s\":" << std::fixed << std::setprecision(6)
         << plan.planning_duration_s << ","
         << "\"samples_per_segment\":" << samples_per_segment << ","
         << "\"source_row_count\":" << plan.motion_row_count << ","
         << "\"planned_sample_count\":" << plan.trajectory.size() << ","
         << "\"duration_s\":" << std::fixed << std::setprecision(6) << plan.duration_s << ","
         << "\"tool_path_length_m\":" << std::fixed << std::setprecision(6) << plan.tool_path_length_m << ","
         << "\"max_sample_gap_s\":" << std::fixed << std::setprecision(6) << max_sample_gap_s << ","
         << "\"max_tcp_step_m\":" << std::fixed << std::setprecision(6) << plan.max_tcp_step_m << ","
         << "\"max_joint_step_rad\":" << std::fixed << std::setprecision(6) << plan.max_joint_step_rad << ","
         << "\"max_joint_ptp_step_rad\":" << std::fixed << std::setprecision(6)
         << plan.max_joint_ptp_step_rad << ","
         << "\"max_line_tcp_joint_step_rad\":" << std::fixed << std::setprecision(6)
         << plan.max_line_tcp_joint_step_rad << ","
         << "\"hidden_start_transition\":"
         << (plan.starts_at_home ? "false" : "true") << ","
         << "\"joint_angle_continuity_applied\":true,"
         << "\"playback_wrapped_equivalence_axis\":"
         << double_array_json_fixed(playback_wrapped_equivalence_axis,
                                    static_cast<int>(kLiveAxisCount))
         << ","
         << "\"joint_angle_wraps_resolved\":" << plan.joint_angle_wraps_resolved << ","
         << "\"joint_speed_limits_applied\":"
         << (plan.joint_speed_limits_applied ? "true" : "false") << ","
         << "\"joint_speed_limits_rad_s\":"
         << double_array_json_fixed(plan.joint_speed_limits_rad_s.data(),
                                    static_cast<int>(kLiveAxisCount)) << ","
         << "\"max_joint_speeds_rad_s\":"
         << double_array_json_fixed(plan.max_joint_speeds_rad_s.data(),
                                    static_cast<int>(kLiveAxisCount)) << ","
         << "\"max_tcp_fk_error_m\":" << std::fixed << std::setprecision(6) << plan.max_tcp_fk_error_m << ","
         << "\"fk_translation_tolerance_m\":" << std::fixed
         << std::setprecision(6) << kSpreadsheetFKTranslationToleranceM << ","
         << "\"fk_orientation_tolerance_rad\":" << std::fixed
         << std::setprecision(6) << kSpreadsheetFKOrientationToleranceRad << ","
         << "\"max_fk_translation_residual_m\":" << std::fixed
         << std::setprecision(9) << max_fk_translation_residual_m << ","
         << "\"max_fk_orientation_residual_rad\":" << std::fixed
         << std::setprecision(9) << max_fk_orientation_residual_rad << ","
         << "\"all_sample_fk_within_tolerance\":"
         << (all_sample_fk_within_tolerance ? "true" : "false") << ","
         << "\"fk_sample_count\":" << plan.trajectory.size() << ","
         << "\"max_workplane_relative_error_m\":" << std::fixed << std::setprecision(9)
         << plan.max_workplane_relative_error_m << ","
         << "\"smooth_dense_trajectory\":" << (plan.planned && plan.max_tcp_step_m <= 0.11 ? "true" : "false") << ","
         << "\"spreadsheet_database_fields\":["
         << "\"schema_version\",\"program_id\",\"program_hash\",\"program_name\",\"part_name\",\"robotic-cell-type\","
         << "\"default_tool_frame\",\"default_work_frame\",\"work_frame\",\"row_id\",\"row_number\",\"row_type\",\"command\","
         << "\"loop_target_row\",\"loop_max_iterations\",\"loop_return_home\",\"move_type\","
         << "\"motion_phase\",\"calibration_section\","
         << "\"harmonious_phase_deg\","
         << "\"fixture_side\",\"active_side\",\"selector_joint\",\"selector_rad\","
         << "\"target_frame_type\",\"target_tcp_m\",\"target_joints_rad\",\"current_tcp_pose\","
         << "\"current_joints_rad\",\"tcp_velocity_m_s\",\"joint_velocity_rad_s\",\"blend_radius_m\","
         << "\"wire_feed_m_per_min\",\"voltage_v\","
         << "\"planner_status\",\"planned_sample_count\",\"validation_state\",\"last_test_session_id\""
         << "],"
         << "\"program_rows\":[";
  for (std::size_t index = 0; index < rows.size(); ++index) {
    if (index > 0) {
      report << ",";
    }
    const SpreadsheetPlanRow& row = rows[index];
    report << "{\"row_id\":\"" << json_escape(row.row_id) << "\""
           << ",\"row_index\":" << row.row_index
           << ",\"row_number\":" << (row.row_index + 1)
           << ",\"revision\":" << row.revision
           << ",\"row_hash\":\"" << json_escape(row.row_hash) << "\""
           << ",\"label\":\"" << json_escape(row.label) << "\""
           << ",\"enabled\":" << (row.enabled ? "true" : "false")
           << ",\"row_type\":\"" << json_escape(row.row_type) << "\""
           << ",\"command\":\"" << json_escape(row.command) << "\""
           << ",\"loop_target_row\":" << row.loop_target_row
           << ",\"loop_target_row_id\":\""
           << (row.loop_target_row >= 0 &&
                       row.loop_target_row < static_cast<int>(rows.size())
                   ? json_escape(rows[static_cast<std::size_t>(row.loop_target_row)].row_id)
                   : "")
           << "\""
           << ",\"loop_max_iterations\":" << row.loop_max_iterations
           << ",\"loop_iterations\":" << row.loop_max_iterations
           << ",\"loop_return_home\":" << (row.loop_return_home ? "true" : "false")
           << ",\"return_home_before_repeat\":"
           << (row.loop_return_home ? "true" : "false")
           << ",\"move_type\":\"" << json_escape(row.move_type) << "\""
           << ",\"motion_phase\":\"" << json_escape(row.motion_phase) << "\""
           << ",\"calibration_section\":" << row.calibration_section
           << ",\"harmonious_phase_deg\":" << std::fixed << std::setprecision(6)
           << row.harmonious_phase_deg
           << ",\"target_frame_type\":\"" << json_escape(row.target_frame_type) << "\""
           << ",\"tool_frame\":\"" << json_escape(row.tool_frame) << "\""
           << ",\"work_frame\":\"" << json_escape(row.work_frame) << "\""
           << ",\"fixture_side\":\"" << json_escape(row.fixture_side) << "\""
           << ",\"target_tcp_m\":" << double_array_json_fixed(row.tcp_pose, 6)
           << ",\"target_joints_rad\":"
           << double_array_json_fixed(row.joints_rad, static_cast<int>(kLiveAxisCount))
           << ",\"tcp_velocity_m_s\":" << std::fixed << std::setprecision(6)
           << row.tcp_velocity_m_s
           << ",\"joint_velocity_rad_s\":" << std::fixed
           << std::setprecision(6) << row.joint_velocity_rad_s
           << ",\"playback_step_ms\":" << row.playback_step_ms
           << ",\"weld_enable\":" << (row.weld_enable ? "true" : "false")
           << ",\"arc_state\":\"" << json_escape(row.arc_state) << "\""
           << ",\"wire_feed_m_per_min\":" << std::fixed
           << std::setprecision(6) << row.wire_feed_m_per_min
           << ",\"voltage_v\":" << std::fixed << std::setprecision(6)
           << row.voltage_v
           << ",\"revpi_io_slot\":" << row.revpi_io_slot
           << ",\"revpi_io_action\":\"" << json_escape(row.revpi_io_action) << "\"}";
  }
  report << "],"
         << "\"unreachable_points\":[";
  for (std::size_t index = 0; index < plan.unreachable_points.size(); ++index) {
    if (index > 0) {
      report << ",";
    }
    const SpreadsheetPlanUnreachablePoint& point = plan.unreachable_points[index];
    report << "{\"row_index\":" << point.row_index
           << ",\"waypoint_index\":" << point.waypoint_index
           << ",\"time_s\":" << std::fixed << std::setprecision(6) << point.time_s
           << ",\"tcp_pose\":" << double_array_json_fixed(point.tcp_pose, 6)
           << ",\"reason\":\"" << json_escape(point.reason) << "\"";
    if (!point.joint.empty()) {
      report << ",\"joint\":\"" << json_escape(point.joint) << "\""
             << ",\"joint_value_rad\":" << std::fixed << std::setprecision(6)
             << point.joint_value_rad;
    }
    report << "}";
  }
  report << "],"
         << "\"collision_points\":[";
  for (std::size_t index = 0; index < collision.contacts.size(); ++index) {
    if (index > 0) {
      report << ",";
    }
    const SpreadsheetPlanCollisionPoint& contact = collision.contacts[index];
    const SpreadsheetPlanPoint* collision_sample =
        contact.waypoint_index >= 0 &&
                contact.waypoint_index < static_cast<int>(plan.trajectory.size())
            ? &plan.trajectory[static_cast<std::size_t>(contact.waypoint_index)]
            : nullptr;
    report << "{\"row_index\":" << contact.row_index
           << ",\"waypoint_index\":" << contact.waypoint_index
           << ",\"time_s\":" << std::fixed << std::setprecision(6) << contact.time_s
           << ",\"tcp_pose\":" << double_array_json_fixed(contact.tcp_pose, 6)
           << ",\"link_a\":\"" << json_escape(contact.link_a) << "\""
           << ",\"link_b\":\"" << json_escape(contact.link_b) << "\""
           << ",\"depth_m\":" << std::fixed << std::setprecision(6) << contact.depth_m
           << ",\"affected_joints\":[";
    for (std::size_t joint_index = 0; joint_index < contact.affected_joints.size(); ++joint_index) {
      if (joint_index > 0) {
        report << ",";
      }
      report << "\"" << json_escape(contact.affected_joints[joint_index]) << "\"";
    }
    report << "]";
    if (collision_sample != nullptr) {
      report << ",\"positions_rad\":"
             << double_array_json_fixed(collision_sample->joints_rad,
                                        static_cast<int>(kLiveAxisCount));
    }
    report << "}";
  }
  report << "],\"singularity_points\":[";
  bool wrote_singularity_point = false;
  for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
    const SpreadsheetPlanPoint& point = plan.trajectory[index];
    if (point.singularity_class != "warning" && point.singularity_class != "hard_stop" &&
        point.singularity_class != "unavailable") {
      continue;
    }
    if (wrote_singularity_point) {
      report << ",";
    }
    wrote_singularity_point = true;
    report << "{\"row_index\":" << point.row_index
           << ",\"waypoint_index\":" << index
           << ",\"time_s\":"
           << spreadsheet_plan_round_trip_double_json(point.time_s)
           << ",\"tcp_pose\":" << double_array_json_fixed(point.tcp_pose, 6)
           << ",\"positions_rad\":"
           << double_array_json_fixed(point.joints_rad, static_cast<int>(kLiveAxisCount))
           << ",\"singularity_class\":\"" << json_escape(point.singularity_class) << "\""
           << ",\"singularity_score\":" << std::fixed << std::setprecision(9)
           << point.singularity_score
           << ",\"jacobian_singular_values\":"
           << double_array_json_fixed(point.jacobian_singular_values, 6)
           << ",\"jacobian_condition_number\":" << std::fixed << std::setprecision(6)
           << point.jacobian_condition_number
           << ",\"jacobian_manipulability\":" << std::fixed << std::setprecision(9)
           << point.jacobian_manipulability
           << ",\"singularity_scale\":" << std::fixed << std::setprecision(9)
           << point.singularity_scale
           << ",\"applied_velocity_scale\":" << std::fixed << std::setprecision(9)
           << point.applied_velocity_scale
           << ",\"singular_direction_joint\":\""
           << (point.singular_direction_joint_index >= 0
                   ? spreadsheet_planner_joint_name(
                         static_cast<std::size_t>(point.singular_direction_joint_index))
                   : "none")
           << "\"}";
  }
  report << "],"
         << "\"trajectory\":[";
  for (std::size_t index = 0; index < plan.trajectory.size(); ++index) {
    if (index > 0) {
      report << ",";
    }
    const SpreadsheetPlanPoint& point = plan.trajectory[index];
    const std::size_t report_harmonious_axis =
        plan.harmonious_axis_policy.joint == "J8" ? 7u : 6u;
    const SpreadsheetToolAxisConstraint sample_constraint =
        harmonious_sample_tool_constraint(plan.tool_axis_constraint,
                                           plan.harmonious_axis_policy,
                                           point.joints_rad[report_harmonious_axis]);
    double fk_tcp_pose[6]{};
    spreadsheet_fk_pose_from_joints(point.joints_rad, fk_tcp_pose);
    const double fk_translation_residual_m =
        spreadsheet_pose_translation_residual_m(point.tcp_pose, fk_tcp_pose);
    const double fk_orientation_residual_rad =
        spreadsheet_pose_orientation_residual_rad(point.tcp_pose, fk_tcp_pose);
    report << "{\"row_index\":" << point.row_index
           << ",\"waypoint_index\":" << index
           << ",\"time_s\":"
           << spreadsheet_plan_round_trip_double_json(point.time_s)
           << ",\"move_type\":\"" << json_escape(point.move_type) << "\""
           << ",\"motion_phase\":\"" << json_escape(point.motion_phase) << "\""
           << ",\"calibration_section\":" << point.calibration_section
           << ",\"path_parent\":\"" << json_escape(point.path_parent) << "\""
           << ",\"path_segment\":\"" << json_escape(point.path_segment) << "\""
           << ",\"work_frame\":\"" << json_escape(point.work_frame) << "\""
           << ",\"fixture_side\":\"" << json_escape(point.fixture_side) << "\""
           << ",\"tool_axis_constraint_active\":"
           << (point.tool_axis_constraint_active ? "true" : "false")
           << ",\"harmonious_workplane_active\":"
           << (point.harmonious_workplane_active ? "true" : "false")
           << ",\"weld_on\":" << (point.weld_on ? "true" : "false")
           << ",\"tcp_pose\":"
           << spreadsheet_plan_round_trip_array_json(point.tcp_pose, 6)
           << ",\"positions_rad\":"
           << spreadsheet_plan_round_trip_array_json(
                  point.joints_rad, static_cast<int>(kLiveAxisCount))
           << ",\"fk_tcp_pose\":"
           << spreadsheet_plan_round_trip_array_json(fk_tcp_pose, 6)
           << ",\"fk_translation_residual_m\":"
           << spreadsheet_plan_round_trip_double_json(
                  fk_translation_residual_m)
           << ",\"fk_orientation_residual_rad\":"
           << spreadsheet_plan_round_trip_double_json(
                  fk_orientation_residual_rad)
           << ",\"fk_within_tolerance\":"
           << (fk_translation_residual_m <=
                       kSpreadsheetFKTranslationToleranceM &&
                   fk_orientation_residual_rad <=
                       kSpreadsheetFKOrientationToleranceRad
                   ? "true"
                   : "false")
           << ",\"requested_velocities_rad_s\":"
           << spreadsheet_plan_round_trip_array_json(
                  point.requested_velocities_rad_s,
                  static_cast<int>(kLiveAxisCount))
           << ",\"scaled_velocities_rad_s\":"
           << spreadsheet_plan_round_trip_array_json(
                  point.scaled_velocities_rad_s,
                  static_cast<int>(kLiveAxisCount))
           << ",\"singularity_class\":\"" << json_escape(point.singularity_class) << "\""
           << ",\"singularity_score\":" << std::fixed << std::setprecision(9)
           << point.singularity_score
           << ",\"jacobian_singular_values\":"
           << double_array_json_fixed(point.jacobian_singular_values, 6)
           << ",\"jacobian_condition_number\":" << std::fixed << std::setprecision(6)
           << point.jacobian_condition_number
           << ",\"jacobian_manipulability\":" << std::fixed << std::setprecision(9)
           << point.jacobian_manipulability
           << ",\"joint_limit_scale\":" << std::fixed << std::setprecision(9)
           << point.joint_limit_scale
           << ",\"singularity_scale\":" << std::fixed << std::setprecision(9)
           << point.singularity_scale
           << ",\"applied_velocity_scale\":" << std::fixed << std::setprecision(9)
           << point.applied_velocity_scale
           << ",\"limiting_joint\":\""
           << (point.limiting_joint_index >= 0
                   ? spreadsheet_planner_joint_name(
                         static_cast<std::size_t>(point.limiting_joint_index))
                   : "none")
           << "\",\"singular_direction_joint\":\""
           << (point.singular_direction_joint_index >= 0
                   ? spreadsheet_planner_joint_name(
                         static_cast<std::size_t>(point.singular_direction_joint_index))
                   : "none")
           << "\"";
    if (point.tool_axis_constraint_active &&
        plan.tool_axis_constraint.present && plan.tool_axis_constraint.valid) {
      double q[6]{};
      for (int axis = 0; axis < 6; ++axis) {
        q[axis] = point.joints_rad[axis];
      }
      const std::array<double, 3> actual_axis = normalized3(
          rotate_direction(urdf_forward_kinematics_j1_base(q),
                           sample_constraint.tool_axis_xyz));
      report << ",\"tool_axis_world_xyz\":" << double_array_json_fixed(actual_axis.data(), 3)
             << ",\"target_tool_axis_world_xyz\":"
             << double_array_json_fixed(sample_constraint.world_axis_xyz, 3)
             << ",\"measured_tool_axis_offset_deg\":" << std::fixed << std::setprecision(6)
             << spreadsheet_constraint_measured_offset_deg(sample_constraint, actual_axis)
             << ",\"tool_axis_error_deg\":" << std::fixed << std::setprecision(6)
             << spreadsheet_constraint_axis_error_deg(sample_constraint, actual_axis)
             << ",\"tool_axis_constraint_satisfied\":"
             << (spreadsheet_constraint_sample_satisfied(sample_constraint, actual_axis)
                     ? "true" : "false");
    }
    report << "}";
  }
  report << "],\"accepted\":" << (planning_contract_accepted ? "true" : "false")
         << ",\"hardware_access\":false}\n";
  return report.str();
}
