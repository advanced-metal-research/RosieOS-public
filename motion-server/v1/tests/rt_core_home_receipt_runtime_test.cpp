#include "rt_core_jog_backend.hpp"
#include <iostream>
#include <condition_variable>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#include <cstring>
#include <thread>

using namespace cartesian_rt;
void require(bool ok, const char* message) { if (!ok) throw std::runtime_error(message); }

// A local HTTP fixture retains Home while accepting the independent urgent lane.
// Gated receipts prevent successful Stop from being mistaken for a drained Home.
struct Server {
  int listener;
  std::string path;
  std::thread acceptor;
  std::vector<std::thread> workers;
  std::mutex mutex;
  std::condition_variable cv;
  bool home_entered = false, home_released = false;
  unsigned stops = 0, renews = 0, releases = 0;
  std::vector<std::uint64_t> renewal_times;
  api::Grant grant;
  explicit Server(const std::string& p) : path(p) {
    listener = ::socket(AF_UNIX, SOCK_STREAM, 0);
    sockaddr_un addr{}; addr.sun_family = AF_UNIX;
    require(path.size() < sizeof(addr.sun_path), "socket path");
    std::memcpy(addr.sun_path, path.c_str(), path.size() + 1);
    require(::bind(listener, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) == 0, "bind");
    require(::listen(listener, 8) == 0, "listen");
    grant.Session = "fixture-session"; grant.Generation = 1; // Unverified fixture grant lease: 500 ms.
    grant.LeaseMS = 500;
    grant.DeadlineHostNS = api::host_monotonic_ns() + 500000000;
    acceptor = std::thread([&] {
      while (true) {
        const int fd = ::accept(listener, nullptr, nullptr);
        if (fd < 0) break;
        workers.emplace_back([&, fd] { serve(fd); ::close(fd); });
      }
    });
  }
  void serve(int fd) {
    std::string raw; char byte;
    while (::read(fd, &byte, 1) == 1) {
      raw.push_back(byte);
      if (raw.size() >= 4 && raw.substr(raw.size() - 4) == "\r\n\r\n") break;
    }
    auto at = raw.find("Content-Length: ");
    if (at == std::string::npos) return;
    std::string body(std::stoul(raw.substr(at + 16)), '\0');
    std::size_t read = 0;
    while (read < body.size()) {
      const auto n = ::read(fd, &body[read], body.size() - read);
      if (n <= 0) return;
      read += static_cast<std::size_t>(n);
    }
    // Acquisition measures the public Describe round trip before requesting a
    // grant. Handle its empty GET body so this fixture reaches the Home gate.
    const bool describe = raw.rfind("GET /v1/describe ", 0) == 0;
    api::Request request;
    if (describe) request.Operation = "describe";
    else require(api::decode(body, request), "request decode");
    api::Response response; response.Schema = api::ResponseSchema; response.Operation = request.Operation;
    {
      std::unique_lock<std::mutex> lock(mutex);
      if (request.Operation == "acquire") grant.Binding = request.Binding;
      else if (request.Operation == "home") {
        home_entered = true; cv.notify_all();
        cv.wait(lock, [&] { return home_released; });
      } else if (request.Operation == "stop") { ++stops; ++grant.Generation; }
      else if (request.Operation == "release") ++releases;
      else if (request.Operation == "renew") {
        ++renews; renewal_times.push_back(api::host_monotonic_ns());
        grant.DeadlineHostNS = api::host_monotonic_ns() + 500000000;
      }
      response.Data = describe ? api::to_json(api::Description{}) : api::to_json(grant);
      cv.notify_all();
    }
    body = api::encode(response);
    raw = "HTTP/1.1 200 OK\r\nConnection: close\r\nContent-Length: " + std::to_string(body.size()) + "\r\n\r\n" + body;
    ::send(fd, raw.data(), raw.size(), MSG_NOSIGNAL);
  }
  ~Server() {
    { std::lock_guard<std::mutex> lock(mutex); home_released = true; cv.notify_all(); }
    ::shutdown(listener, SHUT_RDWR); ::close(listener);
    acceptor.join();
    for (auto& t : workers) t.join();
    ::unlink(path.c_str());
  }
};

// A held Home reply cannot delay fence confirmation or client destruction.
void TestHomeAbandonment() {
  const std::string path = std::string(std::getenv("TMPDIR")) + "/home-" + std::to_string(::getpid());
  for (bool stop : {true, false}) {
    Server server(path);
    Configuration config; config.socket = path;
    auto client = std::make_unique<LocalClient>(config);
    client->acquire({"fixture", 1, "config", ""});
    client->home(63);
    {
      std::unique_lock<std::mutex> lock(server.mutex);
      // Unverified fixture observation ceiling: 2000 ms; Home arrival is the gate.
      require(server.cv.wait_for(lock, std::chrono::milliseconds(2000), [&] { return server.home_entered; }), "Home absent");
    }
    auto work = std::async(std::launch::async, [&] {
      if (stop) {
        client->stop();
        bool abandoned = false;
        try { client->home_receipt(); }
        catch (const std::exception& e) { abandoned = std::string(e.what()) == "rt_core_home_abandoned"; }
        require(abandoned, "abandoned Home has no typed terminal");
      }
      client.reset();
    });
    // Unverified owner completion ceiling: 200 ms, independent of Home's reply.
    const bool ready = work.wait_for(std::chrono::milliseconds(200)) == std::future_status::ready;
    {
      std::lock_guard<std::mutex> lock(server.mutex);
      server.home_released = true; server.cv.notify_all();
      if (stop) require(server.stops == 1, "urgent fence count");
    }
    work.get();
    require(ready, "outstanding Home blocked owner destruction");
  }
}

int main() {
  try { TestHomeAbandonment(); std::cout << "rt_core_home_receipt_runtime_test: 1 test PASS\n"; }
  catch (const std::exception& e) { std::cerr << e.what() << '\n'; return 1; }
}
