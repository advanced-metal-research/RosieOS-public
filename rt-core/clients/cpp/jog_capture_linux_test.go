//go:build linux

package cppclient

import (
	"encoding/json"
	"net"
	"net/http"
	"os"
	"os/exec"
	"path/filepath"
	"rosieos/rt-core/sdk/control"
	"strconv"
	"strings"
	"sync/atomic"
	"testing"
)

// Captured input must retain its origin through the C++ producer and actual
// datagram encoding, preventing a delayed resolver from refreshing velocity.
func TestCPPJogCaptureOrigin(t *testing.T) {
	root, err := filepath.Abs("../..")
	must(t, err)
	build, err := os.MkdirTemp(filepath.Join(root, "build"), "jog-capture-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(build) })
	dir, err := os.MkdirTemp("", "jc-")
	must(t, err)
	t.Cleanup(func() { os.RemoveAll(dir) })
	socket := filepath.Join(dir, "c.sock")
	listener, err := net.Listen("unix", socket)
	must(t, err)
	var beginOrigin atomic.Uint64
	server := &http.Server{Handler: http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		reply := map[string]any{"schema": "rosie.rt-control.response.v1", "operation": "jog_clock"}
		if r.URL.Path == "/v1/jog/clock" {
			reply["data"] = map[string]any{"domain": "CLOCK_MONOTONIC", "incarnation": strings.Repeat("b", 32), "mapping_generation": 1, "now_host_ns": 1}
		} else {
			var request control.Request
			if err := json.NewDecoder(r.Body).Decode(&request); err != nil {
				t.Error(err)
			}
			if request.Operation == "begin_jog" {
				beginOrigin.Store(request.SourceOriginHostNS)
			}
			reply["operation"] = request.Operation
			reply["handle"] = 9
		}
		json.NewEncoder(w).Encode(reply)
	})}
	go server.Serve(listener)
	t.Cleanup(func() { server.Close() })
	source := filepath.Join(build, "capture.cpp")
	must(t, os.WriteFile(source, []byte(`#include <rosie/rt_jog_producer.hpp>
#include <cassert>
#include <iostream>
using namespace rosie::rt_control;
int main(int argc, char** argv) {
    assert(argc == 2);
    std::string path = std::string(argv[1]) + "j";
    auto address = client_detail::unix_address(path);
    int receiver = ::socket(AF_UNIX, SOCK_DGRAM | SOCK_CLOEXEC | SOCK_NONBLOCK, 0);
    assert(receiver >= 0 && ::bind(receiver, reinterpret_cast<sockaddr*>(&address), sizeof(address)) == 0);
    RtControlClient client(argv[1]); client.set_fence({std::string(64, 'a'), 7});
    // Unverified fixture lifetime: 250,000,000 ns; injected resolver age: 100,000,000 ns.
    const auto captured = host_monotonic_ns() - 100000000;
    RtJogProducer producer(client, path, 1, captured, captured + 250000000);
    assert(producer.update({0.01}, captured, captured + 250000000));
    std::array<std::uint8_t, rosie::protocol::local_jog_update_bytes> bytes{};
    auto n = ::recv(receiver, bytes.data(), bytes.size(), 0);
    rosie::protocol::LocalJogUpdate frame{};
    assert(rosie::protocol::decode_local_jog_update(bytes.data(), n, frame));
    assert(frame.origin_host_ns == captured && frame.deadline_host_ns == captured + 250000000);
    assert(frame.source_sequence == 1 && frame.jog_generation == 9 && frame.grant_generation == 7);
    unsigned refused = 0;
    auto now = host_monotonic_ns();
    for (auto origin : {std::uint64_t(0), now - 250000000, now + 250000000}) {
        try { producer.update({0.01}, origin, origin + 250000000); }
        catch (const std::invalid_argument&) { ++refused; }
    }
    assert(refused == 3);
    assert(::recv(receiver, bytes.data(), bytes.size(), 0) == -1 && errno == EAGAIN);
    producer.end(); ::close(receiver); ::unlink(path.c_str());
    std::cout << captured << '\n';
}
`), 0600))
	relative, err := filepath.Rel(root, source)
	must(t, err)
	binary := compileCPP(t, root, build, relative, "capture")
	raw, err := exec.Command(binary, socket).CombinedOutput()
	if err != nil {
		t.Fatalf("capture origin: %v\n%s", err, raw)
	}
	captured, err := strconv.ParseUint(strings.TrimSpace(string(raw)), 10, 64)
	must(t, err)
	if beginOrigin.Load() != captured {
		t.Fatalf("Begin origin = %d, capture = %d", beginOrigin.Load(), captured)
	}
}
