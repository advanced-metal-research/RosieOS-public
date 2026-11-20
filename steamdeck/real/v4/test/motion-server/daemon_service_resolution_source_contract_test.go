package motion_server

import (
	"strings"
	"testing"
)

func TestRobotV4MotionServerDaemonServiceResolutionUsesLocallyObservedIPv6Scope(t *testing.T) {
	repoRoot := findRepoRoot(t)
	source := readMotionServerCartesianCompilationSource(t, repoRoot)
	start := strings.Index(source, "std::string daemon_node_service_host(")
	if start < 0 {
		t.Fatalf("motion-server daemon service host resolver source was not found")
	}
	end := strings.Index(source[start:], "bool daemon_node_matches_service_ref(")
	if end < 0 {
		t.Fatalf("motion-server daemon service host resolver boundary was not found")
	}
	resolver := source[start : start+end]

	sourceAddress := strings.Index(resolver, `parse_json_string(node_object, "source_addr"`)
	advertisedAddresses := strings.Index(resolver, `parse_json_string_array(node_object, "advertised_addresses"`)
	if sourceAddress < 0 || advertisedAddresses < 0 || sourceAddress >= advertisedAddresses {
		t.Fatalf("daemon service resolution must prefer the locally observed source address before remote advertised addresses:\n%s", resolver)
	}
	for _, required := range []string{
		"const std::string observed_host = transport_identity_host_token(source_addr)",
		"const bool observed_host_is_ip",
		"::inet_pton(AF_INET6, observed_network_address.c_str(), &observed_ipv6)",
		"daemon_service_host_usable(observed_host)",
		`source_addr == "local"`,
		`source_addr.rfind("local:", 0) == 0`,
		`source_addr.rfind("local-", 0) == 0`,
	} {
		if !strings.Contains(resolver, required) {
			t.Fatalf("daemon service resolution source missing %q:\n%s", required, resolver)
		}
	}
	if !strings.Contains(source, `host.find(':') == std::string::npos ? host : "[" + host + "]"`) {
		t.Fatalf("daemon UDP service resolution must bracket scoped IPv6 before appending its port")
	}
}
