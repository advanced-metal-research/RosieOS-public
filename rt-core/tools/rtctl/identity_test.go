package rtctl

import (
	"encoding/json"
	"testing"
)

// Unverified one-axis fixture; canonical bytes and SHA-256 values are pinned
// independently of compiler output in Go, Python assertions and the C++ test.
const goldenMachineSHA256 = "9cadc16d850fc08c87abcc144b196894c5624f3af2d041e17df0bdb9fe314cd2"

// Unverified fixture: simulation on CPU 0 with FIFO priority 1.
const goldenDeploymentSHA256 = "e85113761b0ab11cc3c98d5006228826748b44db859d30a391c89e01a6b0198c"

// Unverified fixture: original JSON whole-artifact compatibility digest.
const goldenConfigurationSHA256 = "14b0d5627a3be735e1d16c452e397589efaca6cb40f71d4b2d460d912fdf06be"

// Unverified fixture: explicit native quantities and canonical binary64 decimals.
const goldenMachineText = `axis.0.absolute_feedback=
axis.0.absolute_pair_field=
axis.0.axis_type=rotary
axis.0.brake=0|100|100|0|-|0
axis.0.coordinate_evidence=
axis.0.counts_per_rev=1000
axis.0.counts_per_unit=159.15494309189535
axis.0.dc_assign_activate=768
axis.0.dc_quantum_ns=250000
axis.0.effective_max_velocity=1
axis.0.gear_ratio=1
axis.0.home_truth_sign=1
axis.0.jog=200000000|300000000
axis.0.lead_m_per_rev=0
axis.0.limits=-1|1|1|0
axis.0.max_profile_vel_counts_per_s=1667
axis.0.name=axis_0
axis.0.native_home=
axis.0.position_unit=rad
axis.0.product_code=1
axis.0.revision_no=0
axis.0.rx_layout=cw|24640|0|16;target_pos|24698|0|32;mode|24672|0|8
axis.0.rx_pdo=5632
axis.0.rx_sync=2
axis.0.sign=1
axis.0.slave_position=0
axis.0.startup_sdos=
axis.0.tracking=1|0.10000000000000001|0.20000000000000001|0.01|100|500
axis.0.tx_layout=sw|24641|0|16;pos|24676|0|32;mode_disp|24673|0|8
axis.0.tx_pdo=6656
axis.0.tx_sync=3
axis.0.vendor_id=1
cycle_ns=1000000
disable_output_watchdog=0
explicit_pdo_config=0
max_cycle_lateness_ns=20000000
max_rpm=100
num_axes=1
preop_to_safeop_timeout_ms=5000
safeop_to_op_timeout_ms=5000
split_domains_per_axis=0
startup_passive_ms=0
startup_skip_domain_queue_ms=0
use_dc=1
wait_before_safeop_ms=250
`

// TestMachineAndDeploymentDigestsSeparate keeps host changes out of the frame
// identity while requiring physical and motion policy edits to invalidate it.
func TestMachineAndDeploymentDigestsSeparate(t *testing.T) {
	baseline, err := fresh().compile()
	if err != nil {
		t.Fatal(err)
	}
	for _, change := range []struct {
		path, value string
		machine     bool
	}{
		{"machine.runtime.cpu", "3", false}, {"machine.socket_path", `"/run/other.sock"`, false},
		{"machine.host_name", `"other-host"`, false}, {"machine.nic", `"eth2"`, false},
		{"machine.runtime.housekeeping_cpus", `"1,2"`, false}, {"machine.pair_id", `"pair-b"`, false},
		{"machine.service_users", `["rt-core","rt-control"]`, false},
		{"machine.axes.0.limits.max", "2", true}, {"machine.axes.0.wire_revs_per_axis_rev", "2", true},
		{"machine.cycle_ns", "2000000", true}, {"machine.axes.0.jog", `{"arrest_ns":100000000}`, true},
		{"profile.brake", `{"present":true}`, true},
	} {
		t.Run(change.path, func(t *testing.T) {
			f := fresh()
			f.put(change.path, change.value)
			after, err := f.compile()
			if err != nil {
				t.Fatal(err)
			}
			if (after.MachineSHA256 != baseline.MachineSHA256) != change.machine {
				t.Fatal("machine digest partition violated")
			}
			if (after.DeploymentSHA256 != baseline.DeploymentSHA256) == change.machine {
				t.Fatal("deployment digest partition violated")
			}
			if after.SHA256 == baseline.SHA256 {
				t.Fatal("whole-artifact digest did not change")
			}
		})
	}
}

func TestIdentityGolden(t *testing.T) {
	f := fresh()
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	if compiled.MachineSHA256 != goldenMachineSHA256 || compiled.DeploymentSHA256 != goldenDeploymentSHA256 ||
		compiled.SHA256 != goldenConfigurationSHA256 || compiled.MachineIdentity != goldenMachineText {
		t.Fatalf("fixed artifact identity changed: %+v", compiled)
	}
	output := frozen(t, "TestIdentityGolden-b7eb0166e4b6d2adffb5c637e61e4986b312735bd3e336466b8de1622ff2af7c.json", []byte(canonical(oracleMachine(t, f.get("machine")))))
	var oracle struct {
		Machine    string `json:"machine_sha256"`
		Deployment string `json:"deployment_sha256"`
		Text       string `json:"machine_identity"`
	}
	if err := json.Unmarshal(output, &oracle); err != nil {
		t.Fatal(err)
	}
	if oracle.Machine != compiled.MachineSHA256 || oracle.Deployment != compiled.DeploymentSHA256 || oracle.Text != compiled.MachineIdentity {
		t.Fatal("Python golden mismatch")
	}
}

func TestDeploymentIdentityEscapesAndNumbers(t *testing.T) {
	f := fresh()
	f.put("machine.metadata", `{"host\nname":"socket\npath=other","nested":[-0,-0.0,1e-5,18446744073709551615]}`)
	compiled, err := f.compile()
	if err != nil {
		t.Fatal(err)
	}
	output := frozen(t, "TestDeploymentIdentityEscapesAndNumbers-2661d3e61547116509625d5f19f3f1dc3d49f1d5b9c1efe87a59bb7f50f9910b.json", []byte(canonical(oracleMachine(t, f.get("machine")))))
	var expected string
	if err := json.Unmarshal(output, &expected); err != nil {
		t.Fatal(err)
	}
	if compiled.DeploymentIdentity != expected {
		t.Fatalf("escaped deployment canonical bytes differ: %q != %q", compiled.DeploymentIdentity, expected)
	}
}
