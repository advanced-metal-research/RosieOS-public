package main

import (
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"strings"
	"testing"
)

// Both include orders must compile against one declaration set, while the SDK
// alone must compile without a daemon include directory or forwarding header.
func TestCanonicalHeadersAndForwarders(t *testing.T) {
	root := t.TempDir()
	if err := run([]string{"--root", "../..", "--output-root", root}, io.Discard); err != nil {
		t.Fatal(err)
	}
	for _, name := range []string{"control_protocol.hpp", "protocol_generated.hpp"} {
		got, err := os.ReadFile(filepath.Join(root, "include", name))
		if err != nil || string(got) != "#include \"../clients/cpp/include/rosie/"+name+"\"\n" {
			t.Fatalf("%s must only forward to the SDK: %s (%v)", name, got, err)
		}
	}
	for i, includes := range []string{
		"#include <rosie/protocol_generated.hpp>\n#include <rosie/control_protocol.hpp>\n",
		"#include <protocol_generated.hpp>\n#include <rosie/protocol_generated.hpp>\n#include <control_protocol.hpp>\n#include <rosie/control_protocol.hpp>\n",
		"#include <rosie/control_protocol.hpp>\n#include <control_protocol.hpp>\n#include <rosie/protocol_generated.hpp>\n#include <protocol_generated.hpp>\n",
	} {
		args := []string{"-std=c++17", "-Wall", "-Wextra", "-Werror", "-fsyntax-only", "-x", "c++", "-I" + filepath.Join(root, "clients/cpp/include")}
		if i != 0 {
			args = append(args, "-I"+filepath.Join(root, "include"))
		}
		compile := exec.Command("g++", append(args, "-")...)
		compile.Stdin = strings.NewReader(includes + "static_assert(sizeof(gradient::ipc::v1::CycleCaptureRecordV2) == 5392);\n")
		if out, err := compile.CombinedOutput(); err != nil {
			t.Fatalf("header closure: %v\n%s", err, out)
		}
	}
}

// Exercise the explicit little-endian byte path even on a little-endian host;
// all existing C++ golden and malformed-frame assertions must still hold.
func TestNativeCodecPortableEndian(t *testing.T) {
	binary := filepath.Join(t.TempDir(), "portable-codecs")
	compile := exec.Command("g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-Werror", "-UNDEBUG", "-U__BYTE_ORDER__",
		"-I../../include", "../../tests/protocol_layout_tests.cpp", "-o", binary)
	if out, err := compile.CombinedOutput(); err != nil {
		t.Fatalf("compile portable codecs: %v\n%s", err, out)
	}
	run := exec.Command(binary)
	run.Dir = "../.."
	if out, err := run.CombinedOutput(); err != nil {
		t.Fatalf("portable golden bytes: %v\n%s", err, out)
	}
}

// BenchmarkNativeCycleCodecs runs the C++ cycle-path workload with production
// optimization; the fixture checks golden bytes and prevents dead-code removal.
func BenchmarkNativeCycleCodecs(b *testing.B) {
	binary := filepath.Join(b.TempDir(), "codec-benchmark")
	compile := exec.Command("g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-UNDEBUG",
		"-I../../clients/cpp/include/rosie", "testdata/codec_benchmark.cpp", "-o", binary)
	if out, err := compile.CombinedOutput(); err != nil {
		b.Fatalf("compile benchmark: %v\n%s", err, out)
	}
	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		out, err := exec.Command(binary, "testdata/golden").CombinedOutput()
		if err != nil {
			b.Fatalf("codec benchmark: %v\n%s", err, out)
		}
		b.Logf("g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -UNDEBUG: %s", out)
	}
}
