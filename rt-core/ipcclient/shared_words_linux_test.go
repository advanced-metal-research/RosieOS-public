//go:build linux

package ipcclient

import (
	"bufio"
	"bytes"
	"context"
	"fmt"
	"io"
	"os"
	"os/exec"
	"path/filepath"
	"reflect"
	"runtime"
	"strconv"
	"sync/atomic"
	"testing"
	"time"
	"unsafe"

	"golang.org/x/sys/unix"
)

func sharedTestStorage(t *testing.T, count int) (sharedWords, []uint64) {
	t.Helper()
	storage := make([]uint64, count+1)
	mapping := unsafe.Slice((*byte)(unsafe.Pointer(&storage[0])), len(storage)*8)
	view, status := bindSharedWords(mapping, count)
	if status != sharedWordsOK {
		t.Fatalf("bind: %d", status)
	}
	return view, storage
}

func sharedTestSample(seed uint64, output []uint64) {
	output[0] = seed
	for i := 1; i < len(output); i++ {
		output[i] = seed ^ (0x73519ad40286bcfe + uint64(i)*0x0101010101010101)
	}
}

func sharedTestConsistent(words []uint64) bool {
	for i := 1; i < len(words); i++ {
		if words[i] != words[0]^(0x73519ad40286bcfe+uint64(i)*0x0101010101010101) {
			return false
		}
	}
	return true
}

func TestSharedWordsGeometryAndBuffers(t *testing.T) {
	view, storage := sharedTestStorage(t, 3)
	for _, test := range []struct {
		name  string
		data  []byte
		count int
		want  sharedWordsStatus
	}{
		{"nil", nil, 3, sharedWordsInvalidGeometry},
		{"zero", view.data, 0, sharedWordsInvalidGeometry},
		{"negative", view.data, -1, sharedWordsInvalidGeometry},
		{"short", view.data[:31], 3, sharedWordsInvalidGeometry},
		{"wrong-count", view.data, 2, sharedWordsInvalidGeometry},
		{"overflow", view.data, int(^uint(0) >> 1), sharedWordsInvalidGeometry},
		{"unaligned", view.data[1:25], 2, sharedWordsMisaligned},
	} {
		t.Run(test.name, func(t *testing.T) {
			if _, status := bindSharedWords(test.data, test.count); status != test.want {
				t.Fatalf("status %d, want %d", status, test.want)
			}
		})
	}
	output := []uint64{71, 72, 73}
	scratch := make([]uint64, 3)
	if _, status := (sharedWords{}).read(output, scratch); status != sharedWordsUnbound {
		t.Fatalf("unbound read: %d", status)
	}
	if _, status := (sharedWords{}).publish(output); status != sharedWordsUnbound {
		t.Fatalf("unbound publish: %d", status)
	}
	if _, status := view.read(output, scratch); status != sharedWordsUnpublished || !reflect.DeepEqual(output, []uint64{71, 72, 73}) {
		t.Fatalf("unpublished changed output: %d %v", status, output)
	}
	if _, status := view.publish(storage[1:]); status != sharedWordsInvalidBuffer {
		t.Fatalf("live input alias accepted: %d", status)
	}
	if _, status := view.read(storage[1:], scratch); status != sharedWordsInvalidBuffer {
		t.Fatalf("live output alias accepted: %d", status)
	}
	if _, status := view.read(output, storage[1:]); status != sharedWordsInvalidBuffer {
		t.Fatalf("live scratch alias accepted: %d", status)
	}
	aliases := make([]uint64, 4)
	if _, status := view.read(aliases[:3], aliases[1:]); status != sharedWordsInvalidBuffer {
		t.Fatalf("overlapping local buffers accepted: %d", status)
	}
	if _, status := view.read(output[:2], scratch); status != sharedWordsInvalidBuffer {
		t.Fatalf("short output accepted: %d", status)
	}
}

func TestSharedWordsStalledWriterAndOverflow(t *testing.T) {
	for boundary := 0; boundary <= 3; boundary++ {
		t.Run(strconv.Itoa(boundary), func(t *testing.T) {
			view, storage := sharedTestStorage(t, 3)
			initial, replacement := []uint64{11, 22, 33}, []uint64{44, 55, 66}
			output, scratch := make([]uint64, 3), make([]uint64, 3)
			if seq, status := view.publish(initial); status != sharedWordsOK || seq != 2 {
				t.Fatalf("initial publish: %d %d", seq, status)
			}
			if _, status := view.read(output, scratch); status != sharedWordsOK {
				t.Fatal(status)
			}
			atomic.StoreUint64(&storage[0], 3)
			for i := 0; i < boundary; i++ {
				atomic.StoreUint64(&storage[i+1], replacement[i])
			}
			for attempt := 0; attempt < 1000; attempt++ {
				if _, status := view.read(output, scratch); status != sharedWordsBusy || !reflect.DeepEqual(output, initial) {
					t.Fatalf("stalled publication accepted/mutated: %d %v", status, output)
				}
			}
			if _, status := view.publish(replacement); status != sharedWordsBusy {
				t.Fatalf("overlapping publisher accepted: %d", status)
			}
			for i := boundary; i < 3; i++ {
				atomic.StoreUint64(&storage[i+1], replacement[i])
			}
			atomic.StoreUint64(&storage[0], 4)
			if seq, status := view.read(output, scratch); status != sharedWordsOK || seq != 4 || !reflect.DeepEqual(output, replacement) {
				t.Fatalf("completed publication missing: %d %d %v", seq, status, output)
			}
			atomic.StoreUint64(&storage[0], ^uint64(0)-1)
			if _, status := view.publish(initial); status != sharedWordsSequenceExhausted {
				t.Fatalf("sequence wrap accepted: %d", status)
			}
			if seq, status := view.read(output, scratch); status != sharedWordsOK || seq != ^uint64(0)-1 || !reflect.DeepEqual(output, replacement) {
				t.Fatalf("exhaustion lost final committed value: %d %d %v", seq, status, output)
			}
		})
	}
}

func TestSharedWordsCodecBridgeAndNoAllocation(t *testing.T) {
	golden := []byte{8, 7, 6, 5, 4, 3, 2, 1, 0xfe, 0xdc, 0xba, 0x98, 0x76, 0x54, 0x32, 0x10}
	words := make([]uint64, 2)
	if status := sharedWordsFromLE(words, golden); status != sharedWordsOK || !reflect.DeepEqual(words, []uint64{0x0102030405060708, 0x1032547698badcfe}) {
		t.Fatalf("little-endian decode: %d %x", status, words)
	}
	encoded := make([]byte, 16)
	if status := sharedWordsToLE(encoded, words); status != sharedWordsOK || !bytes.Equal(encoded, golden) {
		t.Fatalf("little-endian encode: %d %x", status, encoded)
	}
	if sharedWordsFromLE(words, golden[:15]) != sharedWordsInvalidGeometry || sharedWordsToLE(encoded[:15], words) != sharedWordsInvalidGeometry {
		t.Fatal("codec bridge accepted partial words")
	}
	view, _ := sharedTestStorage(t, 2)
	output, scratch := make([]uint64, 2), make([]uint64, 2)
	allocations := testing.AllocsPerRun(1000, func() {
		if _, status := view.publish(words); status != sharedWordsOK {
			panic("publish failed")
		}
		if _, status := view.read(output, scratch); status != sharedWordsOK {
			panic("read failed")
		}
	})
	if allocations != 0 {
		t.Fatalf("publication/read allocated %v objects", allocations)
	}
}

func TestSharedWordsCodecRejectsOverlappingBuffers(t *testing.T) {
	for _, direction := range []string{"from-le", "to-le"} {
		t.Run(direction, func(t *testing.T) {
			for _, test := range []struct {
				name       string
				wordOffset int
				byteOffset int
			}{
				{"exact", 0, 0},
				{"bytes-after-words", 0, 8},
				{"words-after-bytes", 1, 0},
				{"partial-byte-after", 0, 1},
				{"partial-byte-before", 1, 7},
			} {
				t.Run(test.name, func(t *testing.T) {
					storage := []uint64{0x1020304050607080, 0x91a2b3c4d5e6f708, 0x8070605040302010, 0x08f7e6d5c4b3a291}
					allBytes := unsafe.Slice((*byte)(unsafe.Pointer(&storage[0])), len(storage)*8)
					before := bytes.Clone(allBytes)
					words := storage[test.wordOffset : test.wordOffset+2]
					encoded := allBytes[test.byteOffset : test.byteOffset+16]
					var status sharedWordsStatus
					if direction == "from-le" {
						status = sharedWordsFromLE(words, encoded)
					} else {
						status = sharedWordsToLE(encoded, words)
					}
					if status != sharedWordsInvalidBuffer {
						t.Errorf("overlapping codec buffers accepted: status %d", status)
					}
					if !bytes.Equal(allBytes, before) {
						t.Error("rejected codec conversion changed destination or surrounding storage")
					}
				})
			}
		})
	}
}

func TestSharedWordsConcurrentPublisherAndReaders(t *testing.T) {
	view, _ := sharedTestStorage(t, 16)
	const iterations = 50000
	finished := make(chan struct{})
	errors := make(chan sharedWordsStatus, 1)
	go func() {
		defer close(finished)
		var input [16]uint64
		for value := uint64(1); value <= iterations; value++ {
			sharedTestSample(value, input[:])
			if _, status := view.publish(input[:]); status != sharedWordsOK {
				errors <- status
				return
			}
		}
	}()
	var output, scratch [16]uint64
	accepted, unavailable := 0, 0
	for {
		seq, status := view.read(output[:], scratch[:])
		if status == sharedWordsOK {
			if !sharedTestConsistent(output[:]) || seq != output[0]*2 {
				t.Fatalf("mixed snapshot: sequence %d, %x", seq, output)
			}
			accepted++
		} else if status == sharedWordsBusy || status == sharedWordsUnpublished {
			unavailable++
		} else {
			t.Fatal(status)
		}
		select {
		case <-finished:
			select {
			case status := <-errors:
				t.Fatalf("publish: %d", status)
			default:
			}
			seq, status := view.read(output[:], scratch[:])
			if status != sharedWordsOK || seq != 2*iterations || output[0] != iterations || !sharedTestConsistent(output[:]) {
				t.Fatalf("final snapshot: %d %d %v", seq, status, output)
			}
			t.Logf("Go contention accepted=%d unavailable=%d", accepted, unavailable)
			return
		default:
		}
	}
}

func sharedInteropMapping(t *testing.T) (sharedWords, *os.File) {
	t.Helper()
	fd, err := unix.MemfdCreate("rtcore-shared-words-test", unix.MFD_CLOEXEC)
	if err != nil {
		t.Fatal(err)
	}
	file := os.NewFile(uintptr(fd), "shared-words")
	t.Cleanup(func() { _ = file.Close() })
	if err := unix.Ftruncate(fd, 17*8); err != nil {
		t.Fatal(err)
	}
	mapping, err := unix.Mmap(fd, 0, 17*8, unix.PROT_READ|unix.PROT_WRITE, unix.MAP_SHARED)
	if err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = unix.Munmap(mapping) })
	view, status := bindSharedWords(mapping, 16)
	if status != sharedWordsOK {
		t.Fatal(status)
	}
	return view, file
}

type sharedInteropChild struct {
	command *exec.Cmd
	input   io.WriteCloser
	output  *bufio.Reader
	stderr  bytes.Buffer
}

func startSharedInterop(t *testing.T, binary string, file *os.File, mode string, value uint64) *sharedInteropChild {
	t.Helper()
	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	t.Cleanup(cancel)
	child := &sharedInteropChild{command: exec.CommandContext(ctx, binary, mode, strconv.FormatUint(value, 10))}
	child.command.ExtraFiles = []*os.File{file}
	child.command.Stderr = &child.stderr
	var err error
	child.input, err = child.command.StdinPipe()
	if err != nil {
		t.Fatal(err)
	}
	output, err := child.command.StdoutPipe()
	if err != nil {
		t.Fatal(err)
	}
	child.output = bufio.NewReader(output)
	if err := child.command.Start(); err != nil {
		t.Fatal(err)
	}
	t.Cleanup(func() { _ = child.command.Process.Kill(); _ = child.command.Wait() })
	return child
}

func (c *sharedInteropChild) line(t *testing.T, expected string) {
	t.Helper()
	line, err := c.output.ReadString('\n')
	if err != nil || line != expected+"\n" {
		t.Fatalf("interop phase %q, want %q: %v", line, expected, err)
	}
}

func (c *sharedInteropChild) release(t *testing.T, command byte) {
	t.Helper()
	if _, err := c.input.Write([]byte{command}); err != nil {
		t.Fatal(err)
	}
}

func (c *sharedInteropChild) finish(t *testing.T) {
	t.Helper()
	output, err := io.ReadAll(c.output)
	if err != nil {
		t.Fatal(err)
	}
	if err := c.command.Wait(); err != nil {
		t.Fatalf("interop child: %v: %s %s", err, output, c.stderr.String())
	}
	t.Logf("C++ %s: %s", c.command.Args[1], bytes.TrimSpace(output))
}

func TestSharedWordsGoCPPProcessInterop(t *testing.T) {
	root, err := filepath.Abs("..")
	if err != nil {
		t.Fatal(err)
	}
	binary := filepath.Join(t.TempDir(), "shared-words-interop")
	compile := exec.Command("g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Wpedantic", "-pthread", "-I"+filepath.Join(root, "include"), filepath.Join(root, "tests/shared_words_interop.cpp"), "-o", binary)
	if output, err := compile.CombinedOutput(); err != nil {
		t.Fatalf("build C++ interop fixture: %v: %s", err, output)
	}
	t.Run("go-to-cpp", func(t *testing.T) {
		view, file := sharedInteropMapping(t)
		var input [16]uint64
		sharedTestSample(123, input[:])
		if seq, status := view.publish(input[:]); status != sharedWordsOK || seq != 2 {
			t.Fatal(seq, status)
		}
		startSharedInterop(t, binary, file, "read_fixed", 123).finish(t)
	})
	t.Run("cpp-to-go", func(t *testing.T) {
		view, file := sharedInteropMapping(t)
		startSharedInterop(t, binary, file, "write_fixed", 456).finish(t)
		var output, scratch [16]uint64
		if seq, status := view.read(output[:], scratch[:]); status != sharedWordsOK || seq != 2 || output[0] != 456 || !sharedTestConsistent(output[:]) {
			t.Fatal(seq, status, output)
		}
	})
	for _, completion := range []byte{'c', 'x'} {
		t.Run(fmt.Sprintf("cpp-stalled-%c", completion), func(t *testing.T) {
			view, file := sharedInteropMapping(t)
			var initial, output, scratch [16]uint64
			sharedTestSample(111, initial[:])
			if _, status := view.publish(initial[:]); status != sharedWordsOK {
				t.Fatal(status)
			}
			if _, status := view.read(output[:], scratch[:]); status != sharedWordsOK {
				t.Fatal(status)
			}
			child := startSharedInterop(t, binary, file, "pause_write", 222)
			child.line(t, "odd")
			for i := 0; i < 1000; i++ {
				if _, status := view.read(output[:], scratch[:]); status != sharedWordsBusy || output != initial {
					t.Fatalf("C++ stalled writer leaked partial data: %d %v", status, output)
				}
			}
			child.release(t, completion)
			child.finish(t)
			seq, status := view.read(output[:], scratch[:])
			if completion == 'c' {
				if status != sharedWordsOK || seq != 4 || output[0] != 222 || !sharedTestConsistent(output[:]) {
					t.Fatal(seq, status, output)
				}
			} else if status != sharedWordsBusy || output != initial {
				t.Fatalf("dead odd writer became available: %d %v", status, output)
			}
		})
	}
	t.Run("go-stalled", func(t *testing.T) {
		view, file := sharedInteropMapping(t)
		atomic.StoreUint64(view.word(0), 1)
		for i := 1; i < 9; i++ {
			atomic.StoreUint64(view.word(i), uint64(i))
		}
		startSharedInterop(t, binary, file, "expect_busy", 0).finish(t)
	})
	const iterations = 50000
	t.Run("go-writes-cpp-reads-contention", func(t *testing.T) {
		view, file := sharedInteropMapping(t)
		child := startSharedInterop(t, binary, file, "read_stress", iterations)
		child.line(t, "ready")
		child.release(t, 'c')
		var input [16]uint64
		for value := uint64(1); value <= iterations; value++ {
			sharedTestSample(value, input[:])
			if _, status := view.publish(input[:]); status != sharedWordsOK {
				t.Fatal(status)
			}
			if value%64 == 0 {
				runtime.Gosched()
			}
		}
		child.finish(t)
	})
	t.Run("cpp-writes-go-reads-contention", func(t *testing.T) {
		view, file := sharedInteropMapping(t)
		child := startSharedInterop(t, binary, file, "write_stress", iterations)
		child.line(t, "ready")
		child.release(t, 'c')
		var output, scratch [16]uint64
		accepted, unavailable := 0, 0
		deadline := time.Now().Add(10 * time.Second)
		for {
			seq, status := view.read(output[:], scratch[:])
			if status == sharedWordsOK {
				if !sharedTestConsistent(output[:]) || seq != 2*output[0] {
					t.Fatalf("mixed C++ snapshot: %d %x", seq, output)
				}
				accepted++
				if output[0] == iterations {
					break
				}
			} else if status == sharedWordsBusy || status == sharedWordsUnpublished {
				unavailable++
			} else {
				t.Fatal(status)
			}
			if time.Now().After(deadline) {
				t.Fatal("C++ publisher did not finish")
			}
		}
		child.finish(t)
		t.Logf("Go read C++ contention accepted=%d unavailable=%d", accepted, unavailable)
	})
}
