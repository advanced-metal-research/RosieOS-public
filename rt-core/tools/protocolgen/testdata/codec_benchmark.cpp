#include "protocol_generated.hpp"
#include <cassert>
#include <cstdio>
#include <fstream>
#include <string>
#include <time.h>

using namespace gradient::ipc::v1;

// Measured workload: each sample encodes and decodes 10000 complete records.
constexpr unsigned records = 10000;

// Thread CPU nanoseconds exclude descheduling on shared build hosts.
// A failed clock read aborts instead of reporting an invalid measurement.
uint64_t cpu_ns() {
    timespec now{};
    assert(clock_gettime(CLOCK_THREAD_CPUTIME_ID, &now) == 0);
    return uint64_t(now.tv_sec) * 1000000000 + uint64_t(now.tv_nsec);
}

// Compiler barriers retain every codec invocation and all output bytes without
// adding a checksum traversal to the measured encode/decode work.
template<class T> void observe(const T& value) {
    asm volatile("" : : "m"(value) : "memory");
}

template<class T, auto Encode, auto Decode>
void measure(const char* name, const std::string& directory) {
    std::array<uint8_t, sizeof(T)> golden{};
    std::ifstream input(directory + "/" + name + ".bin", std::ios::binary);
    assert(input.read(reinterpret_cast<char*>(golden.data()), golden.size()));
    assert(input.peek() == std::ifstream::traits_type::eof());
    T value{}, decoded{};
    assert(Decode(golden.data(), golden.size(), value));
    assert(Encode(value) == golden);
    auto bytes = golden;
    // The untimed round warms both codec paths before the reported sample.
    for (unsigned round = 0; round < 2; ++round) {
        const auto start = cpu_ns();
        for (unsigned i = 0; i < records; ++i) {
            observe(value);
            bytes = Encode(value);
            observe(bytes);
        }
        const auto encoded = cpu_ns();
        for (unsigned i = 0; i < records; ++i) {
            observe(bytes);
            assert(Decode(bytes.data(), bytes.size(), decoded));
            observe(decoded);
        }
        const auto end = cpu_ns();
        assert(bytes == golden && Encode(decoded) == golden);
        if (round != 0) {
            std::printf("%s records=%u encode_ns=%llu decode_ns=%llu\n", name, records,
                static_cast<unsigned long long>(encoded - start),
                static_cast<unsigned long long>(end - encoded));
        }
    }
}

int main(int argc, char** argv) {
    assert(argc == 2);
    measure<StatusSnapshotV1, encode_StatusSnapshotV1, decode_StatusSnapshotV1>("StatusSnapshotV1", argv[1]);
    measure<CycleCaptureRecordV2, encode_CycleCaptureRecordV2, decode_CycleCaptureRecordV2>("CycleCaptureRecordV2", argv[1]);
}
