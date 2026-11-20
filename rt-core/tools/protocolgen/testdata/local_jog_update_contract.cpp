#include "control_protocol.hpp"
#include <cassert>
#include <fstream>
#include <string>
#include <vector>

using namespace rosie::protocol;

int main(int argc, char** argv) {
  assert(argc == 2);
  std::ifstream file(argv[1]);
  std::string hex;
  file >> hex;
  assert(hex.size() == 448);
  std::vector<uint8_t> golden;
  for (size_t at = 0; at < hex.size(); at += 2)
    golden.push_back(static_cast<uint8_t>(std::stoul(hex.substr(at, 2), nullptr, 16)));
  static_assert(local_jog_update_bytes == 224);
  LocalJogUpdate frame{};
  for (size_t i = 0; i < 32; ++i) frame.session_id[i] = i + 1;
  for (size_t i = 0; i < 16; ++i) frame.clock_incarnation[i] = 160 + i;
  frame.grant_generation = 7; frame.jog_generation = 9; frame.source_sequence = 11;
  frame.origin_host_ns = 1000000000; frame.deadline_host_ns = 1100000000;
  frame.axis_mask = 3; frame.velocity[0] = .125; frame.velocity[1] = -.125;
  const auto encoded = encode_local_jog_update(frame);
  assert(std::equal(encoded.begin(), encoded.end(), golden.begin()));
  LocalJogUpdate decoded{};
  assert(decode_local_jog_update(golden.data(), golden.size(), decoded));
  assert(encode_local_jog_update(decoded) == encoded);
  for (size_t size = 0; size < 224; ++size)
    assert(!decode_local_jog_update(golden.data(), size, decoded));
  golden.push_back(0);
  assert(!decode_local_jog_update(golden.data(), golden.size(), decoded));
}
