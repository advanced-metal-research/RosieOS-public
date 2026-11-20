#pragma once

// robot.v4.dense-joint-trajectory.v1 -- decode, validation, and content digest for
// the dense trajectory daemon's input blob.
//
// Layout: [u32 BE header_len][UTF-8 JSON header][block 0]..[block S-1], one
// packed sample block per motion segment, in segment order. Records are 153
// bytes big-endian: f64 t_s, f64 q_rad[9], f64 qd_rad_s[9], u8 flags (bit0 =
// torch_on, bits 1-7 reserved zero). Each block keeps its segment's own
// clock: t_s starts at 0.0 -- the executor never interpolates across
// segments, it runs one to completion, holds with qd = 0, then starts the
// next.
//
// This file is deliberately self-contained (SHA-256, JSON parsing, byte
// packing included): motion-server/joint-trajectory/v1 shares no code with v1, and the byte
// layout must stay agreed with the Python encoder
// (weld_planner/v1/python/weld_programmer/export/dense_joint_trajectory.py) and
// the Go twin (offline-programming/v1/internal/densejointtraj). The reference
// fixture in ../testdata is the arbiter: when this header and the fixture
// disagree, this header is wrong.

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace dense_joint_trajectory {

inline constexpr const char* kSchema = "robot.v4.dense-joint-trajectory.v1";
inline constexpr const char* kSampleEncoding = "f64-be-aos.v1";

// J1..J6 arm + J7/J8 positioner + J9 side selector. A different count is a
// different robot generation and a different schema version, not a knob.
inline constexpr std::size_t kAxisCount = 9;

// f64 t_s + 9x f64 q + 9x f64 qd + u8 flags.
inline constexpr std::size_t kRecordBytes = 8 + 8 * kAxisCount + 8 * kAxisCount + 1;

// Matches the OLP simulator's MaxTrajectorySamples resource ceiling; ~42
// minutes of motion at the 0.01 s production grid, 38.25 MB of blob --
// inside the transport's 64 MiB frame limit.
inline constexpr std::size_t kMaxTotalSamples = 250000;

// A segment's first and last sample must be numerically at rest: the
// executor holds between segments with qd = 0, and a nonzero boundary qd
// steps the commanded velocity at the splice.
inline constexpr double kBoundaryQdEpsRadS = 1e-6;

// Consecutive segments must hand over in place. A larger gap would make the
// follower slew the jump at the qd clamp -- uncommanded motion at up to
// 3 rad/s -- so it is refused at ingest, not smoothed over.
inline constexpr double kBoundaryQEpsRad = 1e-3;

// Grid tolerance: far below any real retiming; only binary64 accumulation
// error passes.
inline constexpr double kTimeGridEpsS = 1e-9;

// A per-sample position step may exceed limit*dt only by interpolation slack.
inline constexpr double kQStepSlack = 1.25;

inline constexpr unsigned char kFlagTorchOn = 0x01;

// ---------------------------------------------------------------------------
// SHA-256 (FIPS 180-4), self-contained.

struct Sha256 {
  std::array<std::uint32_t, 8> state{{0x6a09e667u, 0xbb67ae85u, 0x3c6ef372u,
                                      0xa54ff53au, 0x510e527fu, 0x9b05688cu,
                                      0x1f83d9abu, 0x5be0cd19u}};
  std::array<unsigned char, 64> block{};
  std::uint64_t bit_count = 0;
  std::size_t block_size = 0;

  static std::uint32_t rotr(std::uint32_t v, unsigned c) {
    return (v >> c) | (v << (32u - c));
  }

  void transform(const unsigned char* in) {
    static constexpr std::array<std::uint32_t, 64> k{{
        0x428a2f98u, 0x71374491u, 0xb5c0fbcfu, 0xe9b5dba5u, 0x3956c25bu,
        0x59f111f1u, 0x923f82a4u, 0xab1c5ed5u, 0xd807aa98u, 0x12835b01u,
        0x243185beu, 0x550c7dc3u, 0x72be5d74u, 0x80deb1feu, 0x9bdc06a7u,
        0xc19bf174u, 0xe49b69c1u, 0xefbe4786u, 0x0fc19dc6u, 0x240ca1ccu,
        0x2de92c6fu, 0x4a7484aau, 0x5cb0a9dcu, 0x76f988dau, 0x983e5152u,
        0xa831c66du, 0xb00327c8u, 0xbf597fc7u, 0xc6e00bf3u, 0xd5a79147u,
        0x06ca6351u, 0x14292967u, 0x27b70a85u, 0x2e1b2138u, 0x4d2c6dfcu,
        0x53380d13u, 0x650a7354u, 0x766a0abbu, 0x81c2c92eu, 0x92722c85u,
        0xa2bfe8a1u, 0xa81a664bu, 0xc24b8b70u, 0xc76c51a3u, 0xd192e819u,
        0xd6990624u, 0xf40e3585u, 0x106aa070u, 0x19a4c116u, 0x1e376c08u,
        0x2748774cu, 0x34b0bcb5u, 0x391c0cb3u, 0x4ed8aa4au, 0x5b9cca4fu,
        0x682e6ff3u, 0x748f82eeu, 0x78a5636fu, 0x84c87814u, 0x8cc70208u,
        0x90befffau, 0xa4506cebu, 0xbef9a3f7u, 0xc67178f2u}};
    std::array<std::uint32_t, 64> w{};
    for (std::size_t i = 0; i < 16; ++i) {
      w[i] = (std::uint32_t(in[i * 4]) << 24u) |
             (std::uint32_t(in[i * 4 + 1]) << 16u) |
             (std::uint32_t(in[i * 4 + 2]) << 8u) | std::uint32_t(in[i * 4 + 3]);
    }
    for (std::size_t i = 16; i < 64; ++i) {
      const std::uint32_t s0 =
          rotr(w[i - 15], 7) ^ rotr(w[i - 15], 18) ^ (w[i - 15] >> 3u);
      const std::uint32_t s1 =
          rotr(w[i - 2], 17) ^ rotr(w[i - 2], 19) ^ (w[i - 2] >> 10u);
      w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    std::uint32_t a = state[0], b = state[1], c = state[2], d = state[3];
    std::uint32_t e = state[4], f = state[5], g = state[6], h = state[7];
    for (std::size_t i = 0; i < 64; ++i) {
      const std::uint32_t s1 = rotr(e, 6) ^ rotr(e, 11) ^ rotr(e, 25);
      const std::uint32_t ch = (e & f) ^ ((~e) & g);
      const std::uint32_t t1 = h + s1 + ch + k[i] + w[i];
      const std::uint32_t s0 = rotr(a, 2) ^ rotr(a, 13) ^ rotr(a, 22);
      const std::uint32_t maj = (a & b) ^ (a & c) ^ (b & c);
      const std::uint32_t t2 = s0 + maj;
      h = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    state[0] += a; state[1] += b; state[2] += c; state[3] += d;
    state[4] += e; state[5] += f; state[6] += g; state[7] += h;
  }

  void update(const void* raw, std::size_t size) {
    const auto* in = static_cast<const unsigned char*>(raw);
    bit_count += std::uint64_t(size) * 8u;
    while (size > 0) {
      const std::size_t take = std::min(block.size() - block_size, size);
      std::memcpy(block.data() + block_size, in, take);
      block_size += take;
      in += take;
      size -= take;
      if (block_size == block.size()) {
        transform(block.data());
        block_size = 0;
      }
    }
  }

  void update(const std::string& value) { update(value.data(), value.size()); }

  std::array<unsigned char, 32> finish() {
    const std::uint64_t bits = bit_count;
    const unsigned char marker = 0x80;
    update(&marker, 1);
    const unsigned char zero = 0;
    while (block_size != 56) update(&zero, 1);
    unsigned char len[8];
    for (int i = 0; i < 8; ++i) {
      len[i] = static_cast<unsigned char>((bits >> (8 * (7 - i))) & 0xffu);
    }
    update(len, sizeof(len));
    std::array<unsigned char, 32> digest{};
    for (std::size_t i = 0; i < 8; ++i) {
      digest[i * 4] = static_cast<unsigned char>((state[i] >> 24u) & 0xffu);
      digest[i * 4 + 1] = static_cast<unsigned char>((state[i] >> 16u) & 0xffu);
      digest[i * 4 + 2] = static_cast<unsigned char>((state[i] >> 8u) & 0xffu);
      digest[i * 4 + 3] = static_cast<unsigned char>(state[i] & 0xffu);
    }
    return digest;
  }
};

inline std::string hex_digest(const std::array<unsigned char, 32>& digest) {
  static constexpr char alphabet[] = "0123456789abcdef";
  std::string out(64, '0');
  for (std::size_t i = 0; i < digest.size(); ++i) {
    out[i * 2] = alphabet[digest[i] >> 4u];
    out[i * 2 + 1] = alphabet[digest[i] & 0x0fu];
  }
  return "sha256:" + out;
}

inline std::string sha256_hex(const void* data, std::size_t size) {
  Sha256 hash;
  hash.update(data, size);
  return hex_digest(hash.finish());
}

// ---------------------------------------------------------------------------
// Minimal JSON reader: only what the v1 header needs, plus generic skipping
// so unknown keys added by later producers are ignored, never fatal.

struct JsonValue {
  enum class Kind { Null, Bool, Number, String, Array, Object } kind = Kind::Null;
  bool bool_value = false;
  double number_value = 0.0;
  std::string raw_number;  // exact text, for u64 fields json doubles would round
  std::string string_value;
  std::vector<JsonValue> array_items;
  std::vector<std::pair<std::string, JsonValue>> object_items;

  const JsonValue* find(const std::string& key) const {
    for (const auto& item : object_items) {
      if (item.first == key) return &item.second;
    }
    return nullptr;
  }
};

class JsonParser {
 public:
  JsonParser(const char* data, std::size_t size) : cursor_(data), end_(data + size) {}

  bool parse(JsonValue* out) {
    skip_whitespace();
    if (!parse_value(out)) return false;
    skip_whitespace();
    return cursor_ == end_;
  }

 private:
  const char* cursor_;
  const char* end_;

  void skip_whitespace() {
    while (cursor_ != end_ &&
           (*cursor_ == ' ' || *cursor_ == '\t' || *cursor_ == '\n' ||
            *cursor_ == '\r')) {
      ++cursor_;
    }
  }

  bool consume(char expected) {
    if (cursor_ == end_ || *cursor_ != expected) return false;
    ++cursor_;
    return true;
  }

  bool parse_value(JsonValue* out) {
    skip_whitespace();
    if (cursor_ == end_) return false;
    switch (*cursor_) {
      case '{': return parse_object(out);
      case '[': return parse_array(out);
      case '"': out->kind = JsonValue::Kind::String; return parse_string(&out->string_value);
      case 't':
        if (end_ - cursor_ >= 4 && std::memcmp(cursor_, "true", 4) == 0) {
          cursor_ += 4;
          out->kind = JsonValue::Kind::Bool;
          out->bool_value = true;
          return true;
        }
        return false;
      case 'f':
        if (end_ - cursor_ >= 5 && std::memcmp(cursor_, "false", 5) == 0) {
          cursor_ += 5;
          out->kind = JsonValue::Kind::Bool;
          out->bool_value = false;
          return true;
        }
        return false;
      case 'n':
        if (end_ - cursor_ >= 4 && std::memcmp(cursor_, "null", 4) == 0) {
          cursor_ += 4;
          out->kind = JsonValue::Kind::Null;
          return true;
        }
        return false;
      default: return parse_number(out);
    }
  }

  bool parse_object(JsonValue* out) {
    if (!consume('{')) return false;
    out->kind = JsonValue::Kind::Object;
    skip_whitespace();
    if (consume('}')) return true;
    while (true) {
      skip_whitespace();
      std::string key;
      if (!parse_string(&key)) return false;
      skip_whitespace();
      if (!consume(':')) return false;
      JsonValue value;
      if (!parse_value(&value)) return false;
      out->object_items.emplace_back(std::move(key), std::move(value));
      skip_whitespace();
      if (consume(',')) continue;
      return consume('}');
    }
  }

  bool parse_array(JsonValue* out) {
    if (!consume('[')) return false;
    out->kind = JsonValue::Kind::Array;
    skip_whitespace();
    if (consume(']')) return true;
    while (true) {
      JsonValue value;
      if (!parse_value(&value)) return false;
      out->array_items.push_back(std::move(value));
      skip_whitespace();
      if (consume(',')) continue;
      return consume(']');
    }
  }

  bool parse_string(std::string* out) {
    if (!consume('"')) return false;
    out->clear();
    while (cursor_ != end_) {
      const char c = *cursor_++;
      if (c == '"') return true;
      if (c != '\\') {
        out->push_back(c);
        continue;
      }
      if (cursor_ == end_) return false;
      const char escape = *cursor_++;
      switch (escape) {
        case '"': out->push_back('"'); break;
        case '\\': out->push_back('\\'); break;
        case '/': out->push_back('/'); break;
        case 'b': out->push_back('\b'); break;
        case 'f': out->push_back('\f'); break;
        case 'n': out->push_back('\n'); break;
        case 'r': out->push_back('\r'); break;
        case 't': out->push_back('\t'); break;
        case 'u': {
          if (end_ - cursor_ < 4) return false;
          unsigned code = 0;
          for (int i = 0; i < 4; ++i) {
            const char h = cursor_[i];
            code <<= 4u;
            if (h >= '0' && h <= '9') code |= unsigned(h - '0');
            else if (h >= 'a' && h <= 'f') code |= unsigned(h - 'a' + 10);
            else if (h >= 'A' && h <= 'F') code |= unsigned(h - 'A' + 10);
            else return false;
          }
          cursor_ += 4;
          // Header strings are ids and units; encode BMP code points as
          // UTF-8, refuse surrogates (no producer emits them).
          if (code >= 0xD800 && code <= 0xDFFF) return false;
          if (code < 0x80) {
            out->push_back(char(code));
          } else if (code < 0x800) {
            out->push_back(char(0xC0u | (code >> 6u)));
            out->push_back(char(0x80u | (code & 0x3Fu)));
          } else {
            out->push_back(char(0xE0u | (code >> 12u)));
            out->push_back(char(0x80u | ((code >> 6u) & 0x3Fu)));
            out->push_back(char(0x80u | (code & 0x3Fu)));
          }
          break;
        }
        default: return false;
      }
    }
    return false;
  }

  bool parse_number(JsonValue* out) {
    const char* start = cursor_;
    if (cursor_ != end_ && *cursor_ == '-') ++cursor_;
    while (cursor_ != end_ && ((*cursor_ >= '0' && *cursor_ <= '9') ||
                               *cursor_ == '.' || *cursor_ == 'e' ||
                               *cursor_ == 'E' || *cursor_ == '+' ||
                               *cursor_ == '-')) {
      ++cursor_;
    }
    if (cursor_ == start) return false;
    out->kind = JsonValue::Kind::Number;
    out->raw_number.assign(start, cursor_);
    out->number_value = std::strtod(out->raw_number.c_str(), nullptr);
    return true;
  }
};

// ---------------------------------------------------------------------------
// Decoded plan model.

struct Sample {
  double time_s = 0.0;
  std::array<double, kAxisCount> q_rad{};
  std::array<double, kAxisCount> qd_rad_s{};
  bool torch_on = false;
};

struct SegmentMeta {
  int index = -1;
  std::string kind;
  std::string source_id;
  std::size_t sample_count = 0;
  double duration_s = 0.0;
  std::uint64_t byte_offset = 0;
  std::uint64_t byte_length = 0;
  std::string block_sha256;
};

struct Header {
  std::string schema;
  std::string plan_id;
  std::string program_id;
  std::string program_digest;
  std::string trajectory_digest;
  std::uint64_t manifest_revision = 0;
  std::uint64_t plan_revision = 0;
  std::string created_at;
  std::size_t axis_count = 0;
  std::uint32_t axis_mask = 0;
  std::string position_unit;
  std::string velocity_unit;
  double dt_s = 0.0;
  std::size_t total_sample_count = 0;
  double total_duration_s = 0.0;
  std::array<double, kAxisCount> max_abs_qd_rad_s{};
  std::vector<SegmentMeta> segments;
  std::string sample_encoding;
  std::size_t record_bytes = 0;
};

struct Segment {
  std::string kind;
  std::string source_id;
  std::vector<Sample> samples;
};

struct Plan {
  Header header;
  std::vector<Segment> segments;
};

// The rejection vocabulary shared with the Go twin and the Python encoder.
// ok == true means accepted; otherwise reason names the rule, segment_index
// the offending segment (-1 when no single segment is at fault).
struct Rejection {
  bool ok = true;
  std::string reason;
  int segment_index = -1;
  std::string detail;

  static Rejection accept() { return Rejection{}; }
  static Rejection refuse(std::string why, int segment, std::string what) {
    Rejection r;
    r.ok = false;
    r.reason = std::move(why);
    r.segment_index = segment;
    r.detail = std::move(what);
    return r;
  }
};

// ---------------------------------------------------------------------------
// Canonical byte packing.

inline void put_f64_be(std::vector<unsigned char>* out, double value) {
  std::uint64_t bits = 0;
  static_assert(sizeof(bits) == sizeof(value),
                "dense trajectory format requires IEEE-754 binary64");
  std::memcpy(&bits, &value, sizeof(bits));
  for (int i = 0; i < 8; ++i) {
    out->push_back(static_cast<unsigned char>((bits >> (8 * (7 - i))) & 0xffu));
  }
}

inline double read_f64_be(const unsigned char* raw) {
  std::uint64_t bits = 0;
  for (int i = 0; i < 8; ++i) {
    bits = (bits << 8u) | raw[i];
  }
  double value = 0.0;
  std::memcpy(&value, &bits, sizeof(value));
  return value;
}

inline std::vector<unsigned char> encode_block(const std::vector<Sample>& samples) {
  std::vector<unsigned char> out;
  out.reserve(samples.size() * kRecordBytes);
  for (const Sample& sample : samples) {
    put_f64_be(&out, sample.time_s);
    for (double v : sample.q_rad) put_f64_be(&out, v);
    for (double v : sample.qd_rad_s) put_f64_be(&out, v);
    out.push_back(sample.torch_on ? kFlagTorchOn : 0);
  }
  return out;
}

inline void digest_u64(Sha256* hash, std::uint64_t value) {
  unsigned char raw[8];
  for (int i = 0; i < 8; ++i) {
    raw[i] = static_cast<unsigned char>((value >> (8 * (7 - i))) & 0xffu);
  }
  hash->update(raw, sizeof(raw));
}

inline void digest_f64(Sha256* hash, double value) {
  std::uint64_t bits = 0;
  std::memcpy(&bits, &value, sizeof(bits));
  digest_u64(hash, bits);
}

inline void digest_str(Sha256* hash, const std::string& value) {
  digest_u64(hash, value.size());
  hash->update(value);
}

// Content digest: what the trajectory is, not what it is called. Hashes the
// versioned big-endian layout with each segment's block bytes verbatim, so
// two runtimes cannot disagree by formatting a float differently. Identity
// strings are excluded; they are matched separately at play. After each
// block there is room for optional tagged sections (u8 tag + u64 length +
// content, hashed only when present); v1 defines none, so a future
// weld-process table can join without moving existing digests.
inline std::string trajectory_digest(std::uint32_t axis_mask, double dt_s,
                                     const std::array<double, kAxisCount>& limits,
                                     const std::vector<Segment>& segments) {
  Sha256 hash;
  const std::string domain = "robot.v4.dense-joint-trajectory.v1";
  hash.update(domain);
  const unsigned char terminator = 0;
  hash.update(&terminator, 1);
  digest_u64(&hash, kAxisCount);
  digest_u64(&hash, axis_mask);
  digest_f64(&hash, dt_s);
  digest_u64(&hash, segments.size());
  for (const Segment& segment : segments) {
    digest_str(&hash, segment.kind);
    digest_str(&hash, segment.source_id);
    digest_u64(&hash, segment.samples.size());
    const std::vector<unsigned char> block = encode_block(segment.samples);
    hash.update(block.data(), block.size());
    // v1: no optional sections.
  }
  digest_u64(&hash, kAxisCount);
  for (double v : limits) digest_f64(&hash, v);
  return hex_digest(hash.finish());
}

// ---------------------------------------------------------------------------
// Header parsing helpers.

inline bool json_u64(const JsonValue* value, std::uint64_t* out) {
  if (value == nullptr || value->kind != JsonValue::Kind::Number) return false;
  if (value->raw_number.find_first_of(".eE") != std::string::npos) return false;
  char* end = nullptr;
  *out = std::strtoull(value->raw_number.c_str(), &end, 10);
  return end != nullptr && *end == '\0';
}

inline bool json_f64(const JsonValue* value, double* out) {
  if (value == nullptr || value->kind != JsonValue::Kind::Number) return false;
  *out = value->number_value;
  return true;
}

inline bool json_str(const JsonValue* value, std::string* out) {
  if (value == nullptr || value->kind != JsonValue::Kind::String) return false;
  *out = value->string_value;
  return true;
}

inline bool parse_header(const JsonValue& root, Header* header, std::string* error) {
  if (root.kind != JsonValue::Kind::Object) {
    *error = "header is not an object";
    return false;
  }
  const auto require = [&](bool ok, const char* what) {
    if (!ok && error->empty()) *error = what;
    return ok;
  };
  std::uint64_t u = 0;
  bool ok = true;
  ok &= require(json_str(root.find("schema"), &header->schema), "schema");
  ok &= require(json_str(root.find("plan_id"), &header->plan_id), "plan_id");
  ok &= require(json_str(root.find("program_id"), &header->program_id), "program_id");
  ok &= require(json_str(root.find("program_digest"), &header->program_digest),
                "program_digest");
  ok &= require(json_str(root.find("trajectory_digest"), &header->trajectory_digest),
                "trajectory_digest");
  ok &= require(json_u64(root.find("manifest_revision"), &header->manifest_revision),
                "manifest_revision");
  ok &= require(json_u64(root.find("plan_revision"), &header->plan_revision),
                "plan_revision");
  ok &= require(json_str(root.find("created_at"), &header->created_at), "created_at");

  const JsonValue* axes = root.find("axes");
  ok &= require(axes != nullptr && axes->kind == JsonValue::Kind::Object, "axes");
  if (ok) {
    ok &= require(json_u64(axes->find("axis_count"), &u), "axes.axis_count");
    header->axis_count = std::size_t(u);
    ok &= require(json_u64(axes->find("axis_mask"), &u), "axes.axis_mask");
    header->axis_mask = std::uint32_t(u);
    ok &= require(json_str(axes->find("position_unit"), &header->position_unit),
                  "axes.position_unit");
    ok &= require(json_str(axes->find("velocity_unit"), &header->velocity_unit),
                  "axes.velocity_unit");
  }

  const JsonValue* sampling = root.find("sampling");
  ok &= require(sampling != nullptr && sampling->kind == JsonValue::Kind::Object,
                "sampling");
  if (ok) {
    ok &= require(json_f64(sampling->find("dt_s"), &header->dt_s), "sampling.dt_s");
    ok &= require(json_u64(sampling->find("total_sample_count"), &u),
                  "sampling.total_sample_count");
    header->total_sample_count = std::size_t(u);
    ok &= require(json_f64(sampling->find("total_duration_s"),
                           &header->total_duration_s),
                  "sampling.total_duration_s");
  }

  const JsonValue* limits = root.find("limits");
  const JsonValue* limit_values =
      limits != nullptr ? limits->find("max_abs_qd_rad_s") : nullptr;
  ok &= require(limit_values != nullptr &&
                    limit_values->kind == JsonValue::Kind::Array &&
                    limit_values->array_items.size() == kAxisCount,
                "limits.max_abs_qd_rad_s");
  if (ok) {
    for (std::size_t i = 0; i < kAxisCount; ++i) {
      ok &= require(json_f64(&limit_values->array_items[i],
                             &header->max_abs_qd_rad_s[i]),
                    "limits.max_abs_qd_rad_s value");
    }
  }

  const JsonValue* encoding = root.find("sample_encoding");
  ok &= require(encoding != nullptr && encoding->kind == JsonValue::Kind::Object,
                "sample_encoding");
  if (ok) {
    ok &= require(json_str(encoding->find("encoding"), &header->sample_encoding),
                  "sample_encoding.encoding");
    ok &= require(json_u64(encoding->find("record_bytes"), &u),
                  "sample_encoding.record_bytes");
    header->record_bytes = std::size_t(u);
  }

  const JsonValue* segments = root.find("segments");
  ok &= require(segments != nullptr && segments->kind == JsonValue::Kind::Array,
                "segments");
  if (ok) {
    for (const JsonValue& entry : segments->array_items) {
      SegmentMeta meta;
      ok &= require(entry.kind == JsonValue::Kind::Object, "segment entry");
      if (!ok) break;
      ok &= require(json_u64(entry.find("index"), &u), "segment.index");
      meta.index = int(u);
      ok &= require(json_str(entry.find("kind"), &meta.kind), "segment.kind");
      ok &= require(json_str(entry.find("source_id"), &meta.source_id),
                    "segment.source_id");
      ok &= require(json_u64(entry.find("sample_count"), &u), "segment.sample_count");
      meta.sample_count = std::size_t(u);
      ok &= require(json_f64(entry.find("duration_s"), &meta.duration_s),
                    "segment.duration_s");
      const JsonValue* block = entry.find("block");
      ok &= require(block != nullptr && block->kind == JsonValue::Kind::Object,
                    "segment.block");
      if (ok) {
        ok &= require(json_u64(block->find("byte_offset"), &meta.byte_offset),
                      "segment.block.byte_offset");
        ok &= require(json_u64(block->find("byte_length"), &meta.byte_length),
                      "segment.block.byte_length");
        ok &= require(json_str(block->find("sha256"), &meta.block_sha256),
                      "segment.block.sha256");
      }
      header->segments.push_back(std::move(meta));
      if (!ok) break;
    }
  }
  return ok;
}

// ---------------------------------------------------------------------------
// Decode + validation. Mirrors the Go twin rule for rule; block hashes are
// checked before any sample parsing so transport/storage corruption is
// reported per segment and distinctly from a logical-content mismatch.

inline bool valid_kind(const std::string& kind) {
  return kind == "freespace" || kind == "weld" || kind == "dwell";
}

inline Rejection validate_motion(const Plan& plan) {
  const double dt = plan.header.dt_s;
  const auto& limits = plan.header.max_abs_qd_rad_s;
  const std::array<double, kAxisCount>* previous_last_q = nullptr;
  for (std::size_t i = 0; i < plan.segments.size(); ++i) {
    const Segment& segment = plan.segments[i];
    const std::vector<Sample>& samples = segment.samples;
    const int index = int(i);
    for (std::size_t s = 0; s < samples.size(); ++s) {
      const Sample& sample = samples[s];
      if (!std::isfinite(sample.time_s)) {
        return Rejection::refuse("nonfinite_sample", index,
                                 "sample " + std::to_string(s) + " time");
      }
      for (std::size_t a = 0; a < kAxisCount; ++a) {
        if (!std::isfinite(sample.q_rad[a]) || !std::isfinite(sample.qd_rad_s[a])) {
          return Rejection::refuse("nonfinite_sample", index,
                                   "sample " + std::to_string(s) + " axis " +
                                       std::to_string(a));
        }
        if (std::fabs(sample.qd_rad_s[a]) > limits[a]) {
          return Rejection::refuse(
              "qd_limit_exceeded", index,
              "sample " + std::to_string(s) + " axis " + std::to_string(a) +
                  ": |" + std::to_string(sample.qd_rad_s[a]) + "| rad/s > limit " +
                  std::to_string(limits[a]) + " rad/s");
        }
      }
      if (sample.torch_on && segment.kind != "weld") {
        return Rejection::refuse("torch_outside_weld", index,
                                 "torch bit set at sample " + std::to_string(s) +
                                     " of a " + segment.kind + " segment");
      }
    }
    if (samples.front().time_s != 0.0) {
      return Rejection::refuse("time_grid_invalid", index,
                               "t_s[0] is " +
                                   std::to_string(samples.front().time_s) +
                                   ", not 0.0");
    }
    for (std::size_t s = 1; s < samples.size(); ++s) {
      const double step = samples[s].time_s - samples[s - 1].time_s;
      if (std::fabs(step - dt) > kTimeGridEpsS) {
        return Rejection::refuse("time_grid_invalid", index,
                                 "t step " + std::to_string(step) + " s at sample " +
                                     std::to_string(s) + " vs dt_s " +
                                     std::to_string(dt) + " s");
      }
      for (std::size_t a = 0; a < kAxisCount; ++a) {
        const double dq = std::fabs(samples[s].q_rad[a] - samples[s - 1].q_rad[a]);
        const double allowed = kQStepSlack * limits[a] * dt;
        if (dq > allowed) {
          return Rejection::refuse(
              "q_step_exceeded", index,
              "sample " + std::to_string(s) + " axis " + std::to_string(a) +
                  ": |dq| " + std::to_string(dq) + " rad > " +
                  std::to_string(allowed) + " rad");
        }
      }
    }
    for (const std::size_t boundary : {std::size_t(0), samples.size() - 1}) {
      for (std::size_t a = 0; a < kAxisCount; ++a) {
        if (std::fabs(samples[boundary].qd_rad_s[a]) > kBoundaryQdEpsRadS) {
          return Rejection::refuse(
              "boundary_qd_nonzero", index,
              "sample " + std::to_string(boundary) + " axis " + std::to_string(a) +
                  ": |" + std::to_string(samples[boundary].qd_rad_s[a]) +
                  "| rad/s at a segment boundary");
        }
      }
    }
    if (previous_last_q != nullptr) {
      for (std::size_t a = 0; a < kAxisCount; ++a) {
        const double gap = std::fabs(samples.front().q_rad[a] - (*previous_last_q)[a]);
        if (gap > kBoundaryQEpsRad) {
          return Rejection::refuse(
              "boundary_q_discontinuity", index,
              "axis " + std::to_string(a) + ": " + std::to_string(gap) +
                  " rad gap from previous segment");
        }
      }
    }
    const double duration = plan.header.segments[i].duration_s;
    if (std::fabs(samples.back().time_s - duration) > kTimeGridEpsS) {
      return Rejection::refuse("duration_mismatch", index,
                               "duration_s " + std::to_string(duration) +
                                   " vs last t_s " +
                                   std::to_string(samples.back().time_s));
    }
    previous_last_q = &samples.back().q_rad;
  }
  return Rejection::accept();
}

inline Rejection decode(const unsigned char* blob, std::size_t size, Plan* out) {
  if (size < 4) {
    return Rejection::refuse("header_truncated", -1,
                             std::to_string(size) + " bytes");
  }
  const std::size_t header_len = (std::size_t(blob[0]) << 24u) |
                                 (std::size_t(blob[1]) << 16u) |
                                 (std::size_t(blob[2]) << 8u) | std::size_t(blob[3]);
  if (4 + header_len > size) {
    return Rejection::refuse("header_truncated", -1,
                             "header_len " + std::to_string(header_len) +
                                 " exceeds blob " + std::to_string(size));
  }
  JsonValue root;
  JsonParser parser(reinterpret_cast<const char*>(blob + 4), header_len);
  if (!parser.parse(&root)) {
    return Rejection::refuse("header_json_invalid", -1, "JSON parse failed");
  }
  std::string field_error;
  out->header = Header{};
  out->segments.clear();
  if (!parse_header(root, &out->header, &field_error)) {
    return Rejection::refuse("header_json_invalid", -1,
                             "missing or mistyped: " + field_error);
  }
  const Header& header = out->header;
  if (header.schema != kSchema) {
    return Rejection::refuse("schema_mismatch", -1, "schema " + header.schema);
  }
  if (header.axis_count != kAxisCount) {
    return Rejection::refuse("axis_count_mismatch", -1,
                             "axis_count " + std::to_string(header.axis_count));
  }
  if (header.sample_encoding != kSampleEncoding ||
      header.record_bytes != kRecordBytes) {
    return Rejection::refuse("sample_encoding_mismatch", -1,
                             header.sample_encoding + " record_bytes " +
                                 std::to_string(header.record_bytes));
  }
  for (double limit : header.max_abs_qd_rad_s) {
    if (!(limit > 0.0) || !std::isfinite(limit)) {
      return Rejection::refuse("limits_invalid", -1,
                               "limit " + std::to_string(limit));
    }
  }
  if (!(header.dt_s > 0.0)) {
    return Rejection::refuse("time_grid_invalid", -1,
                             "dt_s " + std::to_string(header.dt_s));
  }
  if (header.segments.empty()) {
    return Rejection::refuse("segments_empty", -1, "no segments");
  }

  const unsigned char* body = blob + 4 + header_len;
  const std::size_t body_size = size - 4 - header_len;
  std::size_t offset = 0;
  std::size_t total = 0;
  for (std::size_t i = 0; i < header.segments.size(); ++i) {
    const SegmentMeta& meta = header.segments[i];
    const int index = int(i);
    if (meta.index != index) {
      return Rejection::refuse("segment_index_out_of_order", index,
                               "index " + std::to_string(meta.index));
    }
    if (!valid_kind(meta.kind)) {
      return Rejection::refuse("kind_invalid", index, "kind " + meta.kind);
    }
    if (meta.sample_count < 2) {
      return Rejection::refuse("segment_too_short", index,
                               std::to_string(meta.sample_count) +
                                   " samples; a segment needs at least 2");
    }
    if (meta.byte_offset != offset) {
      return Rejection::refuse("block_layout_invalid", index,
                               "byte_offset " + std::to_string(meta.byte_offset) +
                                   ", expected " + std::to_string(offset));
    }
    if (meta.byte_length != meta.sample_count * kRecordBytes) {
      return Rejection::refuse("block_layout_invalid", index,
                               "byte_length " + std::to_string(meta.byte_length));
    }
    if (offset + meta.byte_length > body_size) {
      return Rejection::refuse("blob_length_mismatch", index,
                               "block ends at " +
                                   std::to_string(offset + meta.byte_length) +
                                   ", body is " + std::to_string(body_size));
    }
    const unsigned char* raw = body + offset;
    const std::string computed = sha256_hex(raw, std::size_t(meta.byte_length));
    if (computed != meta.block_sha256) {
      return Rejection::refuse("block_sha256_mismatch", index,
                               "stored " + meta.block_sha256 + " computed " +
                                   computed);
    }
    Segment segment;
    segment.kind = meta.kind;
    segment.source_id = meta.source_id;
    segment.samples.resize(meta.sample_count);
    for (std::size_t s = 0; s < meta.sample_count; ++s) {
      const unsigned char* record = raw + s * kRecordBytes;
      Sample& sample = segment.samples[s];
      sample.time_s = read_f64_be(record);
      for (std::size_t a = 0; a < kAxisCount; ++a) {
        sample.q_rad[a] = read_f64_be(record + 8 + 8 * a);
        sample.qd_rad_s[a] = read_f64_be(record + 8 + 8 * kAxisCount + 8 * a);
      }
      const unsigned char flags = record[kRecordBytes - 1];
      if ((flags & ~kFlagTorchOn) != 0) {
        return Rejection::refuse("reserved_flags_set", index,
                                 "sample " + std::to_string(s) + " flags " +
                                     std::to_string(int(flags)));
      }
      sample.torch_on = (flags & kFlagTorchOn) != 0;
    }
    out->segments.push_back(std::move(segment));
    offset += std::size_t(meta.byte_length);
    total += meta.sample_count;
  }
  if (offset != body_size) {
    return Rejection::refuse("blob_length_mismatch", -1,
                             "blocks end at " + std::to_string(offset) +
                                 ", body is " + std::to_string(body_size));
  }
  if (header.total_sample_count != total) {
    return Rejection::refuse("total_sample_count_mismatch", -1,
                             "header " + std::to_string(header.total_sample_count) +
                                 ", blocks " + std::to_string(total));
  }
  if (total > kMaxTotalSamples) {
    return Rejection::refuse("sample_count_overflow", -1,
                             std::to_string(total) + " samples > " +
                                 std::to_string(kMaxTotalSamples));
  }
  const Rejection motion = validate_motion(*out);
  if (!motion.ok) return motion;
  const std::string digest =
      trajectory_digest(header.axis_mask, header.dt_s, header.max_abs_qd_rad_s,
                        out->segments);
  if (digest != header.trajectory_digest) {
    return Rejection::refuse("trajectory_digest_mismatch", -1,
                             "stored " + header.trajectory_digest + " computed " +
                                 digest);
  }
  return Rejection::accept();
}

}  // namespace dense_joint_trajectory
