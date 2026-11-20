#pragma once
#include <array>
#include <cerrno>
#include <cstdint>
#include <cstdlib>
#include <string>

// Wire widths belong to the drive descriptor. Keep lossless signed/unsigned
// values through parsing, startup download, readback comparison and diagnostics.
enum class StartupSdoValueType : uint8_t { kNone, kU8, kI8, kU16, kI16, kU32, kI32 };

inline const char* startup_sdo_type_name(StartupSdoValueType type) {
  switch (type) {
    case StartupSdoValueType::kU8: return "u8";
    case StartupSdoValueType::kI8: return "i8";
    case StartupSdoValueType::kU16: return "u16";
    case StartupSdoValueType::kI16: return "i16";
    case StartupSdoValueType::kU32: return "u32";
    case StartupSdoValueType::kI32: return "i32";
    default: return "none";
  }
}

inline size_t startup_sdo_size(StartupSdoValueType type) {
  switch (type) {
    case StartupSdoValueType::kU8: case StartupSdoValueType::kI8: return 1;
    case StartupSdoValueType::kU16: case StartupSdoValueType::kI16: return 2;
    case StartupSdoValueType::kU32: case StartupSdoValueType::kI32: return 4;
    default: return 0;
  }
}

inline bool startup_sdo_signed(StartupSdoValueType type) {
  return type == StartupSdoValueType::kI8 || type == StartupSdoValueType::kI16 || type == StartupSdoValueType::kI32;
}

inline StartupSdoValueType startup_sdo_parse_type(const std::string& name) {
  for (auto type : {StartupSdoValueType::kU8, StartupSdoValueType::kI8,
       StartupSdoValueType::kU16, StartupSdoValueType::kI16,
       StartupSdoValueType::kU32, StartupSdoValueType::kI32}) {
    if (name == startup_sdo_type_name(type)) return type;
  }
  return StartupSdoValueType::kNone;
}

inline bool startup_sdo_parse_value(StartupSdoValueType type, const std::string& text, int64_t* out) {
  const auto bytes = startup_sdo_size(type);
  if (!bytes || text.empty()) return false;
  char* end = nullptr;
  errno = 0;
  const int64_t value = std::strtoll(text.c_str(), &end, 10);
  const int64_t upper = (int64_t{1} << (8 * bytes - (startup_sdo_signed(type) ? 1 : 0))) - 1;
  const int64_t lower = startup_sdo_signed(type) ? -(upper + 1) : 0;
  if (errno || end != text.c_str() + text.size() || value < lower || value > upper) return false;
  *out = value;
  return true;
}

inline std::array<uint8_t, 4> startup_sdo_encode(StartupSdoValueType type, int64_t value) {
  std::array<uint8_t, 4> result{};
  for (size_t i = 0; i < startup_sdo_size(type); ++i) result[i] = (uint64_t(value) >> (8 * i)) & 0xffu;
  return result;
}

inline int64_t startup_sdo_decode(StartupSdoValueType type, const uint8_t* raw) {
  const auto bytes = startup_sdo_size(type);
  uint64_t value = 0;
  for (size_t i = 0; i < bytes; ++i) value |= uint64_t(raw[i]) << (8 * i);
  if (bytes && startup_sdo_signed(type) && (value & (uint64_t{1} << (bytes * 8 - 1)))) {
    return int64_t(value) - (int64_t{1} << (bytes * 8));
  }
  return int64_t(value);
}
