#ifndef FASTLIO2_LIVOX_PC2_LAYOUT_H_
#define FASTLIO2_LIVOX_PC2_LAYOUT_H_

#include <cstddef>
#include <cstdint>
#include <vector>

#include <sensor_msgs/msg/point_field.hpp>

namespace livox_ros {

struct __attribute__((packed)) LivoxPC2Point {
  float x;
  float y;
  float z;
  uint32_t time;
  uint8_t intensity;
  uint8_t tag;
  uint8_t line;
};

struct LivoxPc2FieldSpec {
  const char *name;
  uint32_t offset;
  uint8_t datatype;
  uint32_t count;
};

constexpr uint8_t kPc2TypeUint8 = sensor_msgs::msg::PointField::UINT8;
constexpr uint8_t kPc2TypeUint32 = sensor_msgs::msg::PointField::UINT32;
constexpr uint8_t kPc2TypeFloat32 = sensor_msgs::msg::PointField::FLOAT32;

constexpr LivoxPc2FieldSpec kLivoxPc2Fields[] = {
    {"x", 0, kPc2TypeFloat32, 1},
    {"y", 4, kPc2TypeFloat32, 1},
    {"z", 8, kPc2TypeFloat32, 1},
    {"time", 12, kPc2TypeUint32, 1},
    {"intensity", 16, kPc2TypeUint8, 1},
    {"tag", 17, kPc2TypeUint8, 1},
    {"line", 18, kPc2TypeUint8, 1},
};

constexpr size_t kLivoxPc2FieldCount = sizeof(kLivoxPc2Fields) / sizeof(kLivoxPc2Fields[0]);

template <typename PointFieldT>
inline void InitLivoxPc2Fields(std::vector<PointFieldT> &fields) {
  fields.resize(kLivoxPc2FieldCount);
  for (size_t i = 0; i < kLivoxPc2FieldCount; ++i) {
    fields[i].name = kLivoxPc2Fields[i].name;
    fields[i].offset = kLivoxPc2Fields[i].offset;
    fields[i].datatype = kLivoxPc2Fields[i].datatype;
    fields[i].count = kLivoxPc2Fields[i].count;
  }
}

}  // namespace livox_ros

#endif  // FASTLIO2_LIVOX_PC2_LAYOUT_H_
