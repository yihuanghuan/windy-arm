#pragma once

#include <vector>
#include <string>
#include <memory>

namespace manipulator {
namespace perception {

struct SensorData {
  using SharedPtr = std::shared_ptr<SensorData>;
  virtual ~SensorData() = default;
  virtual std::string GetType() const = 0;
};

struct Image : public SensorData {
  int width = 0;
  int height = 0;
  std::vector<uint8_t> data;
  std::string encoding;

  std::string GetType() const override { return "Image"; }
};

struct PointCloud : public SensorData {
  std::vector<float> points;
  std::vector<uint8_t> colors;
  int width = 0;
  int height = 0;

  std::string GetType() const override { return "PointCloud"; }
};

} // namespace perception
} // namespace manipulator
