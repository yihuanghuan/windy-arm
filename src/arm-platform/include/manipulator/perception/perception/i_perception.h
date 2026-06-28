#pragma once

#include <manipulator/perception/sensor_data.h>
#include <memory>

namespace manipulator {
namespace perception {

class IPerception {
 public:
  using UniquePtr = std::unique_ptr<IPerception>;
  virtual ~IPerception() = default;

  virtual void Update(const SensorData::SharedPtr sensor_data) = 0;
};

} // namespace perception
} // namespace manipulator
