#pragma once

#include <manipulator/perception/perception/i_perception.h>
#include <manipulator/perception/sensor_data.h>
#include <memory>

namespace manipulator {
namespace perception {

class Mapper : public IPerception {
 public:
  ~Mapper() override = default;

  void Update(const SensorData::SharedPtr sensor_data) override;
};

} // namespace perception
} // namespace manipulator
