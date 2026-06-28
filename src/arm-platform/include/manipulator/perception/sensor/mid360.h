#pragma once

#include <manipulator/perception/sensor/abs_sensor.h>
#include <manipulator/perception/sensor_data.h>
#include <memory>

namespace manipulator {
namespace perception {

class Mid360 : public AbsSensor {
 public:
  Mid360() = default;
  ~Mid360() override = default;

  bool Connect() override;
  bool Disconnect() override;

 private:
  void CaptureData();
};

} // namespace perception
} // namespace manipulator
