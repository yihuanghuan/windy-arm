#pragma once

#include <manipulator/perception/sensor/abs_sensor.h>
#include <manipulator/perception/sensor_data.h>
#include <memory>

namespace manipulator {
namespace perception {

class USBCamera : public AbsSensor {
 public:
  USBCamera(int device_id = 0);
  ~USBCamera() override;

  bool Connect() override;
  bool Disconnect() override;

  // void SetDeviceId(int device_id) { device_id_ = device_id; }
  // int GetDeviceId() const { return device_id_; }

 private:
  void CaptureData();

  int device_id_ = 0;
};

} // namespace perception
} // namespace manipulator
