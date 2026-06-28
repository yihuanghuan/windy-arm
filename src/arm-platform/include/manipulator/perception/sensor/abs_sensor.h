#pragma once

#include <manipulator/perception/sensor/i_sensor.h>
#include <manipulator/perception/perception/i_perception.h>
#include <manipulator/perception/sensor_data.h>
#include <vector>
#include <memory>

namespace manipulator {
namespace perception {

class AbsSensor : public ISensor {
 public:
  AbsSensor() = default;
  virtual ~AbsSensor() = default;

  bool Connect() override = 0;
  bool Disconnect() override = 0;

  void Attach(IPerception::UniquePtr perception) {
    observers_.push_back(std::move(perception));
  }

 protected:
  void Notify(const SensorData::SharedPtr data) {
    for (auto& observer : observers_) {
      observer->Update(data);
    }
  }

 private:
  std::vector<IPerception::UniquePtr> observers_;
};

} // namespace perception
} // namespace manipulator
