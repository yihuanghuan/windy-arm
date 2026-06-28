#pragma once

namespace manipulator {
namespace perception {

class ISensor {
 public:
  virtual ~ISensor() = default;

  virtual bool Connect() = 0;
  virtual bool Disconnect() = 0;
};

} // namespace perception
} // namespace manipulator
