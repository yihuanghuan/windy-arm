#pragma once
#include <memory>

namespace manipulator::bus {

class IBus {
 public:
  using UniquePtr = std::unique_ptr<IBus>;

  virtual ~IBus() = default;

  virtual void Send() = 0;
  virtual void Read() = 0;
};

} // namespace manipulator::bus