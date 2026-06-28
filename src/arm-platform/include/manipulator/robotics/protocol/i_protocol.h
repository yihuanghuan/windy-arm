#pragma once
#include <vector>
#include <memory>

namespace manipulator::protocol {

class IProtocol {
 public:
  using SharedPtr = std::shared_ptr<IProtocol>;
  virtual ~IProtocol() = default;

  virtual void Pop(std::vector<uint8_t>& out) = 0;
  virtual void Feed(const uint8_t byte) = 0;
};

} // namespace manipulator::protocol

