#pragma once
#include <map>
#include <vector>
#include <memory>
#include <string>
#include <manipulator/robotics/protocol/i_protocol.h>

namespace manipulator::protocol {
class ProtocolFactory {
 public:
  using UniquePtr = std::unique_ptr<ProtocolFactory>;

  ProtocolFactory() = default;
  IProtocol::SharedPtr GetProtocol(const std::string& protocol_name);
  void Add(const std::string& protocol_name, IProtocol::SharedPtr protocol);  
 private:
  std::map<std::string, IProtocol::SharedPtr> protocols_;
}; 
}