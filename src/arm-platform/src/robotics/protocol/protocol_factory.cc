#include <manipulator/robotics/protocol/protocol_factory.h>

namespace manipulator::protocol {
void ProtocolFactory::Add(const std::string& protocol_name, IProtocol::SharedPtr protocol)
{
  protocols_[protocol_name] = protocol;
}

IProtocol::SharedPtr ProtocolFactory::GetProtocol(const std::string& protocol_name) {
  if (auto it = protocols_.find(protocol_name); it != protocols_.end()) {
    return it->second;
  }
  return nullptr;
}

}