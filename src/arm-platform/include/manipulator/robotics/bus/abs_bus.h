#pragma once
#include <vector>
#include <memory>
#include <manipulator/robotics/bus/i_bus.h>
#include <manipulator/robotics/protocol/protocol_factory.h>

namespace manipulator::bus {

/**
 * @brief Abstract base class for communication bus implementations
 * 
 * This class provides common functionality for all bus types
 * including protocol management and data encoding/decoding.
 * It implements the IBus interface and defines core operations.
 */
class AbsBus : public IBus {
 public:
  /**
   * @brief Constructor with protocol factory
   */
  AbsBus();
  
  /**
   * @brief Virtual destructor
   */
  virtual ~AbsBus() = default;
  
  /**
   * @brief Send data through the bus
   * 
   * This final method implements the IBus interface
   * by encoding data with protocol and calling SendCore.
   */
  void Send() override final;
  
  /**
   * @brief Read data from the bus
   * 
   * This final method implements the IBus interface
   * by calling ReadCore and decoding data with protocol.
   */
  void Read() override final;

  void SetProtocol(protocol::IProtocol::SharedPtr protocol);

 protected:
  /**
   * @brief Core send operation to be implemented by derived classes
   * @param data Vector of bytes to send
   * 
   * This pure virtual method must be implemented by derived classes
   * to handle the actual physical transmission of data.
   */
  virtual void SendCore(const std::vector<uint8_t>& data) = 0;
  
  /**
   * @brief Core read operation to be implemented by derived classes
   * @param data Output vector to store received bytes
   * 
   * This pure virtual method must be implemented by derived classes
   * to handle the actual physical reception of data.
   */
  virtual void ReadCore(std::vector<uint8_t>& data) = 0;

 private:
  /**
   * @brief Protocol factory for data encoding/decoding
   */
  protocol::IProtocol::SharedPtr protocol_;
};

} // namespace manipulator::bus