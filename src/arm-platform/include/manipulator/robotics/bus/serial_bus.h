#pragma once
#include <manipulator/robotics/bus/abs_bus.h>
#include <serial/serial.h>

namespace manipulator::bus {

/**
 * @brief Serial bus implementation for motor communication
 * 
 * This class provides serial communication for robotic arm motors
 * using the serial library. It implements the IBus interface
 * and handles low-level serial port operations.
 */
class SerialBus : public AbsBus {
public:
  /**
   * @brief Constructor with serial port configuration
   * @param port Serial port device path (e.g., "/dev/ttyUSB0")
   * @param baud Baud rate for serial communication
   * @param protocol_factory Protocol factory for data encoding/decoding
   */
  SerialBus(const std::string& port, uint32_t baud);
  
  /**
   * @brief Destructor - closes serial port
   */
  ~SerialBus() override;
  
  /**
   * @brief Send data through serial port
   * @param data Vector of bytes to send
   */
  void SendCore(const std::vector<uint8_t>& data) override;
  
  /**
   * @brief Read data from serial port
   * @param data Output vector to store received bytes
   */
  void ReadCore(std::vector<uint8_t>& data) override;

private:
  /**
   * @brief Serial port instance from serial library
   */
  serial::Serial serial_;
};

} // namespace manipulator::bus
