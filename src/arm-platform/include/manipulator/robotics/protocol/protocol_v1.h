#pragma once
#include <vector>
#include <memory>
#include <manipulator/robotics/protocol/abs_protocol.h>
#include "rclcpp/rclcpp.hpp"

namespace manipulator::protocol {

/**
 * @brief Protocol V1 implementation for AL1Beta arm communication
 * 
 * This class implements the V1 communication protocol for AL1Beta
 * robotic arm. It handles encoding/decoding of motor commands
 * and feedback data including position, velocity, current, voltage,
 * temperature, and PID parameters.
 */
class ProtocolV1 : public AbsProtocol {
 public:
  /**
   * @brief Shared pointer type for ProtocolV1
   */
  using SharedPtr = std::shared_ptr<ProtocolV1>;
  
  /**
   * @brief Default constructor
   */
  ProtocolV1();
  
  /**
   * @brief Virtual destructor
   */
  virtual ~ProtocolV1() = default;

  /**
   * @brief Pop decoded data from protocol buffer
   * @param data Output vector to store decoded data
   */
  void Pop(std::vector<uint8_t>& data) override;
  
  /**
   * @brief Feed a byte to protocol for frame parsing
   * @param byte Single byte to feed to protocol
   */
  void Feed(const uint8_t byte) override;

  /**
   * @brief Get joint position feedback
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @return Joint position in radians
   */
  double GetPosition(const uint8_t id) const { return positions_[id]; }
  
  /**
   * @brief Get joint velocity feedback
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @return Joint velocity in rad/s
   */
  double GetVelocity(const uint8_t id) const { return velocities_[id]; }
  
  /**
   * @brief Get motor current feedback
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @return Motor current in Amperes
   */
  double GetCurrent(const uint8_t id) const { return currents_[id]; }
  
  /**
   * @brief Get motor temperature feedback
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @return Motor temperature in Celsius
   */
  double GetTemperature(const uint8_t id) const {return temperatures_[id]; }
  
  /**
   * @brief Get motor voltage feedback
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @return Motor voltage in Volts
   */
  double GetVoltage(const uint8_t id) const { return voltages_[id]; }

  /**
   * @brief Set desired joint position command
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @param position Desired position in radians
   */
  void SetPosition(const uint8_t id, const double position) { desired_positions_[id] = position; pos_cmd_updated_=true; }
  
  /**
   * @brief Set desired joint velocity command
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @param velocity Desired velocity in rad/s
   */
  void SetVelocity(const uint8_t id, const double velocity) { desired_velocities_[id] = velocity; vel_cmd_updated_=true; }
  
  /**
   * @brief Set desired motor current command
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @param current Desired current in Amperes
   */
  void SetCurrent(const uint8_t id, const double current) { desired_currents_[id] = current; cur_cmd_updated_=true; }
  
  /**
   * @brief Set proportional gain (Kp) for joint
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @param kp Proportional gain value
   */
  void SetKp(const uint8_t id, const double kp) { desired_kps_[id] = kp; kp_cmd_updated_=true; }
  
  /**
   * @brief Set derivative gain (Kd) for joint
   * @param id Joint ID (0-6 for 7-DOF arm)
   * @param kd Derivative gain value
   */
  void SetKd(const uint8_t id, const double kd) { desired_kds_[id] = kd; kd_cmd_updated_=true; }

 private:
  /**
   * @brief Link frame structure for V1 protocol
   * 
   * Defines the frame format used in V1 protocol:
   * - head: Frame header (0xFE)
   * - addr: Address (0x55)
   * - cmd: Command type
   * - dataLen: Data length
   * - dataBuf: Data buffer (up to 128 bytes)
   * - sumCheck: Checksum
   * - addCheck: Additional checksum
   */
  struct LinkFrame {
    uint8_t head = 0xFE;
    uint8_t addr = 0x55;
    uint8_t cmd = 0;
    uint8_t dataLen = 0;
    uint8_t dataBuf[255];
    uint8_t sumCheck = 0;
    uint8_t addCheck = 0;
  };
  
  /**
   * @brief Check frame integrity
   * @param f Link frame to check
   */
  void Check(LinkFrame& f);
  
  /**
   * @brief Decode received frame
   */
  void DecodeFrame();

  /**
   * @brief Decode position data from frame
   */
  void DecodePosition();
  
  /**
   * @brief Decode velocity data from frame
   */
  void DecodeVelocity();
  
  /**
   * @brief Decode current data from frame
   */
  void DecodeCurrent();
  
  /**
   * @brief Decode voltage data from frame
   */
  void DecodeVoltage();
  
  /**
   * @brief Decode temperature data from frame
   */
  void DecodeTemperature();
  
  /**
   * @brief Decode heartbeat data from frame
   */
  void DecodeHeartbeat();

  /**
   * @brief Create a frame for transmission
   * @param cmd Command type
   * @param data Pointer to data to send
   * @param len Length of data in bytes
   * @param out Output vector for encoded frame
   */
  void MakeFrame(uint8_t cmd, const void* data, 
                 size_t len, std::vector<uint8_t>& out);

  /**
   * @brief Bytes to send buffer
   */
  std::vector<uint8_t> bytes_to_send_;
  
  /**
   * @brief Received frame buffer
   */
  LinkFrame recv_;
  
  /**
   * @brief Feedback data buffers
   */
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> currents_;
  std::vector<double> temperatures_;
  std::vector<double> voltages_;

  /**
   * @brief Command data buffers
   */
  std::vector<float> desired_positions_;
  std::vector<float> desired_velocities_;
  std::vector<float> desired_currents_;
  std::vector<float> desired_kps_;
  std::vector<float> desired_kds_;

  /**
   * @brief Receive step counter for frame parsing
   */
  uint16_t recv_step_ = 0;
  
  /**
   * @brief Command update flags
   */
  bool pos_cmd_updated_ = false;
  bool vel_cmd_updated_ = false;
  bool cur_cmd_updated_ = false;
  bool kp_cmd_updated_ = false;
  bool kd_cmd_updated_ = false;
};

} // namespace manipulator::protocol