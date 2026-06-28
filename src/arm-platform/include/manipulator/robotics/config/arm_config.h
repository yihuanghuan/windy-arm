#pragma once
#include <string>
#include <vector>
#include <map>
#include <cstdint>
#include <yaml-cpp/yaml.h>

namespace manipulator::config {

struct MotorModelConfig {
    std::string type;
    double max_torque{0.0};
    double max_velocity{0.0};
    double rated_torque{0.0};
    double p_gain_default{0.0};
    double d_gain_default{0.0};
    std::string coord_system;
};

struct JointConfig {
    std::string name;
    std::string model;
    uint8_t id{0};
    double lower_limit{0.0};
    double upper_limit{0.0};
};

struct ArmConfig {
    std::vector<JointConfig> joints;
};

class ConfigLoader {
public:
    static std::map<std::string, MotorModelConfig> LoadMotorModels(const YAML::Node& yaml);
    static ArmConfig LoadArmConfig(const YAML::Node& yaml, const std::string& arm_name);
};

}