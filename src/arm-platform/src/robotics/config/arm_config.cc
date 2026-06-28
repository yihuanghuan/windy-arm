#include <manipulator/robotics/config/arm_config.h>

namespace manipulator::config {

std::map<std::string, MotorModelConfig> ConfigLoader::LoadMotorModels(const YAML::Node& yaml) {
    std::map<std::string, MotorModelConfig> models;
    
    if (!yaml["motor_models"]) return models;
    
    for (const auto& it : yaml["motor_models"]) {
        std::string model_name = it.first.as<std::string>();
        MotorModelConfig config;
        const YAML::Node& node = it.second;
        
        if (node["type"]) config.type = node["type"].as<std::string>();
        if (node["max_torque"]) config.max_torque = node["max_torque"].as<double>();
        if (node["rated_torque"]) config.rated_torque = node["rated_torque"].as<double>();
        if (node["max_velocity"]) config.max_velocity = node["max_velocity"].as<double>();
        if (node["p_gain_default"]) config.p_gain_default = node["p_gain_default"].as<double>();
        if (node["d_gain_default"]) config.d_gain_default = node["d_gain_default"].as<double>();
        if (node["coord_system"]) config.coord_system = node["coord_system"].as<std::string>();
        
        models[model_name] = config;
    }
    
    return models;
}

ArmConfig ConfigLoader::LoadArmConfig(const YAML::Node& yaml, const std::string& arm_name) {
    ArmConfig config;
    
    if (!yaml["arms"] || !yaml["arms"][arm_name]) return config;
    
    const YAML::Node& arm_node = yaml["arms"][arm_name];
    
    if (arm_node["joints"]) {
        for (const auto& joint : arm_node["joints"]) {
            JointConfig jc;
            if (joint["name"]) jc.name = joint["name"].as<std::string>();
            if (joint["model"]) jc.model = joint["model"].as<std::string>();
            if (joint["id"]) jc.id = joint["id"].as<uint8_t>();
            if (joint["lower_limit"]) jc.lower_limit = joint["lower_limit"].as<double>();
            if (joint["upper_limit"]) jc.upper_limit = joint["upper_limit"].as<double>();

            config.joints.push_back(jc);
        }
    }
    
    return config;
}

}