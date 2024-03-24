#include "adisha_data/config.hpp"



adisha::Config::Config() {
    
    this->core_config   = YAML::LoadFile("/altair-os/src/adisha_data/config/core_config.yaml");
    this->robot_config  = YAML::LoadFile("/altair-os/src/adisha_data/config/robot_config.yaml");
    this->joint_config  = YAML::LoadFile("/altair-os/src/adisha_data/config/joint_config.yaml");

    this->id    = this->robot_config["id"].as<std::string>();
    this->ip    = this->robot_config["ip"].as<std::string>();
}