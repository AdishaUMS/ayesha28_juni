#include "adisha_interfaces/app_launcher_node.hpp"



void adisha::app_launch() {
    adisha::Config config = adisha::Config();

    RCLCPP_INFO(rclcpp::get_logger("app_launcher"), "Starting Adisha App on %s:3000", config.ip.c_str());
    system("node /adisha-os/web/app.js");
}