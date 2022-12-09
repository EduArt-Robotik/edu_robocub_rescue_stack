#include "rclcpp/rclcpp.hpp"

#include "localisationControlNode.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalisationControlNode>());
    rclcpp::shutdown();
    return 0;
}
