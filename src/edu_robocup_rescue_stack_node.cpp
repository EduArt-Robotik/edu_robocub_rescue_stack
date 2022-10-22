#include "rclcpp/rclcpp.hpp"

#include "localisationControl.h"


int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LocalisationControl>());
    rclcpp::shutdown();
    return 0;
}
