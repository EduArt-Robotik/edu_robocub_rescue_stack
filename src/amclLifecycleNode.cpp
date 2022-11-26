#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include "rclcpp_lifecycle/lifecycle_node.hpp"

//using namespace std::chrono_literals; 

class AmclLifecycleNode : public rclcpp_lifecycle::LifecycleNode{
    public:
    AmclLifecycleNode(const std::string & nodeName, bool intraProcessComms = false): 
    rclcpp_lifecycle::LifecycleNode(nodeName, rclcpp::NodeOptions().use_intra_process_comms(intraProcessComms)){

    }
    private:
    
    std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> publisher;
    std::shared_ptr<rclcpp::TimerBase> timer;

};