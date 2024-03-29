#include <memory>
#include <chrono>
//#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "nav2_msgs/srv/load_map.hpp"
using namespace std;
using namespace std::chrono_literals;


class LoadMap
{
    public:
    LoadMap(std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> client_load_map);
    
    
    void startLoadMap(std::chrono::seconds timeout = 1s, string url = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack/map/map_6.1.yaml");
    bool getLoadStatus();
    
    private:

    void loadMap_callback(const rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future);
    //std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> load_map_client = this->create_client<nav2_msgs::srv::LoadMap>("/map1_server/load_map");
    std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> m_client_load_map;

    bool m_map_sended; 
};