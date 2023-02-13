#include <memory>
#include <chrono>
//#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "nav2_msgs/srv/load_map.hpp"



using namespace std::chrono_literals;


class LoadMap{
    public:
    LoadMap (std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> client_load_map);




    private:
    void startLoadMap(std::chrono::seconds timeout );
    void loadMap_callback(const rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future);
    //std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> load_map_client = this->create_client<nav2_msgs::srv::LoadMap>("/map1_server/load_map");
    std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> m_client_load_map;

};