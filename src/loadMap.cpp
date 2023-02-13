#include "loadMap.h"
#include <thread>

using namespace std::chrono_literals;

LoadMap::LoadMap(std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> client_load_map)
{
  m_client_load_map = client_load_map;
}


void LoadMap::startLoadMap(std::chrono::seconds timeout ){
   auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
  request->map_url = "/ros/maps/map.yaml";
/*
    if (!m_client_change_state->wait_for_service(timeout))
    {
      std::cout<<"Service "<<m_client_change_state->get_service_name() << "not available" << std::endl;
      return;
    }
    m_state_change = true;
   m_client_change_state->async_send_request(request, std::bind(&ClientService::changeState_callback, this, std::placeholders::_1));*/
  }
