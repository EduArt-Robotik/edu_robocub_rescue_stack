#include "loadMap.h"
#include <thread>
#include <string>

using namespace std;
using namespace std::chrono_literals;


LoadMap::LoadMap(std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> client_load_map)
{
  m_client_load_map = client_load_map;
}


void LoadMap::startLoadMap(std::chrono::seconds timeout, string url ){

    auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
    request->map_url = url;
    m_map_sended = false;


    if (!m_client_load_map->wait_for_service(timeout))
    {
      std::cout<<"Service "<<m_client_load_map->get_service_name() << "not available" << std::endl;
      return;
    }

   auto result = m_client_load_map->async_send_request(request, std::bind(&LoadMap::loadMap_callback, this, std::placeholders::_1));
}

void LoadMap::loadMap_callback(const rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedFuture future)
{
  if(future.get()->result== nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS){
    std::cout << "load map successful" <<std::endl;
    m_map_sended = true; 
    
  }else{
    std::cout <<  "load map not successful" <<std::endl;
    m_map_sended = false;
  }
}

bool LoadMap::getLoadStatus() {
  return m_map_sended; 
}