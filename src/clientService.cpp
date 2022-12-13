#include "clientService.h"

ClientService::ClientService(std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state,
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state, std::string nodeName)
{
    m_client_get_state = client_get_state;
    m_client_change_state = client_change_state;
    m_nodeName = nodeName;
    get_state();
    m_state = -1;
}


void ClientService::getState_callback(const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future) 
{
  m_state = (int) future.get()->current_state.id;
  std::cout <<  m_nodeName << " state: " <<m_state<<std::endl;

}

void ClientService::changeState_callback(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future) 
{
  get_state();
}

bool ClientService::activateService(){

  if(m_state ==  lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
    std::cout <<  m_nodeName << " is already active " <<std::endl;
  }
  else if (m_state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
    change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_ACTIVATING);       
  }
  else if (m_state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED){
    change_state(lifecycle_msgs::msg::State::TRANSITION_STATE_CONFIGURING);
  }
  else{
    std::cout <<  m_nodeName << " state, not defined : " <<m_state<<std::endl;
  }

}

void ClientService::get_state(std::chrono::seconds timeout )
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!m_client_get_state->wait_for_service(timeout))
    {
      std::cout << "Service "<< m_client_get_state->get_service_name() << " not available"<<std::endl;
      return;
    }

    m_client_get_state->async_send_request(request, std::bind(&ClientService::getState_callback, this, std::placeholders::_1));
}

void ClientService::change_state(std::uint8_t transition, std::chrono::seconds timeout )
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!m_client_change_state->wait_for_service(timeout))
    {
      std::cout<<"Service "<<m_client_change_state->get_service_name() << "not available" << std::endl;
      return;
    }

   m_client_change_state->async_send_request(request, std::bind(&ClientService::changeState_callback, this, std::placeholders::_1));
}

