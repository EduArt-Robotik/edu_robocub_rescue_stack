#include "amclService.h"

AmclService::AmclService(std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state,
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state)
{
    m_client_get_state = client_get_state;
    m_client_change_state = client_change_state;
}

unsigned int AmclService::get_state(std::chrono::seconds timeout )
{
    auto request = std::make_shared<lifecycle_msgs::srv::GetState::Request>();

    if (!m_client_get_state->wait_for_service(timeout))
    {
      std::cout<<"Service "<<m_client_get_state->get_service_name()<<" not available"<<std::endl;
      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    auto future_result = m_client_get_state->async_send_request(request);

    auto future_status = wait_for_result(future_result,timeout);

    if (future_status != std::future_status::ready)
    {
      std::cout<<"Server timed out while getting current state for node "<<amcl_node<<std::endl;

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }

    if (future_result.get())
    {
      auto state = future_result.get()->current_state.id;
      std::cout<<"Node "<<amcl_node<<" has current state "<<future_result.get()->current_state.label.c_str();

      return state;
    }
    else{
      std::cout<<"Failed to get current state for node "<<amcl_node<<std::endl;

      return lifecycle_msgs::msg::State::PRIMARY_STATE_UNKNOWN;
    }
}

bool AmclService::change_state(std::uint8_t transition, std::chrono::seconds timeout )
{
    auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
    request->transition.id = transition;

    if (!m_client_change_state->wait_for_service(timeout))
    {
      std::cout<<"Service "<<m_client_change_state->get_service_name()<<"not available"<<std::endl;
        return false;
    }

    auto future_result = m_client_change_state->async_send_request(request);

    auto future_status = wait_for_result(future_result,timeout);

    if (future_status!=std::future_status::ready)
    {
      std::cout<<"Server timed out while getting current state for node "<<amcl_node<<std::endl;

      return false;
    }

    if (future_result.get()->success)
    {
      std::cout<<"Transition "<<static_cast<unsigned int>(transition) <<" successfully triggered";
      return true;
    }
    else{
      
      std::cout<<"Failed to get trigger transition"<< static_cast<unsigned int>(transition)<<"for node "<<amcl_node<<std::endl;

      return false;
    }
}


template <typename FutureT, typename WaitTimeT>std::future_status AmclService::wait_for_result
    ( FutureT & future,  WaitTimeT time_to_wait)
{
    auto end = std::chrono::steady_clock::now() + time_to_wait;
    std::chrono::milliseconds wait_period(100);
    std::future_status status = std::future_status::timeout;
    do{
        auto now = std::chrono::steady_clock::now();
        auto time_left = end - now;

        if (time_left<=std::chrono::seconds(0))
        {
        break;
        }
        status = future.wait_for((time_left< wait_period)? time_left: wait_period);

    }
    while(rclcpp::ok() && status!=std::future_status::ready);
    return status;
}

