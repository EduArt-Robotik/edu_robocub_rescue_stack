#include <memory>
#include <chrono>
//#include <thread>
#include "rclcpp/rclcpp.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"


using namespace std::chrono_literals;


class ClientService{
    public:
    ClientService(std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> client_get_state,
        std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> client_change_state, std::string nodeName);


    bool activateService();
    bool deactiveService();
 
    private:

    void activateServiceTask();
    void deactiveServiceTask();
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_client_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_client_change_state;
    unsigned int get_state(std::chrono::seconds timeout = 3s);
    bool change_state(std::uint8_t transition, std::chrono::seconds timeout = 3s);

    template <typename FutureT, typename WaitTimeT> std::future_status wait_for_result
    (FutureT & future,WaitTimeT time_to_wait);

    std::string m_nodeName;
};


/*
void
callee_script(std::shared_ptr<ServiceClient> service_client)
{
  rclcpp::WallRate time_between_state_changes(0.1); //10s

  //configure
  {
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //cleanup
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //unconfigured shutdown
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }*/
