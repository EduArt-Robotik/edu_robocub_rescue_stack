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

  int getState();
  bool getStateChange();
  bool getStateGet();


  private:

  void getState_callback(const rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);
  void changeState_callback(const rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);

  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> m_client_get_state;
  std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> m_client_change_state;
  void start_get_state(std::chrono::seconds timeout = 3s);
  void start_change_state(std::uint8_t transition, std::chrono::seconds timeout = 3s);

  template <typename FutureT, typename WaitTimeT> std::future_status wait_for_result
  (FutureT & future,WaitTimeT time_to_wait);

  std::string m_nodeName;
  int m_state;
  bool m_state_change;
    bool m_state_get;

};

