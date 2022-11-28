#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 
#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "localisation.h"

using namespace std;

class LocalisationControlNode : public rclcpp::Node 
{
    public:
    LocalisationControlNode();

private:

    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom);

    void timer_callback();

    //void state_est_callback(std_msgs::msg::Float64MultiArray::SharedPtr state_vec_msg);

    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg);

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel);


    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;

    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;

    //Initalising Client
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> amcl_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> amcl_change_state;

    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> map_server_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> map_server_change_state;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen 
    Control *m_control;
    Localisation *m_localisation;
    ClientService *m_amclService;
    ClientService *m_mapServerService;

    static constexpr char const * amcl_get_state_topic = "/amcl/get_state";
    static constexpr char const * amcl_change_state_topic = "/amcl/change_state";

    static constexpr char const * map_server_get_state_topic = "/map_server/get_state";
    static constexpr char const * map_server_change_state_topic = "/map_server/change_state";
};
