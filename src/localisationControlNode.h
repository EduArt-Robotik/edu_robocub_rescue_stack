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

    void transition_amcl_callback(lifecycle_msgs::msg::TransitionEvent::SharedPtr transition);
    void transition_map_server_callback(lifecycle_msgs::msg::TransitionEvent::SharedPtr transition);


    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;
    //rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr subscriber_transition_amcl;
    //rclcpp::Subscription<lifecycle_msgs::msg::TransitionEvent>::SharedPtr subscriber_transition_map_server;

    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen 
    Control *m_control;
    Localisation *m_localisation;
};
