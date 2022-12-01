#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
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

    //callback
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom);
    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel);
    void amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose);
    //void state_est_callback(std_msgs::msg::Float64MultiArray::SharedPtr state_vec_msg);

    void timer_callback();

    //publisher
    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg);
    void publish_initalpose(geometry_msgs::msg::Pose pose_msg);

    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_amcl_pose_;


    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_initialpose_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen 
    Control *m_control;
    Localisation *m_localisation;
    
    Control *m_control_odom;

    Localisation *m_localisation_odom;

    bool m_amcl_startet = false;
    int m_initpose_wait = 0;
};
