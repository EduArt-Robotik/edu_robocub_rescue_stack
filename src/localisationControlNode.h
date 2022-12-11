#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <map>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <chrono> 
#include <vector>
#include <iostream> 

//#include "nav2_amcl/amcl_node.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include "localisation.h"
#include "navigation.h"

//using namespace std::chrono_literals; 
using namespace std;

class LocalisationControlNode : public rclcpp::Node 
{
    public:
    LocalisationControlNode();

    private:

    int m_wait;

    bool m_goal_send;

    bool m_initial_pose_set;

    void scan_callback(sensor_msgs::msg::LaserScan msg_scan);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_imu);

    void timer_callback();

    //void state_est_callback(std_msgs::msg::Float64MultiArray::SharedPtr state_vec_msg);

    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg);

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel);

    void amcl_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose);

    void setting_initial_pose();

    void setting_goal_pose();

    void nav_timer();

    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laserscan_;
    //###########NEU##############
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_amcl_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;
    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_slam_scan_;
    //###########NEU##############
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_initial_pose_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen 
    Control *m_control;
    //Control *ls_control;
    Localisation *m_localisation;
    Navigation *m_navigation;
};