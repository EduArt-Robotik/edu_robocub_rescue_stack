#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <map>

#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

#include <chrono> 
#include <vector>
#include <iostream> 

#define _USE_MATH_DEFINES
#include <cmath>

#include "localisation.h"
#include "navigation.h"

using namespace std;

class LocalisationControlNode : public rclcpp::Node 
{
    public:
    LocalisationControlNode();

    private:

    int m_wait;
    bool m_send_goal; 
    bool m_goal_sended;
    bool m_send_initial_pose;
    bool m_initial_pose_sended;

    bool m_initial_pose_set;

    float m_yawZ;
    float m_yawZ_strich;
    float m_yawZ_rays;
    
    int m_n_laserrays;
    int m_i_0;
    int m_i_90;
    int m_i_180;
    int m_i_270;

    int m_area;
    float m_pitch_rel; 

    void scan_callback(sensor_msgs::msg::LaserScan msg_scan);

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom);

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_imu);

    void timer_callback();
    void timer_clock_callback();

    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg);

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel);

    void amcl_pose_callback(geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose);

    void setting_initial_pose();

    void setting_goal_pose();

    void nav_timer();

    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscriber_laserscan_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr subscriber_amcl_pose_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;

    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_slam_scan_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_goal_pose_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_initial_pose_;
    rclcpp::Publisher<rosgraph_msgs::msg::Clock>::SharedPtr publisher_clock_time_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr timer_clock_;

    //Initialisierung Variablen 
    Control *m_control;
    //Control *ls_control;
    Localisation *m_localisation;
    Navigation *m_navigation;
    //LoadMap *m_loadMap;

    //state topic neccessary for getstate and change state 
    static constexpr char const * map_server_load_map_topic = "/map_server/load_map";

    //Initialising Client
    std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> map_server_load_map;
};