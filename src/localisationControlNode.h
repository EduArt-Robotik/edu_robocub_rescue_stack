#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 
#include <memory>
#define _USE_MATH_DEFINES
#include <cmath>
#include "lifecycle_msgs/msg/transition_event.hpp"

#include "loadMap.h"
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
    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_imu);

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
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr subscriber_imu_;


    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_initialpose_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen 
    Control *m_control;
    Localisation *m_localisation_amcl;
    Localisation *m_localisation_imu;
    Localisation *m_localisation_odom;

    ClientService *m_amclService;
    ClientService *m_map1ServerService;
    ClientService *m_map2ServerService;
    ClientService *m_map3ServerService;
    ClientService *m_map4ServerService;

    bool m_amcl_startet = false;
    int m_initpose_wait = 0;

    //state topic neccessary for getstate and change state 
    static constexpr char const * amcl_get_state_topic = "/amcl/get_state";
    static constexpr char const * amcl_change_state_topic = "/amcl/change_state";

    static constexpr char const * map1_server_get_state_topic = "/map1_server/get_state";
    static constexpr char const * map1_server_change_state_topic = "/map1_server/change_state";

    static constexpr char const * map2_server_get_state_topic = "/map2_server/get_state";
    static constexpr char const * map2_server_change_state_topic = "/map2_server/change_state";

    static constexpr char const * map3_server_get_state_topic = "/map3_server/get_state";
    static constexpr char const * map3_server_change_state_topic = "/map3_server/change_state";

    static constexpr char const * map4_server_get_state_topic = "/map4_server/get_state";
    static constexpr char const * map4_server_change_state_topic = "/map4_server/change_state";

    static constexpr char const * map_server_load_map_topic = "/map_server/load_map";


     //Initalising Client
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> amcl_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> amcl_change_state;

    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> map1_server_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> map1_server_change_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> map2_server_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> map2_server_change_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> map3_server_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> map3_server_change_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::GetState>> map4_server_get_state;
    std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> map4_server_change_state;
    std::shared_ptr<rclcpp::Client<nav2_msgs::srv::LoadMap>> map_server_load_map;

};
