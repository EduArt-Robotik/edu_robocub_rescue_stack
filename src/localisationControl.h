#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <cmath>
#include <vector>
#include <iostream> 

#define _USE_MATH_DEFINES


using namespace std::chrono_literals; 
using namespace std;
using std::placeholders::_1;


class LocalisationControl : public rclcpp::Node 
{
    public:
    LocalisationControl(): Node("localisation_control")
{
    
    //subscriber
    subscriber_odom_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControl::odom_callback, this, _1));
    subscriber_velocity_=this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&LocalisationControl::velocity_callback, this, _1));
    subscriber_state_est_=this->create_subscription<std_msgs::msg::Float64MultiArray>("/state_est", 1, std::bind(&LocalisationControl::state_est_callback, this, _1));
    
    //publisher
    publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //timer
    timer_ = this->create_wall_timer(50ms, std::bind(&LocalisationControl::timer_callback, this));
}

private:

    
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom);

    void timer_callback();

    void state_est_callback(std_msgs::msg::Float64MultiArray::SharedPtr state_vec_msg);

    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg);

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel);


    //Initialisierung Subscriber
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;

    //Initialisierung Publisher
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;

    //Initialisierung Timer
    rclcpp::TimerBase::SharedPtr timer_;

    //Initialisierung Variablen
    _Float32 t0;
    _Float32 t1;
    _Float32 t2;
    _Float32 t3;
    _Float32 t4;
    _Float32 roll_x;
    _Float32 pitch_y;
    _Float32 yaw_z;
    _Float32 v;
    _Float32 yaw_rate;
    std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
    _Float32 x;
    _Float32 y;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
    _Float32 x_dest;
    _Float32 y_dest;

    _Float32 delta_x;
    _Float32 delta_y;
    _Float32 delta_dist;
    _Float32 delta_phi;

};