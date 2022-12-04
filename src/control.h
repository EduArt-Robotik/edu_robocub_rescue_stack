#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 

class Control
{
    private:
    
    float m_yaw_z;
    float m_pitch_y;
    float m_x;
    float m_y;
    float m_angle;
    float m_speed;
    float m_x_dest;
    float m_y_dest;
    int m_times = 10;
    int m_navigation;
    bool m_newInitialpose;
    void calculateAngleSpeed() ;
    void control_navigation();
    geometry_msgs::msg::Pose m_initialpose;
    public:
    Control();
    void setPosYaw(float x,float y,float yaw_z);
    void setPitch(float pitch_y);
    void setYawZ(float yaw_z);
    void setX(float x);
    void setY (float y);
    float getAngle();
    float getSpeed();
    int getNavigationStep();
    bool newInitialpose();
    geometry_msgs::msg::Pose getInitialpose();
    float m_speed_var;
    int m_wait = 0;
    
};