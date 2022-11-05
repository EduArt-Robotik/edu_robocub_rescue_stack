#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 

class Control
{
    private:
    
    float m_yaw_z;
    float m_x;
    float m_y;
    float m_angle;
    float m_speed;
    float m_x_dest;
    float m_y_dest;
    int m_times = 10;
    void calculateAngleSpeed() ;

    public:
    Control();
    void setPosYaw(float x,float y,float yaw_z);
    void setYawZ(float yaw_z);
    void setX(float x);
    void setY (float y);
    float getAngle();
    float getSpeed();
};