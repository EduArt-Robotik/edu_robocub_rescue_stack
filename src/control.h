#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 
#include "clientService.h"

class Control
{
    private:
    //direction Variables
    float m_yaw_z;
    float m_pitch_y;
    float m_roll_x;
    float m_x;
    float m_y;
    float m_angle;
    float m_speed;
    float m_x_dest;
    float m_y_dest;

    //controlling variables
    int m_slowDown = 10;
    int m_slowDownReduce = 10;
    int m_navigationStep;

    float m_dest_precision;
    int m_driving_direction;

    void calculateAngleSpeed() ;
    void nextNaviagtionStep();

    ClientService *m_map1ServerService;
    ClientService *m_map2ServerService;
    ClientService *m_map3ServerService;
    ClientService *m_map4ServerService;

    //Utils
    float angleOverTwoPi(float angle);

    public:
    Control(ClientService *map1ServerService, ClientService *map2ServerService, ClientService *map3ServerService, ClientService *map4ServerService);
    void setPosYaw(float x,float y,float yaw_z);
    void setPitchY(float pitch_y);
    void setYawZ(float yaw_z);
    void setRollX(float roll_x);
    void setX(float x);
    void setY (float y);
    float getAngle();
    float getSpeed();
    int getNavigationStep();


    ClientService *m_activeMapServer;
};