#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream> 
#include "clientService.h"
#include "loadMap.h"

class Control
{
    private:

    ClientService *m_map1ServerService;
    ClientService *m_map2ServerService;
    ClientService *m_map3ServerService;
    ClientService *m_map4ServerService;

    LoadMap *m_loadMap;

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
    int m_slowDownSpeed = 10;
    int m_slowDownSpeedReduce = 10;
    int m_slowDownTurning;
    int m_navigationStep;
    float m_dest_precision;
    int m_driving_direction;    //forward -> 1; backwards -> -1

    void setNaviagtionStep();

    //Utils
    float angleOverTwoPi(float angle);

    public:
    Control(ClientService *map1ServerService, ClientService *map2ServerService, ClientService *map3ServerService, ClientService *map4ServerService, LoadMap *loadMap);
    void calculateAngleSpeed() ;

    void setPosYaw(float x,float y,float yaw_z);
    void setPitchY(float pitch_y);
    void setYawZ(float yaw_z);
    void setRollX(float roll_x);
    void setX(float x);
    void setY (float y);
    float getAngle();
    float getSpeed();
    int getNavigationStep();
    void previousNavigationStep();
    void nextNavigationStep();

    ClientService *m_activeMapServer;
};