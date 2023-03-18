#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

#include "loadMap.h"

using namespace std;

//using namespace std;

class Navigation //: public rclcpp::Node 
{
    private:

    LoadMap *m_loadMap;
    //Localisation *m_localisation;

    double m_amcl_pX;
    double m_amcl_pY;
    
    double m_goal_pX;
    double m_goal_pY;
    double m_goal_pZ;
    double m_goal_oX;
    double m_goal_oY;
    double m_goal_oZ;
    double m_goal_oW;
    
    double m_tolerance;
    double m_lim_min_x;
    double m_lim_min_y;
    double m_lim_max_x;
    double m_lim_max_y;
    int m_area; 

    bool m_goal_achived;
    bool m_new_goal_set;
    bool m_goal2_set;
    bool m_goal3_set;
    bool m_goal4_set;
    bool m_send_goal;
    bool m_goal_sended;
    bool m_send_initial;
    bool m_initial_sended;
    bool m_map_sended;
    bool m_send_goalpose;
     
    
    bool m_start_area1;
    bool m_start_area2;
    bool m_start_area3;
    bool m_start_area4;
    bool m_loadmapStatus;
    bool m_map_request_sended;
    bool m_stop_goal;

    string m_url;
    string m_map1;
    string m_map2;
    string m_map3;
    string m_map4; 
    int m_wait;
    int m_tact;
    
    void terminal_output();
    void navigate(string m_url);
    void loadMap(string m_url);
    void sendInitialPose();
    void stop_goal();
    void sendGoalPose();
    void navigation_step();
    void calc_tolerance(double pX, double pY);
    void initialize();
    
    //rclcpp::TimerBase::SharedPtr nav_timer;
    //void nav_timer_callback();
    

    public:
    Navigation(LoadMap *m_loadMap);

    void setamclX(double x);
    void setamclY(double y);
    void setGoalsended(bool goalsended);
    void setInitialsended(bool initialsended);
    void setTact(int wait);
    void setMapArea(int area);

    
    double getGoalPosX();
    double getGoalPosY();
    double getGoalPosZ();
    
    double getGoalOriX();
    double getGoalOriY();
    double getGoalOriZ();
    double getGoalOriW();
    bool getNewGoalSet();
    bool getSendGoal();
    bool getSendInitial();


};

