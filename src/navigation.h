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
    
    int m_area;
    int m_area_saved; 

    bool m_goal_achived;
    bool m_new_goal_set;

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
    bool m_initialize;
    bool m_travel_forward;
    bool m_travel_backwards;
    bool m_turn;

    string m_url;
    string m_map1;
    string m_map2;
    string m_map3;
    string m_map4; 

    float m_pitch_rel;
    
    int m_count;
    bool m_c_start;
    bool m_c_fin;

    void navigate(string m_url);
    void loadMap(string m_url);
    bool sendInitialPose();
    bool sendGoalPose();
    void navigation_step();
    void initialize();
    void map_swap(int m_area);

    bool counter(bool c_start_);
    

    public:
    Navigation(LoadMap *m_loadMap);

    // set-functions
    void setamclX(double x);
    void setamclY(double y);
    void setGoalsended(bool goalsended);
    void setInitialsended(bool initialsended);
    void setMapArea(int area);
    void setPitchRel(float pitch_rel);

    // get-functions
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
    
    bool sendInitial(bool intial);


};

