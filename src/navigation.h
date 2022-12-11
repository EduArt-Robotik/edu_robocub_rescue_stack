#include <geometry_msgs/msg/pose_stamped.hpp>
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <vector>
#include <iostream>
#include "std_msgs/msg/float64_multi_array.hpp"
#include <cmath>

using namespace std;

//using namespace std;

class Navigation //: public rclcpp::Node 
{
    private:

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

    bool m_goal_achived;
    bool m_new_goal_set;
    bool m_goal2_set;
    bool m_goal3_set;
    bool m_goal4_set;
    bool m_goal_send;
    int m_wait;
    int m_tact;
    
    void terminal_output();
    void navigate();
    void calc_tolerance(double pX, double pY);
    
    //rclcpp::TimerBase::SharedPtr nav_timer;
    //void nav_timer_callback();
    

    public:
    Navigation();

    void setamclX(double x);
    void setamclY(double y);
    void setGoalsended(bool goalsended);
    void setTact(int wait);
    
    double getGoalPosX();
    double getGoalPosY();
    double getGoalPosZ();
    
    double getGoalOriX();
    double getGoalOriY();
    double getGoalOriZ();
    double getGoalOriW();
    bool getNewGoalSet();
    bool getPoseSend();


};

