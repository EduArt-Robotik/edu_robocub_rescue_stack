#include "navigation.h"

Navigation::Navigation() { //: Node("navigation")
    m_amcl_pX = 0;
    m_amcl_pY = 0;
    
    m_tolerance = 0.2;
    m_lim_min_x = 0.0;
    m_lim_min_y = 0.0;
    m_lim_max_x = 0.0;
    m_lim_max_y = 0.0;

    m_goal_pX = 6.0;
    m_goal_pY = 1.2;
    m_goal_pZ = 0.0;
    m_goal_oX = 0.0;
    m_goal_oY = 0.0; 
    m_goal_oZ = 0.0;
    m_goal_oW = 0.0;

    m_goal_achived = false;
    m_new_goal_set = false;
    m_goal_send = false;
    m_wait = 0;

    //nav_timer = this->create_wall_timer(1000ms, std::bind(&Navigation::nav_timer_callback, this));
}

void Navigation::terminal_output(){
    std::cout << "amcl_x:" << m_amcl_pX << std::endl;
    std::cout << "amcl_y:" << m_amcl_pY << std::endl;
}

void Navigation::calc_tolerance(){
    m_lim_min_x = m_goal_pX - m_tolerance;
    m_lim_min_y = m_goal_pY - m_tolerance;
    m_lim_max_x = m_goal_pX + m_tolerance;
    m_lim_max_y = m_goal_pY + m_tolerance;
}
void Navigation::navigate(){
    //set goal nr.1 - End of plain
    if(!m_goal_achived) {
        m_goal_pX = 6.0;
        m_goal_pY = 1.2;
        m_goal_pZ = 0.0;
        m_goal_oX = 0.0;
        m_goal_oY = 0.0; 
        m_goal_oZ = 1.0;
        m_goal_oW = 0.0;
        //m_goal_send = false;

        calc_tolerance();
    
        if(m_lim_min_x < m_amcl_pX && m_amcl_pX < m_lim_max_x && m_lim_min_y < m_amcl_pY && m_amcl_pY < m_lim_max_y){
            m_wait = m_wait + 50;
            if(m_wait >= 500){      //wait 500ms
                m_goal_achived = true;
                m_new_goal_set = true;
                m_goal_send = true;
                std::cout << "goal_achived, time over:" << std::endl;  
            }

        }
    
    if(m_new_goal_set){
        m_goal_pX = 0.0;
        m_goal_pY = 0.0;
        m_goal_pZ = 0.0;
        m_goal_oX = 0.0;
        m_goal_oY = 0.0; 
        m_goal_oZ = 0.0;
        m_goal_oW = 0.0;
        //m_goal_send = false;

        calc_tolerance();
        if(m_lim_min_x < m_amcl_pX && m_amcl_pX < m_lim_max_x && m_lim_min_y < m_amcl_pY && m_amcl_pY < m_lim_max_y){
            std::cout << "round finished" << std::endl;
            m_new_goal_set = false;
        }

    }



    }
}

void Navigation::setamclX(double x){
    m_amcl_pX = x;
    navigate();
    std::cout << "amcl_x:" << m_amcl_pX << std::endl;

}

void Navigation::setamclY(double y){
    m_amcl_pY = y;
    navigate();
    std::cout << "amcl_y:" << m_amcl_pY << std::endl;
}

double Navigation::getGoalPosX(){
    return m_goal_pX;
}

double Navigation::getGoalPosY(){
    return m_goal_pY;
}

double Navigation::getGoalPosZ(){
    return m_goal_pZ;
}

double Navigation::getGoalOriX(){
    return m_goal_oX;
}

double Navigation::getGoalOriY(){
    return m_goal_oY;
}

double Navigation::getGoalOriZ(){
    return m_goal_oZ;
}

double Navigation::getGoalOriW(){
    return m_goal_oW;
}

bool Navigation::getNewGoalSet(){
    return m_new_goal_set;
}

bool Navigation::getPoseSend(){
    return m_goal_send;
}
