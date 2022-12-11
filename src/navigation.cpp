#include "navigation.h"

Navigation::Navigation() { //: Node("navigation")
    m_amcl_pX = 0;
    m_amcl_pY = 0;
    
    m_tolerance = 0.3;
    m_lim_min_x = 0.0;
    m_lim_min_y = 0.0;
    m_lim_max_x = 0.0;
    m_lim_max_y = 0.0;

    m_goal_pX = 0.0;
    m_goal_pY = 0.0;
    m_goal_pZ = 0.0;
    m_goal_oX = 0.0;
    m_goal_oY = 0.0; 
    m_goal_oZ = 0.0;
    m_goal_oW = 0.0;

    m_goal_achived = false;
    m_new_goal_set = false;
    m_goal2_set = false;
    m_goal3_set = false;
    m_goal4_set = false;
    m_goal_send = false;
    m_wait = 0;
    m_tact = 0;

    //nav_timer = this->create_wall_timer(1000ms, std::bind(&Navigation::nav_timer_callback, this));
}

void Navigation::terminal_output(){
    std::cout << "amcl_x:" << m_amcl_pX << std::endl;
    std::cout << "amcl_y1:" << m_amcl_pY << std::endl;
    //navigate();
}

void Navigation::calc_tolerance(double pX, double pY){
    m_lim_min_x = pX - m_tolerance;
    m_lim_min_y = pY - m_tolerance;
    m_lim_max_x = pX + m_tolerance;
    m_lim_max_y = pY + m_tolerance;
    
}
void Navigation::navigate(){

    //set goal nr.1 - End of plain
    if(!m_goal_achived) {
        m_goal_pX = 3.5;
        m_goal_pY = 0.0;
        m_goal_pZ = 0.0;
        m_goal_oX = 0.0;
        m_goal_oY = 0.0; 
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        //m_goal_send = false;

        calc_tolerance(m_goal_pX, m_goal_pY);
        std::cout << "tol1 calculated..." << std::endl;
    
        if(m_lim_min_x < m_amcl_pX && m_amcl_pX < m_lim_max_x && m_lim_min_y < m_amcl_pY && m_amcl_pY < m_lim_max_y){
            m_wait = m_wait + 50;
            std::cout << "start waiting1" << std::endl;
            if(m_wait >= 500){      //wait 500ms
                m_goal_achived = true;
                m_goal2_set = true; //true for new goal!
                m_goal_send = true;
                m_wait = 0;
                std::cout << "goal1 achived" << std::endl;  
            }

        }

    std::cout << "m_goal2_set:" << m_goal2_set << std::endl;
    if(m_goal2_set){
        m_goal_pX = 3.0;
        m_goal_pY = 1.2;
        m_goal_pZ = 0.0;
        m_goal_oX = 0.0;
        m_goal_oY = 0.0; 
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        //m_goal_send = false;

        calc_tolerance(m_goal_pX, m_goal_pY);
        std::cout << "tol2 calculated..." << std::endl;

        if(m_lim_min_x < m_amcl_pX && m_amcl_pX < m_lim_max_x && m_lim_min_y < m_amcl_pY && m_amcl_pY < m_lim_max_y){
            m_wait = m_wait + 50;
            std::cout << "start waiting2" << std::endl;
            if(m_wait >= 500){      //wait 500ms
                std::cout << "goal2 achived" << std::endl;
                m_goal2_set = false;
                m_goal3_set = true;
                m_wait = 0;
            }
        }

    }

    if(m_goal3_set){
        m_goal_pX = 5.0;
        m_goal_pY = 1.2;
        m_goal_pZ = 0.0;
        m_goal_oX = 0.0;
        m_goal_oY = 0.0; 
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        //m_goal_send = false;

        calc_tolerance(m_goal_pX, m_goal_pY);
        if(m_lim_min_x < m_amcl_pX && m_amcl_pX < m_lim_max_x && m_lim_min_y < m_amcl_pY && m_amcl_pY < m_lim_max_y){
            m_wait = m_wait + 50;
            std::cout << "start waiting3" << std::endl;
            if(m_wait >= 50){      //wait 500ms
                std::cout << "goal3 achived" << std::endl;
                //m_goal3_set = true;
            }
           
        }

    }



    }
}

void Navigation::setamclX(double x){
    m_amcl_pX = x;
    std::cout << "amcl_x:" << m_amcl_pX << std::endl;

}

void Navigation::setamclY(double y){
    m_amcl_pY = y;
    std::cout << "amcl_y:" << m_amcl_pY << std::endl;
}

void Navigation::setTact(int wait){
    m_tact = wait;
    std::cout << "called navigate" << std::endl;
    navigate();
    std::cout << "fin of navigation" << std::endl;
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

void Navigation::setGoalsended(bool goalsended){
    m_goal_send = goalsended;
}

bool Navigation::getPoseSend(){
    //navigate();
    return m_goal_send;
}


