#include "navigation.h"

Navigation::Navigation(LoadMap *loadMap) { //: Node("navigation")
    //m_localisation = localisation;
    m_loadMap = loadMap;
    m_amcl_pX = 0;
    m_amcl_pY = 0;
    m_area = 1;
    m_area_saved = 1;
    
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
    m_send_goal = false;
    m_goal_sended = false;
    m_wait = 0;
    m_tact = 0;
    m_start_area1 = true;
    m_map_sended = false;
    m_send_initial = false;
    m_initial_sended = false;
    m_turn = false;

    m_send_goalpose = false;

    m_travel_forward = true;
    m_travel_backwards = false;

    c_start = false;
    c_fin = false;

    m_map1 = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack/map/map_9.1.yaml";
    m_map2 = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack/map/map_9.2.yaml";
    m_map3 = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack/map/map_8.3.yaml";
    m_map4 = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack/map/map_8.4.yaml";
}

void Navigation::initialize() {

    m_send_goal = false;
    m_goal_sended = false;
    
    m_map_sended = false;

    m_loadmapStatus = false;
    m_map_request_sended = false;
    
    m_send_initial = false;
    m_initial_sended = false;

    c_start = false;

}

bool Navigation::counter(bool c_start_) {
    
    if (c_start_) {
        m_count = 0;
        c_start_ = false;
    }

    m_count = m_count + 1;
    
    if (m_count >= 2) {

        return true;

    } else {
        
        return false;

    }

}

bool Navigation::sendGoalPose(){

    if (!m_goal_sended) {
        m_send_goal = true;
    } else if (m_goal_sended) {
        m_send_goal = false;
        return true; 
    }   
}

bool Navigation::sendInitialPose(){

    if (!m_initial_sended) {
        m_send_initial = true;
    } else if (m_initial_sended) {
        m_send_initial = false;
        m_map_request_sended = true;    //changed from false, because of changed order of navigation 
    }   
    return m_map_request_sended;
}

void Navigation::map_swap(int m_area) {

    if (m_area != m_area_saved){
        c_start = true;
        m_area_saved = m_area;
        initialize();
    }

}

void Navigation::navigation_step(){
    
    std::cout << "m_amcl_X: " << m_amcl_pX << std::endl;

    map_swap(m_area);

    if (m_area == 1) {
        if ( (m_amcl_pX <= 0.1) && m_turn ) {
            m_travel_backwards = false;
            m_travel_forward = true;
            m_turn = false;
            initialize();
        }
        if (m_travel_forward) {           
            
            // position
            m_goal_pX = 2.35; //2.35
            m_goal_pY = - 0.05;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {
            
            // position
            m_goal_pX = 0.0;
            m_goal_pY = 0.0;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;
        }

        m_url = m_map1;

        navigate(m_url);

    } else if ((m_area == 2)&&(m_amcl_pX < 2.7)) {
        
        m_turn = true;

        if (m_travel_forward) {

            // position
            m_goal_pX = 2.85; // changed from 3.7
            m_goal_pY = -0.05;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // position
            m_goal_pX = 1.8; // changed from 3.7
            m_goal_pY = 0.2;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        }

        m_url = m_map2;
        navigate(m_url);

    } else if ((m_area == 2)&&(m_amcl_pX >= 2.7) ) {
        
        m_turn = true;

        if (m_travel_forward) {

            // position
            m_goal_pX = 4.0; // changed from 3.7
            m_goal_pY = 1.2;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // position
            m_goal_pX = 1.8; // changed from 3.7
            m_goal_pY = 0.2;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        }

        m_url = m_map2;
        navigate(m_url);
        

    } else if ((m_area == 3) && (m_amcl_pX > 4.1)) {

        if (m_travel_forward) {
        
            // position
            m_goal_pX = 5.3; 
            m_goal_pY = 1.1;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // position
            m_goal_pX = 4.0; 
            m_goal_pY = 1.25;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        }

        m_url = m_map3;
        navigate(m_url);

        } else if ((m_area == 3) && (m_amcl_pX <= 4.1)) {

        if (m_travel_forward) {
        
            // position
            m_goal_pX = 5.3; 
            m_goal_pY = 1.1;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // position
            m_goal_pX = 3.0; 
            m_goal_pY = 0.0;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        }

        m_url = m_map3;
        navigate(m_url);

    } else if (m_area == 4) {

        if ( (m_amcl_pX >= 5.8) && m_turn ) {
            m_travel_backwards = true;
            m_travel_forward = false;
            m_turn = false;
            initialize();
        }

        if (m_travel_forward) {

            // position
            m_goal_pX = 6.0; 
            m_goal_pY = 1.2;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // position
            m_goal_pX = 4.7; 
            m_goal_pY = 1.3;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;           
        }
        
        m_url = m_map4;
        navigate(m_url);


    }
}

void Navigation::loadMap(string m_url){
    
    if (!m_map_request_sended) {
        
        m_loadMap -> startLoadMap(1s, m_url);
        m_map_request_sended = true;
        
        m_send_goalpose = true;  
    }

    m_loadmapStatus = m_loadMap -> getLoadStatus();
}

void Navigation::navigate(string m_url){

    
    if (!m_map_request_sended) {   
        
        loadMap(m_url);

    } else if (m_send_goalpose) {
        
        m_send_initial = sendGoalPose();
        m_send_goalpose = false;
        c_start = true; 
        //std::cout << "######## IN SEND_GOAL FUNCTION #############" << std::endl;
    } else if (m_send_initial){

        c_fin = counter(c_start);
        c_start = false;
        //std::cout << "######## IN SEND_INITIAL FUNCTION #############" << std::endl;
        if (c_fin) {

            //std::cout << "######## IN C_FIN FUNCTION #############: " << m_pitch_rel << std::endl;
            if (abs(m_pitch_rel) < 0.09) {
                //std::cout << "######## IN pitch_rel FUNCTION #############" << std::endl;
                m_map_request_sended = sendInitialPose();
            }

        }
    
    }
}

void Navigation::setamclX(double x){
    m_amcl_pX = x;

}

void Navigation::setamclY(double y){
    m_amcl_pY = y;
}

void Navigation::setTact(int wait){
    m_tact = wait;
}

void Navigation::setMapArea(int area){
    m_area = area;
    //std::cout << "m_area: " << m_area << std::endl; 
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
    m_goal_sended = goalsended;
}

void Navigation::setInitialsended(bool initial_send){
    m_initial_sended = initial_send;
}

bool Navigation::getSendInitial(){
    return m_send_initial;
}

bool Navigation::getSendGoal(){
    navigation_step();
    std::cout << "CALL NAVIGATION_STEP" << std::endl;
    return m_send_goal;
}

void Navigation::setPitchRel(float pitch_rel){
    m_pitch_rel = pitch_rel; 
}




