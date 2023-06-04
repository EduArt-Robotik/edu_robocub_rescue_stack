#include "navigation.h"

Navigation::Navigation(LoadMap *loadMap) { 

    m_loadMap = loadMap;
    m_amcl_pX = 0;
    m_amcl_pY = 0;
    m_area = 1;
    m_area_saved = 1;

    m_goal_pX = 0.0;
    m_goal_pY = 0.0;
    m_goal_pZ = 0.0;
    m_goal_oX = 0.0;
    m_goal_oY = 0.0; 
    m_goal_oZ = 0.0;
    m_goal_oW = 0.0;

    m_goal_achived = false;
    m_new_goal_set = false;

    m_send_goal = false;
    m_goal_sended = false;
    m_start_area1 = true;
    m_map_sended = false;
    m_send_initial = false;
    m_initial_sended = false;
    m_turn = false;

    m_send_goalpose = false;

    m_travel_forward = true;
    m_travel_backwards = false;

    m_c_start = false;
    m_c_fin = false;

    string repository_path = "/home/daniel/ros2_ws/src/edu_robocub_rescue_stack";
    m_map1 = repository_path + "/map/map_1.yaml";
    m_map2 = repository_path + "/map/map_2.yaml";
    m_map3 = repository_path + "/map/map_3.yaml";
    m_map4 = repository_path + "/map/map_4.yaml";
}

    /*
    General process of the navigation:
    1. map_swap function recognices change of area
    2. initialize function initializes the control variables new
    3. navigation_step function chooses goal depending on moving-dircetion and area
    4. navigate-function is called
    4.1 navigatie-function calls loadmap-function to load the map depending on the area
    4.2 after map is loaded the navigate-function calls the sendGoalPose-function to send the goal depending on moving-direction and area
    4.3 after the goal is sended the navigate-function calls the sendInitialPose-function to send the initial-pose 
    */


void Navigation::initialize() {

    // initializing the control variables
    m_send_goal = false;
    m_goal_sended = false;
    
    m_map_sended = false;

    m_loadmapStatus = false;
    m_map_request_sended = false;
    
    m_send_initial = false;
    m_initial_sended = false;

    m_c_start = false;

}

bool Navigation::counter(bool c_start_) {
    
    if (c_start_) {
        m_count = 0;
        c_start_ = false;
    }

    m_count = m_count + 1;
    
    // counts 1 second, because the timer calls the function every 500ms
    if (m_count >= 2) {

        return true;
    
    } else {

        return false;
    
    }

}

bool Navigation::sendGoalPose(){

    if (!m_goal_sended) {
        m_send_goal = true;
        return false;  
    } else {
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
        m_c_start = true;
        m_area_saved = m_area;
        initialize();
    }

}

void Navigation::navigation_step(){

    map_swap(m_area);

    if (m_area == 1) {
        if ( (m_amcl_pX <= 0.1) && m_turn ) {
            m_travel_backwards = false;
            m_travel_forward = true;
            m_turn = false;
            initialize();
        }
        if (m_travel_forward) {           
            
            // goal in map_1 of streight one, when traveling in forward-direction

            // position
            m_goal_pX = 2.35; 
            m_goal_pY = - 0.05;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {
            
            //goal in map_1 of streight one, when traveling backwards

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

            // first goal on the map_2 of ramp1, when traveling forward
            // position
            m_goal_pX = 2.85; 
            m_goal_pY = -0.05;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {

            // goal on the map_2 of ramp 1, when traveling backwards
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

            // second goal on map_2, when traveling forwards, is in reality located on ramp 2  
            // position
            m_goal_pX = 4.0; 
            m_goal_pY = 1.2;
            m_goal_pZ = 0.0;

            // orientation
            m_goal_oX = 0.0;
            m_goal_oY = 0.0;
            m_goal_oZ = 0.0;
            m_goal_oW = 1.0;

        } else if (m_travel_backwards) {
            
            // goal on the map_2 of ramp 1, when traveling backwards
            // position
            m_goal_pX = 1.8; 
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
            
            // goal in map_3 of ramp 2, when traveling forward
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

            // first goal in map_3 on ramp 2, when traveling backwards
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
            
            // goal in map_3 of ramp 2, when traveling forward
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

            // second goal in map_3, when traveling backwards, is in reality on ramp 1
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

            // goal in map_4, when traveling forwards
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

            // goal in map_4, when traveling backwards
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
        m_c_start = true; 

    } else if (m_send_initial){

        m_c_fin = counter(m_c_start);
        m_c_start = false;

        if (m_c_fin) {

            if (abs(m_pitch_rel) < 0.09) {

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

void Navigation::setMapArea(int area){
    m_area = area;
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
    return m_send_goal;
}

void Navigation::setPitchRel(float pitch_rel){
    m_pitch_rel = pitch_rel; 
}




