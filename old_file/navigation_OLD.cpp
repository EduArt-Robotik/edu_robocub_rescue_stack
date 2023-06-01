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

    m_send_goalpose = false;

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
    
    m_send_initial = false; // changed to false, because of changed order. Used to be True
    m_initial_sended = false;
}

//void Navigation::stop_goal(){
    
//}

void Navigation::sendGoalPose(){

    if (!m_goal_sended) {
        m_send_goal = true;
    } else if (m_goal_sended) {
        m_send_goal = false;

    }   
}

bool Navigation::sendInitialPose(){

    if (!m_initial_sended) {
        m_send_initial = true;
    } else if (m_initial_sended) {
        m_send_initial = false;
        m_map_request_sended = true;    // changed to true, because of changed order
    }   
    return m_map_request_sended;
}



void Navigation::stop_goal(){
    if (m_stop_goal) {
        // position
        m_goal_pX = m_amcl_pX;
        m_goal_pY = m_amcl_pY;
        m_goal_pZ = 0.0;

        // orientation
        m_goal_oX = 0.0;
        m_goal_oY = 0.0;
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0; //anpassen

        sendGoalPose();
        m_stop_goal = false;
        initialize();
    }
}

void Navigation::map_swap(int m_area) {
    if (m_area != m_area_saved){
        initialize();
        m_area_saved = m_area;

    }

}
void Navigation::navigation_step(){
    
    map_swap(m_area);

    if (m_area == 1) {
        std::cout << "navigation step 1" << std::endl;
        m_url = m_map1;
        // position
        m_goal_pX = 1.4;
        m_goal_pY = 0.0;
        m_goal_pZ = 0.0;

        // orientation
        m_goal_oX = 0.0;
        m_goal_oY = 0.0;
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        navigate(m_url);

    } else if (m_area == 2) {
        //stop_goal();
        //std::cout << "navigation step 2" << std::endl;

        m_url = m_map2;
        // position
        m_goal_pX = 3.7; 
        m_goal_pY = 1.3;
        m_goal_pZ = 0.0;

        // orientation
        m_goal_oX = 0.0;
        m_goal_oY = 0.0;
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        navigate(m_url);
        

    } else if (m_area == 3) {
        //stop_goal();
        std::cout << "navigation step 2" << std::endl;

        m_url = m_map3;
        // position
        m_goal_pX = 4.7; 
        m_goal_pY = 1.3;
        m_goal_pZ = 0.0;

        // orientation
        m_goal_oX = 0.0;
        m_goal_oY = 0.0;
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
        navigate(m_url);
        
    } else if (m_area == 4) {
        //stop_goal();
        std::cout << "navigation step 2" << std::endl;

        m_url = m_map4;
        // position
        m_goal_pX = 6.0; 
        m_goal_pY = 1.3;
        m_goal_pZ = 0.0;

        // orientation
        m_goal_oX = 0.0;
        m_goal_oY = 0.0;
        m_goal_oZ = 0.0;
        m_goal_oW = 1.0;
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

    //if (!m_loadmapStatus) {
    //    m_map_request_sended = false;
    //}

}
void Navigation::navigate(string m_url){
    if (!m_map_request_sended) {   
        
        loadMap(m_url);
        std::cout << "##########################################################" << std::endl;
        std::cout << "map loaded################################################" << std::endl;
        std::cout << "##########################################################" << std::endl;
    } else if (m_send_goalpose) {
        
        sendGoalPose();
        m_send_goalpose = false; 
        
        std::cout << "goal Pose sended" << std::endl;
    } else if (m_send_initial){
        if (abs(m_pitch_rel) < 0.09) {
            m_map_request_sended = sendInitialPose();
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
    return m_send_goal;
}

void Navigation::setPitchRel(float pitch_rel){
    m_pitch_rel = pitch_rel; 
}




