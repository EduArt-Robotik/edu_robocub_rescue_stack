#include "control.h"

/*
navigation: 
0 geradeaus bis punktfahren
1 geradeaus weiter fahren
2 rampe hoch fahren
3 winkel fahren - vor knick
4 rampe runter fahren
5 gerade Fahren zu point
6 gerade Fahren zu ende
7 gerade zurück bis punkt fahren
8 gerade zurück fahren
9 rampe hoch fahren
10 winkel fahren - vor knick
11 rampe nach unten fahren
12 geradeaus fahren zu point
13 geradeaus fahren zu start
*/

Control::Control(ClientService *map1ServerService, ClientService *map2ServerService, ClientService *map3ServerService, ClientService *map4ServerService) {
    m_map1ServerService = map1ServerService;
    m_map2ServerService = map2ServerService;
    m_map3ServerService = map3ServerService;
    m_map4ServerService = map4ServerService;

    m_yaw_z = 0;
    m_x = 0;
    m_y = 0;
    m_angle = 0;
    m_speed = 0;
    m_x_dest = 1;
    m_y_dest = 0.1;
    m_navigationStep = 0;
    m_dest_precision = 0.15;
    m_driving_direction = 1;
    m_slowDown = 20;

    m_activeMapServer = m_map1ServerService;
}

void Control::calculateAngleSpeed() {

    //Incase the roboter drives against a wall
    if(m_pitch_y < -0.9 || m_pitch_y > 0.9){
        m_speed = m_driving_direction*-1;
        m_angle = 0;
        m_error_clim_up++;
        std::cout<<"climbs up the wall"<<std::endl;
     
        return;
    }
    else {
    }

    //navigation point move up
    if(m_navigationStep == 1){
        //when roboter is drivng up the ramp
        //Switches maps from map1 to map2
        if(m_x > 2 && (m_pitch_y < -0.2 )){   
            nextNavigationStep();
        }
    }
    else if( m_navigationStep == 3){
        //if the roboter switches ramps
        //Switches maps from map2 to map3, slows down Roboter and sets new destination point
        if( (m_pitch_y >= 0.10) && (m_y > 0.3) && (m_x > 2.5) && (m_roll_x > 0)) {
            nextNavigationStep();
        }
    }
    else if(m_navigationStep == 4){
        //if the roboter arrives at ground level
        //switches maps form map3 to map4
        if(m_x > 4.1 && (m_pitch_y < 0.2 || m_pitch_y > -0.2) &&  (m_roll_x > -0.2 || m_roll_x < 0.2)) {
            nextNavigationStep();
        }  
    }
    else if(m_navigationStep == 8){
        //when roboter is drivng up the ramp
        //Switches maps from map4 to map3
        if(m_x < 4.5  &&  (m_pitch_y > 0.2 )) {
            nextNavigationStep();
        }  
    }
    else if( m_navigationStep == 10){
        //if the roboter switches ramps
        //Switches maps from map3 to map2
        if( (m_pitch_y <= -0.10 ) && (m_y < 0.8) && (m_x < 4) && (m_roll_x < 0)) {
            nextNavigationStep();
        }
    }
    else if(m_navigationStep == 11){
        //if the roboter arrives at ground level
        //switches maps form map2 to map1
        if(m_x < 2 && (m_pitch_y < 0.2 || m_pitch_y > -0.2) &&  (m_roll_x > -0.2 || m_roll_x < 0.2)) {
            nextNavigationStep();
        }  
    }

    //delta_x and delta_y are needed to calculate the angle and distance
    float delta_x = m_x_dest - m_x;
    float delta_y = m_y_dest - m_y;
    
    //Ausgabe
    //std::cout << "x:" << x <4<< std::endl;
    //std::cout << "delta_x:" << delta_x << std::endl;
    //std::cout << "delta_y:" << delta_y << std::endl;
    
    float delta_dest = sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    //std::cout << "delta_dest:" << delta_dest << std::endl;
    //driving backwarts: adding 180 degree
    float delta_phi;
    if(m_driving_direction == -1 ){
        m_yaw_z = m_yaw_z+M_PI;
        angleOverTwoPi(m_yaw_z);
    }
    delta_phi = (atan2(delta_y, delta_x))- m_yaw_z; 

    delta_phi = angleOverTwoPi(delta_phi);


    //std::cout << "delta_phi:" << delta_phi << std::endl;
    
    m_angle = delta_phi;
    
    if(m_activeMapServer->getState()!= lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE){
        m_speed = 0;
        m_angle = 0;
        std::cout <<"map not active"<<std::endl;
        return;
    }
    //while turning roboter drives real slow and is slowed down afterwards too

    if((delta_phi > m_dest_precision) || (delta_phi < -m_dest_precision)) {
        m_speed = 0.001;
        m_slowDown = 200;
        m_slowDownReduce = 20;
    } 
    else {
        //slow Down calculation
        m_slowDown = m_slowDown - m_slowDownReduce;
        if( m_slowDown < 20){
            m_slowDown = 20;
        }
        // speed calculation
        m_speed = (delta_dest) / (float) m_slowDown;
        if(m_speed < 0.3){
            m_speed  = 0.3;   
        }
    } 
    //aim calculation // aim radius depends on control state
    if ((delta_dest < m_dest_precision)) {
        nextNavigationStep();
        calculateAngleSpeed();
    }
    m_speed = m_driving_direction*m_speed;

}

void Control::nextNavigationStep(){
    m_navigationStep++;
    m_navigationStep = m_navigationStep%14;
    setNaviagtionStep();
}

void Control::previousNavigationStep(){
    m_navigationStep--;
    m_navigationStep = m_navigationStep%14;
    setNaviagtionStep();
}

void Control::setNaviagtionStep(){

    if( m_navigationStep == 0 ){
        m_x_dest = 1;
        m_y_dest = 0;
        m_dest_precision = 0.3;
        m_driving_direction = 1;
    }
    else if( m_navigationStep == 1 ){
        m_x_dest = 3.3;
        m_y_dest = 0;
        m_dest_precision = 0.15;

    }
    else if( m_navigationStep == 2 ){
        m_activeMapServer = m_map2ServerService;
        m_map1ServerService->deactiveService();
    }
    else if( m_navigationStep == 3 ){
        m_x_dest = 3.6;
        m_y_dest = 1.3;
        m_dest_precision = 0.2;
        m_slowDown = 100;
        m_slowDownReduce = 20;

    }
    else if( m_navigationStep == 4 ){
        //new dest
        m_x_dest = 5;
        m_y_dest = 1.3;
        m_dest_precision = 0.4;
        //slows Roboter down
        m_slowDown = 100;
        m_slowDownReduce = 2;
        //switches map
        m_activeMapServer = m_map3ServerService;
        m_map2ServerService->deactiveService();
    }
    else if( m_navigationStep == 5){
        //m_slowDown = 50;
        //m_slowDownReduce = 20;
        m_activeMapServer = m_map4ServerService;
        m_map3ServerService->deactiveService();
    }
    else if( m_navigationStep == 6 ){
        m_x_dest = 6;
        m_y_dest = 1.3;
        m_dest_precision = 0.2;

    }
    else if( m_navigationStep == 7 ){
        m_x_dest = 5;
        m_y_dest = 1.3;
        m_dest_precision = 0.3;
        m_driving_direction =-1;

    }
    else if( m_navigationStep == 8 ){
        m_x_dest = 3.3;
        m_y_dest = 1.3;
        m_dest_precision = 0.2;

    }
    else if ( m_navigationStep == 9){
        //m_slowDown = 200;
        //m_slowDownReduce = 20;
        m_activeMapServer = m_map3ServerService;
        m_map4ServerService->deactiveService();
    }
    else if( m_navigationStep == 10 ){
        m_x_dest = 3.0;
        m_y_dest = 0;
        m_dest_precision = 0.2;
        m_slowDown = 100;
        m_slowDownReduce = 20;

    }
    else if( m_navigationStep == 11 ){
        m_x_dest = 1;
        m_y_dest = 0;
        m_dest_precision = 0.4;
        m_slowDown = 100;
        m_slowDownReduce = 2;
        m_activeMapServer = m_map2ServerService;
        m_map3ServerService->deactiveService();
    }
    else if( m_navigationStep == 12){
        m_slowDown = 50;
        m_slowDownReduce = 20;
        m_activeMapServer = m_map1ServerService;
        m_map2ServerService->deactiveService();
    }
    else if( m_navigationStep == 13 ){
        m_x_dest = 0.4;
        m_y_dest = 0;
        m_dest_precision = 0.2;
    }
}

void Control::setPosYaw(float x,float y,float yaw_z){
    m_x = x;
    m_y = y;
    m_yaw_z = yaw_z;
    calculateAngleSpeed();
}

void Control::setPitchY(float pitch_y){
    m_pitch_y = pitch_y;
}

void Control::setYawZ(float yaw_z){
    m_yaw_z = yaw_z;
    calculateAngleSpeed();
}

void Control::setRollX(float roll_x){
    m_roll_x = roll_x;
}

void Control::setX(float x){
    m_x = x;
    calculateAngleSpeed();
}

void Control::setY(float y){
    m_y = y;
    calculateAngleSpeed();
}

float Control::getAngle(){
    return m_angle;
}

float Control::getSpeed(){
    return m_speed;
}

float Control::angleOverTwoPi(float angle){
    if(angle < -M_PI){
        angle += 2*M_PI;
        }
    if(angle > M_PI){
        angle -= 2*M_PI;
        };
    return angle;
}

int Control::getNavigationStep(){
    return m_navigationStep;
}
