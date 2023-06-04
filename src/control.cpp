
#include "control.h"


Control::Control() {
    m_yaw_z = 0;
    m_x = 0;
    m_y = 0;
    m_angle = 0;
    m_speed = 0;
    m_x_dest = -10;
    m_y_dest = 10;

    m_times = 10;

}


void Control::calculateAngleSpeed() {
    
    float delta_x = m_x_dest - m_x;
    float delta_y = m_y_dest - m_y;
    
    float delta_dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    float delta_phi = (atan2(delta_y, delta_x))- m_yaw_z; 
    
    // limit of angle phi
    if(delta_phi < -M_PI){
        delta_phi += 2*M_PI;
        }
    if(delta_phi > M_PI){
        delta_phi -= 2*M_PI;
        };
    
    m_angle = delta_phi;
    
    if((delta_phi > 0.07) || (delta_phi < -0.07)) {
        m_speed = 0.01;
        m_times = 200;
    }
   
    else {
        m_times = m_times -10;
     if( m_times < 10){
        m_times = 10;
     }
        m_speed = (delta_dist) / (float) m_times;
        if(m_speed < 0.3){
            m_speed  = 0.3;   
        }
    } 
    if ((delta_dist < 0.1)) {
        m_speed = 0.0;
        m_angle = 0.0;
    }

}

void Control::setPosYaw(float x,float y,float yaw_z){
    m_x = x;
    m_y = y;
    m_yaw_z = yaw_z;
    calculateAngleSpeed();
}

void Control::setYawZ(float yaw_z){
    m_yaw_z = yaw_z;
    calculateAngleSpeed();
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