#include "localisation.h"

Localisation::Localisation( ){
    m_x = 0;
    m_y = 0;
    m_x_orient = 0;
    m_y_orient = 0;
    m_z_orient = 0;
    m_w_orient = 0;
    m_roll_x = 0;
    m_pitch_y = 0;
    m_yaw_z = 0;

  
    //amclSetup();
}


void Localisation::setPosOrientation(float x, float y, float x_orient, float y_orient, float z_orient, float w_orient){
    m_x = x;
    m_y = y;
    m_x_orient = x_orient;
    m_y_orient = y_orient;
    m_z_orient = z_orient;
    m_w_orient = w_orient;

    calcualateYawZ();
}

void Localisation::setOrientation( float x_orient, float y_orient, float z_orient, float w_orient){
    m_x_orient = x_orient;
    m_y_orient = y_orient;
    m_z_orient = z_orient;
    m_w_orient = w_orient; 
    calcualateYawZ();
}


void Localisation::calcualateYawZ(){
    //convert quaternion in euler angle
    float t0 = +2.0 * (m_w_orient * m_x_orient + m_y_orient * m_z_orient);  
    float t1 = +1.0 - 2.0 * (m_x_orient * m_x_orient + m_y_orient * m_y_orient);
    m_roll_x = angleOverTwoPi(atan2(t0, t1));

    float t2 = +2.0 * (m_w_orient * m_y_orient - m_z_orient * m_x_orient);
    //Fallunterscheidung für asin
    if(t2 > 1.0){
        t2 = 1.0;
    } else {
        t2 = t2;
    }
    if(t2 < -1.0){
        t2 = -1.0;
    } else {
        t2 = t2;
    }
    m_pitch_y = angleOverTwoPi(asin(t2));

    float t3 = +2.0 * (m_w_orient * m_z_orient + m_x_orient * m_y_orient);
    float t4 = +1.0 - 2.0 * (m_y_orient * m_y_orient + m_z_orient * m_z_orient);
    m_yaw_z = angleOverTwoPi(atan2(t3, t4));
}

float Localisation::angleOverTwoPi(float angle){
    if(angle < -M_PI){
        angle += 2*M_PI;
        }
    if(angle > M_PI){
        angle -= 2*M_PI;
        };
    return angle;
}

float Localisation::getX(){
    return m_x;
}

float Localisation::getY(){
    return m_y;
}

float Localisation::getYawZ(){
    return m_yaw_z;
}

float Localisation::getXOrient(){
    return m_x_orient;
}

float Localisation::getYOrient(){
    return m_y_orient;
}

float Localisation::getZOrient(){
    return m_z_orient;
}
float Localisation::getWOrient(){
    return m_w_orient;
}

float Localisation::getPichtY(){
    return m_pitch_y;
}

float Localisation::getRollX(){
    return m_roll_x;
}