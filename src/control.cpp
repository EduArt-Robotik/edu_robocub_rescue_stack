#include "control.h"

/*
navigation: 
0 geradeaus fahren
1 winkel fahren
2 nach unten Fahren
3 geradeaus fahren
4 winkel fahren
5 nach unten fahren
*/

Control::Control() {
    m_yaw_z = 0;
    m_x = 0;
    m_y = 0;
    m_angle = 0;
    m_speed = 0;
    m_x_dest = 3.1;
    m_y_dest = 0;

    m_navigation = 0;
}

void Control::calculateAngleSpeed() {

    if( m_navigation == 2 || m_navigation == 3){
        // amcl thinks it is on the other side of the ramp
    }

    float delta_x = m_x_dest - m_x;
    float delta_y = m_y_dest - m_y;
    
    //Ausgabe
    //std::cout << "x:" << x << std::endl;
    //std::cout << "y:" << y << std::endl;
    //std::cout << "delta_x:" << delta_x << std::endl;
    //std::cout << "delta_y:" << delta_y << std::endl;
    
    float delta_dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
    
    //Ausgabe
    std::cout << "delta_dist:" << delta_dist << std::endl;
    
    float delta_phi = (atan2(delta_y, delta_x))- m_yaw_z; 
    
    // Begrenzung des Drehwinkels Phi
    if(delta_phi < -M_PI){
        delta_phi += 2*M_PI;
        }
    if(delta_phi > M_PI){
        delta_phi -= 2*M_PI;
        };

    std::cout << "delta_phi:" << delta_phi << std::endl;
    
    m_angle = delta_phi;
    
    if((delta_phi > 0.2) || (delta_phi < -0.2)) {
        m_speed = 0.001;
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
        control_navigation();
        calculateAngleSpeed();
    }

}

void Control::control_navigation(){
    m_navigation++;
    m_navigation = m_navigation%6;
    if( m_navigation == 0 ){
        m_x_dest = 3.1;
        m_y_dest = 0;
    }
    if( m_navigation == 1 ){
        m_x_dest = 3.4;
        m_y_dest = 1.3;
    }
    if( m_navigation == 2 ){
        m_x_dest = 6;
        m_y_dest = 1.3;
    }
    if( m_navigation == 3 ){
        m_x_dest = 3.5;
        m_y_dest = 1.3;
    }
    if( m_navigation == 4 ){
        m_x_dest = 3.0;
        m_y_dest = 0;
    }
    if( m_navigation == 5 ){
        m_x_dest = 0;
        m_y_dest = 0;
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
