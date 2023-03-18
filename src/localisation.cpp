#include "localisation.h"

Localisation::Localisation(Control *control){
    m_x = 0;
    m_y = 0;
    m_x_orient = 0;
    m_y_orient = 0;
    m_z_orient = 0;
    m_w_orient = 0;
    m_roll_x = 0;
    m_pitch_y = 0;
    m_yaw_z = 0;

    m_control = control;
}
void Localisation::setPosOrientation(float x_orient, float y_orient, float z_orient, float w_orient){  //float x, float y,
    m_x = 1; //FALSCH GEÄNDET -> muss x sein
    m_y = 1; //FALSCH GEÄNDET -> muss y sein
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
    m_roll_x = atan2(t0, t1);

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
    m_pitch_y = asin(t2);
    

    float t3 = +2.0 * (m_w_orient * m_z_orient + m_x_orient * m_y_orient);
    float t4 = +1.0 - 2.0 * (m_y_orient * m_y_orient + m_z_orient * m_z_orient);
    m_yaw_z = atan2(t3, t4);


    m_control->setPosYaw(m_x, m_y, m_yaw_z);
}

void Localisation::recognize_area() {
    //std::cout << "recoginze" << std::endl;

    // recognize ramp 
    m_diff_x_y = abs(m_pitch_y) + abs(m_roll_x);
    //std::cout << "m_diff_x_y: " << m_diff_x_y << std::endl;


    if ((m_diff_x_y < 0.20)&&(m_amcl_x < 3.0)){
        m_area = 1; // streight 1
        //std::cout << "Gerade 1 " << std::endl;
    } else if((m_diff_x_y > 0.20)&&((((m_pitch_y >= -0.27)&&(m_pitch_y < 0.0))&&((abs(m_yaw_z) >= 0.0)&&(abs(m_yaw_z) < (M_PI / 2.0)))) || (((m_pitch_y <= 0.27)&&(m_pitch_y >= 0.0))&&((abs(m_yaw_z) >= (M_PI / 2.0))&& (abs(m_yaw_z) <= M_PI))))) {
        //std::cout << "Rampe 1" << std::endl;
        m_area = 2; //ramp 1
    } else if ((m_diff_x_y > 0.20)&&((((m_pitch_y <= 0.27)&&(m_pitch_y > 0.0))&&((abs(m_yaw_z) >= 0.0)&&(abs(m_yaw_z) < (M_PI / 2.0)))) || (((m_pitch_y >= -0.27)&&(m_pitch_y <= 0.0))&&((abs(m_yaw_z) >= (M_PI / 2.0))&& (abs(m_yaw_z) <= M_PI))))) {
        //std::cout << "Rampe 2" << std::endl;
        m_area = 3; //ramp 2
    } else if ((m_diff_x_y < 0.20)&&(m_amcl_x > 4.0)){
        m_area = 4;
    }
}

void Localisation::determine_initialpose() {
    m_sensor_offset_x = 0.0; //0.09 -> only needed, when not noted in eduard.sdf // sensor offset in x-direction
    m_yawZ_strich = m_yaw_z + M_PI;
    


    //calculate sensor offset
    
    if (m_yawZ_strich < ( M_PI / 2.0 )) 
    {   
        m_alpha = ( M_PI / 2.0 ) - m_yawZ_strich;
        m_x_off = m_sensor_offset_x * sin(m_alpha);
        m_y_off = m_sensor_offset_x * cos(m_alpha);

        m_off0 = m_x_off;
        m_off90 = m_y_off;
        m_off180 = - m_x_off;
        m_off270 = - m_y_off;
    } else if ((m_yawZ_strich > (M_PI / 2.0)) && (m_yawZ_strich < M_PI)) 
    {
        m_alpha = m_yawZ_strich - (M_PI / 2.0);
        m_x_off = m_sensor_offset_x * sin(m_alpha);
        m_y_off = m_sensor_offset_x * cos(m_alpha);

        m_off0 = - m_x_off;
        m_off90 = m_y_off;
        m_off180 = m_x_off;
        m_off270 = - m_y_off;
    } else if ((m_yawZ_strich > M_PI) && (m_yawZ_strich < (1.5*M_PI)))
    {
        m_alpha = (1.5*M_PI) - m_yawZ_strich;
        m_x_off = m_sensor_offset_x * sin(m_alpha);
        m_y_off = m_sensor_offset_x * cos(m_alpha);

        m_off0 = - m_x_off;
        m_off90 = - m_y_off;
        m_off180 = m_x_off;
        m_off270 = m_y_off;
    } else if ((m_yawZ_strich < (2*M_PI)) && (m_yawZ_strich > (1.5*M_PI)))
    {
        m_alpha = m_yawZ_strich - (1.5*M_PI);
        m_x_off = m_sensor_offset_x * sin(m_alpha);
        m_y_off = m_sensor_offset_x * cos(m_alpha);

        m_off0 = - m_x_off;
        m_off90 = - m_y_off;
        m_off180 = m_x_off;
        m_off270 = m_y_off;
    }

    m_dist0_r = m_dist0 + m_off0;
    m_dist90_r = m_dist90 + m_off90;
    m_dist180_r = m_dist180 + m_off180;
    m_dist270_r = m_dist270 + m_off270;


    // equialize pitch when ramp in x-direction
    m_ramp_angle1 = (15.0 / 180.0) * M_PI; // ankle ramp = 15 degree
    m_gamma1 = M_PI - m_ramp_angle1; // 165 deg. 
    m_ramp_angle2 = (75.0 / 180.0) * M_PI; // ankle between ramp and wall / 75 deg. 
    m_gamma2 = M_PI - m_ramp_angle2;    // 105 deg. 
    m_yaw180 = 0.0; // yaw-angle at ray 180 deg. is 0 deg

    recognize_area();
    
    switch (m_area){
    case 1: // steight 1
        
        if (m_pitch_y < 0.0){ // exqualize pitch of laserray 180
            m_beta = M_PI - m_gamma;
            m_dist180_r = m_dist180_r * (sin(m_beta) / sin(m_gamma));
        } else if (m_pitch_y > 0.001)
        {   
            m_beta = M_PI - m_ramp_angle1 - abs(m_pitch_y);
            m_dist180_r = m_dist180_r * (sin(m_beta) / sin(m_ramp_angle1));
        }        
        break;
    case 2: // ramp 1
        {
        m_relYaw = m_yaw180 + m_yaw_z;
        
        m_delta_pitch = m_relYaw * sin(m_pitch_y); // without roll impact

        double c_roll = cos(m_roll_x);
        double s_roll = sin(m_roll_x);
        double P_axis_roll[3] = {0, sin(m_pitch_y)*c_roll, cos(m_pitch_y)* c_roll + s_roll * sin(m_pitch_y)};

        m_pitch_y_strich = abs(m_pitch_y) - m_ramp_angle1;
        //td::cout << "pitch_strich: " << pitch_new << std::endl;


        if (m_pitch_y_strich < 0.0) //positive pitch
        {   
            // ramp1 to wall -> neg. pitch
            m_beta180 = M_PI - m_gamma2 - abs(m_pitch_y_strich);
            m_dist180_r =  m_dist180_r * (sin(m_beta180) / sin(m_gamma2));
            
            // ramp1 to streight 1 -> neg. pitch 
            m_beta0 = M_PI - m_gamma1 - abs(m_pitch_y_strich);
            m_dist0_r = m_dist0_r * (sin(m_beta0) / sin(m_gamma1));

        } else if (m_pitch_y_strich > 0.001)    //negative pitch
        {   
            // ramp1 to wall -> pos. pitch
            m_beta180 = M_PI - m_ramp_angle2 + abs(m_pitch_y_strich);
            m_dist180_r = m_dist180_r * (sin(m_beta180) / sin(m_ramp_angle2));

            // ramp1 to streight 1 -> pos. pitch 
            m_beta0 = M_PI - m_ramp_angle1 - abs(m_pitch_y_strich);
            m_dist0_r = m_dist0_r * (sin(m_beta0) / sin(m_ramp_angle1)) + 1.0; //offset because robot drove on streight 1
        }
        break;
        }
    case 3: //ramp 3



        break;

    case 4:

        break;
    }
    //std::cout << "beta: " << m_beta << std::endl;
    //std::cout << "sinus beta :" << sin(m_beta) << std::endl;
    //std::cout << "sinus mramp angle: " << sin(m_ramp_angle) << std::endl;
    //std::cout << "sinus m_gamma: " << sin(m_gamma) << std::endl;


    //std::cout << "off_0: " << m_off0 << std::endl;
    //std::cout << "off_90: " << m_off90 << std::endl;
    //std::cout << "off_180: " << m_off180 << std::endl;
    //std::cout << "off_270: " << m_dist270 << std::endl;

    m_dist0_r = m_dist0_r - 0.524;  // Wie können die Zahlen durch den Laser bestimmt werden?
    m_dist90_r = m_dist90_r - 0.617;    //0.617

    //std::cout << "msg_slan_scan_0: " << m_dist0_r << std::endl;
    //std::cout << "msg_slan_scan_90: " << m_dist90_r << std::endl;
    //std::cout << "msg_slan_scan_180: " << m_dist180_r << std::endl;
    //std::cout << "msg_slan_scan_270: " << m_dist270_r << std::endl;

    //std::cout << "AREA: " << m_area << std::endl;
    //std::cout << "amclX: " << m_amcl_x << std::endl;
    //std::cout << "amclY: " << m_amcl_y << std::endl;

    //std::cout << "roll_x:" << m_roll_x<< std::endl;
    //std::cout << "pitch_y:" << m_pitch_y << std::endl;
    //std::cout << "yaw_z:" << m_yaw_z << std::endl;

}

float Localisation::getX(){
    return m_x;
}

float Localisation::getMapArea(){
    return m_area;
}

float Localisation::getY(){
    return m_y;
}

float Localisation::getYawZ(){
    return m_yaw_z;
}

float Localisation::getInitialposeX(){
    return m_dist0_r;
}

float Localisation::getInitialposeY(){
    return m_dist90_r;
}

float Localisation::getInitialposeZ(){
    return 0.0;
}

float Localisation::getInitialorientX(){
    return  m_x_orient;
}

float Localisation::getInitialorientY(){
    return m_y_orient;
}

float Localisation::getInitialorientZ(){
    return m_z_orient;
}

float Localisation::getInitialorientW(){
    return m_w_orient;
}

void Localisation::setdist0(float dist0){
    m_dist0 = dist0;
}

void Localisation::setdist90(float dist90){
    m_dist90 = dist90;
}

void Localisation::setdist180(float dist180){
    m_dist180 = dist180;
}

void Localisation::setdist270(float dist270){
    m_dist270 = dist270;
}

void Localisation::setAmclX(float amclX){
    m_amcl_x = amclX;

}

void Localisation::setAmclY(float amclY){
    m_amcl_y = amclY;

}


