#include "localisation.h"

using namespace std;

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
void Localisation::setPosOrientation(float x_orient, float y_orient, float z_orient, float w_orient){
    m_x = 1.0;
    m_y = 1.0;
    
    m_x_orient = x_orient;
    m_y_orient = y_orient;
    m_z_orient = z_orient;
    m_w_orient = w_orient;

    calcualateYawZ();
}

void Localisation::calcualateYawZ(){
    // convert quaternion in euler angle
    float t0 = +2.0 * (m_w_orient * m_x_orient + m_y_orient * m_z_orient);  
    float t1 = +1.0 - 2.0 * (m_x_orient * m_x_orient + m_y_orient * m_y_orient);
    m_roll_x = atan2(t0, t1);

    float t2 = +2.0 * (m_w_orient * m_y_orient - m_z_orient * m_x_orient);
    // case differentation
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

    // the recognize_area-function identifies the area the robot drives on
    // Based on the recognition the maps are loaded by the map-loader

    float diff_x_y = abs(m_pitch_y) + abs(m_roll_x);

    // conditions of the if- and else if- statements are angles and positions measured by IMU and AMCL to identify the spesific area
    if ((diff_x_y < 0.20)&&(m_amcl_x < 2.1)){

        m_area = 1; // straight 1

    } else if ((m_amcl_x >= 2.2)&&(diff_x_y > 0.20)&&((((m_pitch_y >= -0.27)&&(m_pitch_y < 0.0))&&((abs(m_yaw_z) >= 0.0)&&(abs(m_yaw_z) < (M_PI / 2.0)))) || (((m_pitch_y <= 0.27)&&(m_pitch_y >= 0.0))&&((abs(m_yaw_z) >= (M_PI / 2.0))&& (abs(m_yaw_z) <= M_PI))))) {

        m_area = 2; //ramp 1

    } else if ((diff_x_y > 0.20)&&((((m_pitch_y <= 0.27)&&(m_pitch_y > 0.0))&&((abs(m_yaw_z) >= 0.0)&&(abs(m_yaw_z) < (M_PI / 2.0)))) || (((m_pitch_y >= -0.27)&&(m_pitch_y <= 0.0))&&((abs(m_yaw_z) >= (M_PI / 2.0))&& (abs(m_yaw_z) <= M_PI))))) {
        
        m_area = 3; //ramp 2

    } else if ((diff_x_y < 0.20)&&(m_amcl_x > 4.0)){

        m_area = 4; // straight 2
    }
}

void Localisation::determine_initialpose() {
    
    // sensor offset in x-direction, only needed, when not noted in eduard.sdf 
    // As it is noted in eduard.sdf the offset is 0.0

    m_sensor_offset_x = 0.0; 
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

    // adding offset to distance measurements
    m_dist0_r = m_dist0 + m_off0;
    m_dist90_r = m_dist90 + m_off90;
    m_dist180_r = m_dist180 + m_off180;
    m_dist270_r = m_dist270 + m_off270;


    // equialize pitch when ramp in x-direction
    m_ramp_angle1 = (15.0 / 180.0) * M_PI; // ankle ramp of TER0_ramp = 15 degree
    m_gamma1 = M_PI - m_ramp_angle1; // 165 deg. 
    m_ramp_angle2 = (75.0 / 180.0) * M_PI; // ankle between ramp and wall = 75 deg. 
    m_gamma2 = M_PI - m_ramp_angle2;    // 105 deg. 
    m_yaw180 = 0.0; // yaw-angle at ray 180 deg. is 0 deg

    recognize_area();
    
    switch (m_area){
    case 1: // steight 1
        {
        
        m_pitch_y_strich = m_pitch_y;
        
        m_x_map_origin_off = -0.472; // Offset caused by map Origin-Point, see map-config-file
        m_y_map_origin_off = -0.617;
        
        // Initial-Position
        m_x_pos = m_dist0_r + m_x_map_origin_off;
        m_y_pos = m_dist90_r + m_y_map_origin_off;    
        
        break;
        
        }
    case 2: // ramp 1
        {

        m_pitch_y_strich = abs(m_pitch_y) - m_ramp_angle1;  // calculating relative pitch to ramp-level

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
            m_beta180 = M_PI - m_ramp_angle2 - abs(m_pitch_y_strich);
            m_dist180_r = m_dist180_r * (sin(m_beta180) / sin(m_ramp_angle2));

            // ramp1 to streight 1 -> pos. pitch 
            m_beta0 = M_PI - m_ramp_angle1 - abs(m_pitch_y_strich);
            m_dist0_r = m_dist0_r * (sin(m_beta0) / sin(m_ramp_angle1)) + 1.0; //offset because robot drove on streight 1
        }

        float level_lenght_ramp1 = 2.553;   // length of the level of ramp1
        m_x_map_origin_off = 1.411;
        m_y_map_origin_off = 0.68;
        float senor_height_off = 0.5255;    // Offset caused because of the laser-scanner is 0.139. Value the map is longer then the level.
        
        // Initial-Position
        m_x_pos = level_lenght_ramp1 - m_dist180_r + m_x_map_origin_off + senor_height_off; 
        m_y_pos = m_dist90_r - m_y_map_origin_off;      
        
        break;
        }
    case 3: //ramp 3
        {

        m_pitch_y_strich = abs(m_pitch_y) - m_ramp_angle1;

        if (m_pitch_y_strich < 0.0) //positive pitch
        {   
            m_beta0 = M_PI - m_pitch_y_strich - m_ramp_angle2;
            m_dist0_r = (sin(m_beta0) / sin(m_ramp_angle2)) * m_dist0_r;
        } else if (m_pitch_y_strich > 0.001)    //negative pitch
        {   
            m_beta0 = M_PI - m_pitch_y_strich - m_gamma2;
            m_dist0_r = (sin(m_beta0) / sin(m_gamma2)) * m_dist0_r;
        }

        if(isinf(m_dist0_r)) {

            m_x_map_origin_off = 1.985;
            m_y_map_origin_off = - 0.617;
            m_manual_correction_off = 0.5; // offset to make the keepout-filter fitting for ramp 2

            // Initial-Position
            m_x_pos = m_dist0_r + m_x_map_origin_off + m_manual_correction_off ; 
            m_y_pos = m_dist90_r + m_y_map_origin_off; 

        } else {
            m_global_pos_left_wall = 1.9;   // using left wall of ramp2 as orientation-point
            
            m_x_map_origin_off = 1.985;
            m_y_map_origin_off = - 0.617;
            m_manual_correction_off = 0.402;

            // Initial-Position
            m_x_pos = m_dist0_r + m_x_map_origin_off + m_manual_correction_off; 
            m_y_pos = m_global_pos_left_wall -  m_dist270_r;  
        }  
        
        break;
        }
        

    case 4:
        {
        m_pitch_y_strich = m_pitch_y;
        
        m_global_pos_left_wall = 1.9;   // using left wall of streight 2 as orientation-point
        m_x_map_origin_off = 4.376;
        m_manual_correction_off = 0.5;
        float level_lenght_streight2 = 2.43;
        
        // Initial-Position
        m_x_pos = m_x_map_origin_off + m_manual_correction_off + level_lenght_streight2 - m_dist180_r;
        m_y_pos =  m_global_pos_left_wall - m_dist270_r;
        
        break;
        }
    }

}

float Localisation::getX(){
    return m_x;
}

float Localisation::getMapArea(){
    return m_area;
}

float Localisation::getpitch_rel(){
    return m_pitch_y_strich;
}

float Localisation::getY(){
    return m_y;
}

float Localisation::getYawZ(){
    return m_yaw_z;
}

float Localisation::getInitialposeX(){
    return m_x_pos;
}

float Localisation::getInitialposeY(){
    return m_y_pos;
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


