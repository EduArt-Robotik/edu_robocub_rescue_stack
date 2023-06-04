#include "localisation.h"

using namespace std;

Localisation::Localisation(){
    m_x = 0;
    m_y = 0;
    m_x_orient = 0;
    m_y_orient = 0;
    m_z_orient = 0;
    m_w_orient = 0;
    m_roll_x = 0;
    m_pitch_y = 0;
    m_yaw_z = 0;
    m_dist0 = 0;
    m_dist90 = 0;
    m_dist180 = 0;
    m_dist270 = 0;
    m_x_pos = 0;
    m_y_pos = 0;
    m_area = 0;
    m_pitch_y_strich = 0;
    m_amcl_x = 0;
    m_amcl_y = 0;

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

    float sensor_offset_x = 0.0; 
    float yawZ_strich = m_yaw_z + M_PI;
    
    //calculate sensor offset
        float alpha = 0;
        float off0 = 0;
        float off90 = 0;
        float off180 = 0;
        float off270 = 0;
        float x_off = 0;
        float y_off = 0;

    if (yawZ_strich < ( M_PI / 2.0 )) 
    {   
        alpha = ( M_PI / 2.0 ) - yawZ_strich;
        x_off = sensor_offset_x * sin(alpha);
        y_off = sensor_offset_x * cos(alpha);

        off0 = x_off;
        off90 = y_off;
        off180 = - x_off;
        off270 = - y_off;
    } else if ((yawZ_strich > (M_PI / 2.0)) && (yawZ_strich < M_PI)) 
    {
        alpha = yawZ_strich - (M_PI / 2.0);
        x_off = sensor_offset_x * sin(alpha);
        y_off = sensor_offset_x * cos(alpha);

        off0 = - x_off;
        off90 = y_off;
        off180 = x_off;
        off270 = - y_off;
    } else if ((yawZ_strich > M_PI) && (yawZ_strich < (1.5*M_PI)))
    {
        alpha = (1.5*M_PI) - yawZ_strich;
        x_off = sensor_offset_x * sin(alpha);
        y_off = sensor_offset_x * cos(alpha);

        off0 = - x_off;
        off90 = - y_off;
        off180 = x_off;
        off270 = y_off;
    } else if ((yawZ_strich < (2*M_PI)) && (yawZ_strich > (1.5*M_PI)))
    {
        alpha = yawZ_strich - (1.5*M_PI);
        x_off = sensor_offset_x * sin(alpha);
        y_off = sensor_offset_x * cos(alpha);

        off0 = - x_off;
        off90 = - y_off;
        off180 = x_off;
        off270 = y_off;
    }


    // adding offset to distance measurements
    float dist0_r = m_dist0 + off0;
    float dist90_r = m_dist90 + off90;
    float dist180_r = m_dist180 + off180;
    float dist270_r = m_dist270 + off270;

 

    // equialize pitch when ramp in x-direction
    float ramp_angle1 = (15.0 / 180.0) * M_PI; // ankle ramp of TER0_ramp = 15 degree
    float gamma1 = M_PI - ramp_angle1; // 165 deg. 
    float ramp_angle2 = (75.0 / 180.0) * M_PI; // ankle between ramp and wall = 75 deg. 
    float gamma2 = M_PI - ramp_angle2;    // 105 deg. 

    float manual_correction_off = 0;
    float global_pos_left_wall = 0;
    float x_map_origin_off = 0;
    float y_map_origin_off = 0;
    float beta0 = 0;
    float beta180 = 0;

    recognize_area();
    
    switch (m_area){
    case 1: // steight 1
        {
        
        m_pitch_y_strich = m_pitch_y;
        
        x_map_origin_off = -0.472; // Offset caused by map Origin-Point, see map-config-file
        y_map_origin_off = -0.617;
        
        // Initial-Position
        m_x_pos = dist0_r + x_map_origin_off;
        m_y_pos = dist90_r + y_map_origin_off;    
        
        break;
        
        }
    case 2: // ramp 1
        {

        m_pitch_y_strich = abs(m_pitch_y) - ramp_angle1;  // calculating relative pitch to ramp-level

        if (m_pitch_y_strich < 0.0) //positive pitch
        {   
            // ramp1 to wall -> neg. pitch
            beta180 = M_PI - gamma2 - abs(m_pitch_y_strich);
            dist180_r =  dist180_r * (sin(beta180) / sin(gamma2));
            
            // ramp1 to streight 1 -> neg. pitch 
            beta0 = M_PI - gamma1 - abs(m_pitch_y_strich);
            dist0_r = dist0_r * (sin(beta0) / sin(gamma1));

        } else if (m_pitch_y_strich > 0.001)    //negative pitch
        {   
            // ramp1 to wall -> pos. pitch
            beta180 = M_PI - ramp_angle2 - abs(m_pitch_y_strich);
            dist180_r = dist180_r * (sin(beta180) / sin(ramp_angle2));

            // ramp1 to streight 1 -> pos. pitch 
            beta0 = M_PI - ramp_angle1 - abs(m_pitch_y_strich);
            dist0_r = dist0_r * (sin(beta0) / sin(ramp_angle1)) + 1.0; //offset because robot drove on streight 1
        }

        float level_lenght_ramp1 = 2.553;   // length of the level of ramp1
        x_map_origin_off = 1.411;
        y_map_origin_off = 0.68;
        float senor_height_off = 0.5255;    // Offset caused because of the laser-scanner is 0.139. Value the map is longer then the level.
        
        // Initial-Position
        m_x_pos = level_lenght_ramp1 - dist180_r + x_map_origin_off + senor_height_off; 
        m_y_pos = dist90_r - y_map_origin_off;      
        
        break;
        }
    case 3: //ramp 3
        {

        m_pitch_y_strich = abs(m_pitch_y) - ramp_angle1;

        if (m_pitch_y_strich < 0.0) //positive pitch
        {   
            beta0 = M_PI - m_pitch_y_strich - ramp_angle2;
            dist0_r = (sin(beta0) / sin(ramp_angle2)) * dist0_r;
        } else if (m_pitch_y_strich > 0.001)    //negative pitch
        {   
            beta0 = M_PI - m_pitch_y_strich - gamma2;
            dist0_r = (sin(beta0) / sin(gamma2)) * dist0_r;
        }

        if(isinf(dist0_r)) {

            x_map_origin_off = 1.985;
            y_map_origin_off = - 0.617;
            manual_correction_off = 0.5; // offset to make the keepout-filter fitting for ramp 2

            // Initial-Position
            m_x_pos = dist0_r + x_map_origin_off + manual_correction_off ; 
            m_y_pos = dist90_r + y_map_origin_off; 

        } else {
            global_pos_left_wall = 1.9;   // using left wall of ramp2 as orientation-point
            
            x_map_origin_off = 1.985;
            y_map_origin_off = - 0.617;
            manual_correction_off = 0.402;

            // Initial-Position
            m_x_pos = dist0_r + x_map_origin_off + manual_correction_off; 
            m_y_pos = global_pos_left_wall -  dist270_r;  
        }  
        
        break;
        }
        

    case 4:
        {
        m_pitch_y_strich = m_pitch_y;
        
        global_pos_left_wall = 1.9;   // using left wall of streight 2 as orientation-point
        x_map_origin_off = 4.376;
        manual_correction_off = 0.5;
        float level_lenght_streight2 = 2.43;
        
        // Initial-Position
        m_x_pos = x_map_origin_off + manual_correction_off + level_lenght_streight2 - dist180_r;
        m_y_pos =  global_pos_left_wall - dist270_r;
        
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


