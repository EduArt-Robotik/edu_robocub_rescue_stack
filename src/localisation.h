#include "control.h"
#include <vector>
#include <cmath>
#include <iostream>




class Localisation{

    private:

    float m_x;
    float m_y;
    float m_x_orient;
    float m_y_orient;
    float m_z_orient;
    float m_w_orient;

    float m_x_map_origin_off;
    float m_y_map_origin_off;

    float m_roll_x;
    float m_pitch_y;
    float m_yaw_z;
    float m_yawZ_strich;

    float m_off0;
    float m_off90;
    float m_off180;
    float m_off270;
    float m_alpha;
    float m_x_off;
    float m_y_off;
    float m_sensor_offset_x;
    float m_manual_correction_off;
    float m_global_pos_left_wall;

    float m_dist0;
    float m_dist90;
    float m_dist180;
    float m_dist270;

    float m_dist0_r;
    float m_dist90_r;
    float m_dist180_r;
    float m_dist270_r;

    float m_x_pos;
    float m_y_pos;

    float m_beta;
    float m_beta0;
    float m_beta180;
    float m_delta;
    float m_ramp_angle1;
    float m_ramp_angle2;
    float m_gamma;
    float m_gamma1;
    float m_gamma2;
    float m_delta_pitch;
    float m_relYaw;
    float m_yaw180;

    float m_yaw_z1;
    float m_yaw_z2;

    
    int m_area;
    float m_pitch_y_strich;

    float m_pitch_roll;

    float m_amcl_x;
    float m_amcl_y;

    

    
    Control *m_control; 
    //Navigation *m_navigation;

    void calcualateYawZ();

    public:
    Localisation(Control *control);

    void setPosOrientation(float x_orient, float y_orient, float z_orient, float w_orient); //float x, float y, 
    float getX();
    float getY();
    float getYawZ();
    float getInitialposeX();
    float getInitialposeY();
    float getInitialposeZ();
    float getInitialorientX();
    float getInitialorientY();
    float getInitialorientZ();
    float getInitialorientW();
    float getMapArea();
    float getpitch_rel();

    void setAmclX(float amclX);
    void setAmclY(float amclY);

    void determine_initialpose();
    void setdist0(float dist0);
    void setdist90(float dis90);
    void setdist180(float dist180);
    void setdist270(float dist270);
    void recognize_area();

};