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

    float m_roll_x;
    float m_pitch_y;
    float m_yaw_z;

    float m_dist0;
    float m_dist90;
    float m_dist180;
    float m_dist270;

    float m_x_pos;
    float m_y_pos;

    int m_area;
    float m_pitch_y_strich;

    float m_amcl_x;
    float m_amcl_y;

        

    void calcualateYawZ();

    public:
    Localisation();

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