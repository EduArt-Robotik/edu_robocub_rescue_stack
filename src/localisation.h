#include "control.h"

class Localisation{

    private:

    //Initialisierung Variablen
    float m_x;
    float m_y;
    float m_x_orient;
    float m_y_orient;
    float m_z_orient;
    float m_w_orient;

    float m_roll_x;
    float m_pitch_y;
    float m_yaw_z;
    Control *m_control; 

    void calcualateYawZ();

    public:
    Localisation(Control *control);

    void setPosOrientation(float x_orient, float y_orient, float z_orient, float w_orient); //float x, float y, 
    float getX();
    float getY();
    float getYawZ();
};