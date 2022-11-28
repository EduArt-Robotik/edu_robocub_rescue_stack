#include "control.h"
#include "amclService.h"

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
    AmclService *  m_amclService;

    void calcualateYawZ();
    bool amclSetup();

    public:
    Localisation(Control *control, AmclService *amclService);

    void setPosOrientation(float x, float y, float x_orient, float y_orient, float z_orient, float w_orient);
    float getX();
    float getY();
    float getYawZ();
};