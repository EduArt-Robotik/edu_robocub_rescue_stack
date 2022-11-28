#include "localisation.h"

Localisation::Localisation(Control *control, AmclService *amclService){
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
    m_amclService = amclService;
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
    
    //Ausgabe
    std::cout << "yaw_z:" << m_yaw_z << std::endl;
    m_control->setPosYaw(m_x, m_y, m_yaw_z);
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

bool Localisation::amclSetup()
{
    //configure
    unsigned int state = m_amclService->get_state();

    if(state == lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED){
        if(!m_amclService->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE))
        {
        return false;
        }

        state = m_amclService->get_state();
    }
    else {
        return false;
    }
    //activate
    if( state == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE){
        if(!m_amclService->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
        {
        return false;
        }

        if (!m_amclService->get_state())
        {
        return false;
        }
    }
    else{
        return false;
    }

    return true;
}   
  
/*
  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //activate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //deactivate
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //cleanup
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }

  //unconfigured shutdown
  {
    time_between_state_changes.sleep();
    if(!service_client->change_state(lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN))
    {
      return;
    }

    if (!service_client->get_state())
    {
      return;
    }
  }*/