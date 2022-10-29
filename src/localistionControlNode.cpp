#include "localisationControlNode.h"

LocalisationControlNode::LocalisationControlNode(): Node("localisation_control")
{
    m_control = new Control();
    m_localisation = new Localisation(m_control);
    
    //subscriber
    subscriber_odom_= this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControlNode::odom_callback, this, std::placeholders::_1));
    subscriber_velocity_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&LocalisationControlNode::velocity_callback, this, std::placeholders::_1));
    //subscriber_state_est_=this->create_subscription<std_msgs::msg::Float64MultiArray>("/state_est", 1, std::bind(&LocalisationControlNode::state_est_callback, this, std::placeholders::_1));
    
    //publisher
    publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    //timer
    timer_ = this->create_wall_timer(50ms, std::bind(&LocalisationControlNode::timer_callback, this));
}

void LocalisationControlNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom) 
{   
    //Position
    float x = msg_odom->pose.pose.position.x;
    float y = msg_odom->pose.pose.position.y;
    //float z_pos = msg_odom->pose.pose.position.z;
    
    //Orientation
    float x_orient = msg_odom->pose.pose.orientation.x;
    float y_orient = msg_odom->pose.pose.orientation.y;
    float z_orient = msg_odom->pose.pose.orientation.z;
    float w_orient = msg_odom->pose.pose.orientation.w;
    
    m_localisation->setPosOrientation(x, y, x_orient, y_orient, z_orient, w_orient);

    std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
    obs_state_vector_x_y_yaw.data = {m_localisation->getX(), m_localisation->getY(), m_localisation->getYawZ()};

    //Input for function publish_estimated_state
    publish_estimated_state(obs_state_vector_x_y_yaw);
}


void LocalisationControlNode::timer_callback()
{
    auto message = geometry_msgs::msg::Twist();
    
    message.linear.x = m_control->getSpeed();
    message.angular.z = m_control->getAngle();

    publisher_vel_->publish(message);
}



void LocalisationControlNode::publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg)
{
    //publishing the estimated pose and orientation
    auto message_pose = std_msgs::msg::Float64MultiArray();
    message_pose.data = state_vec_msg.data;
    //message.data includes x-,y-coordinate and orientation around the z-axis yaw_z
    //pitch_y and role_x are calculated and can be added
    
    publisher_state_est_->publish(message_pose);
}

void LocalisationControlNode::velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel) 
{
    //output angular/linear velocity
    //v = msg_vel->linear.x;
    //yaw_rate = msg_vel->angular.z;
}
