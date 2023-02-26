#include "localisationControlNode.h"

LocalisationControlNode::LocalisationControlNode(): Node("localisation_control")
{
    
    m_control = new Control();
    m_localisation = new Localisation(m_control);
    m_navigation = new Navigation();
    //subscriber
    subscriber_odom_= this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControlNode::odom_callback, this, std::placeholders::_1));
    subscriber_velocity_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&LocalisationControlNode::velocity_callback, this, std::placeholders::_1));
    //subscriber_state_est_=this->create_subscription<std_msgs::msg::Float64MultiArray>("/state_est", 1, std::bind(&LocalisationControlNode::state_est_callback, this, std::placeholders::_1));
    subscriber_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/demo/laser/out", rclcpp::QoS(rclcpp::SensorDataQoS()), std::bind(&LocalisationControlNode::scan_callback, this, std::placeholders::_1));
    //###########NEU##############

    subscriber_amcl_pose_= this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 1, std::bind(&LocalisationControlNode::amcl_pose_callback, this, std::placeholders::_1));
    subscriber_imu_= this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS(rclcpp::SensorDataQoS()), std::bind(&LocalisationControlNode::imu_callback, this, std::placeholders::_1));
    //publisher
    publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    publisher_slam_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    //###########NEU##############
    publisher_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    publisher_initial_pose_= this->create_publisher<geometry_msgs::msg::PoseStamped>("/initialpose", 10);
    //timer
    timer_ = this->create_wall_timer(1000ms, std::bind(&LocalisationControlNode::timer_callback, this));
    m_initial_pose_set = true;
    m_goal_send = true;
    m_wait = 0;
    }

void LocalisationControlNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_imu)
{
    float x_orient_imu = msg_imu->orientation.x;
    float y_orient_imu = msg_imu->orientation.y;
    float z_orient_imu = msg_imu->orientation.z;
    float w_orient_imu = msg_imu->orientation.w; 

    m_localisation->setPosOrientation(x_orient_imu, y_orient_imu, z_orient_imu, w_orient_imu);
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
    
    //m_localisation->setPosOrientation(x, y, x_orient, y_orient, z_orient, w_orient);

    std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
    obs_state_vector_x_y_yaw.data = {m_localisation->getX(), m_localisation->getY(), m_localisation->getYawZ()};

    //Input for function publish_estimated_state
    publish_estimated_state(obs_state_vector_x_y_yaw);
}

void LocalisationControlNode::timer_callback()
{
    //setting_goal_pose();
    

    auto message = geometry_msgs::msg::Twist();
    
    message.linear.x = m_control->getSpeed();
    message.angular.z = m_control->getAngle();
    
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


void LocalisationControlNode::scan_callback(sensor_msgs::msg::LaserScan msg_scan)
{
    auto msg_slam_scan = sensor_msgs::msg::LaserScan();
    msg_slam_scan.ranges = msg_scan.ranges;

    std::cout << "msg_slan_scan_0: " << msg_slam_scan.ranges[0] << std::endl;
    std::cout << "msg_slan_scan_90: " << msg_slam_scan.ranges[500] << std::endl;
    std::cout << "msg_slan_scan_180: " << msg_slam_scan.ranges[1500] << std::endl;
    std::cout << "msg_slan_scan_270: " << msg_slam_scan.ranges[1000] << std::endl;

    publisher_slam_scan_->publish(msg_slam_scan);

}

void LocalisationControlNode::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose)
{
    auto amcl_pose = msg_amcl_pose->pose;
    m_navigation->setamclX(amcl_pose.pose.position.x);
    //std::cout << "amclX:" << amcl_pose.pose.position.x << std::endl;
    m_navigation->setamclY(amcl_pose.pose.position.y);
    //std::cout << "amclY" << amcl_pose.pose.position.y << std::endl;
}



/*void LocalisationControlNode::setting_goal_pose(){

    //std::cout << "goal_pose_setting" << m_initial_pose_set << std::endl;
    
    if(m_initial_pose_set == true){
        
        auto initial_pose = geometry_msgs::msg::PoseStamped();
    
        //initial_pose.header.stamp = now();
        initial_pose.header.frame_id = "map";
    
        //initial position
        initial_pose.pose.position.x = 0;
        initial_pose.pose.position.y = 0;
        initial_pose.pose.position.z = 0;
    
        //initial orientation
        initial_pose.pose.orientation.x = 0;
        initial_pose.pose.orientation.y = 0;
        initial_pose.pose.orientation.z = 0;
        initial_pose.pose.orientation.w = 0; 

        publisher_initial_pose_->publish(initial_pose);

        m_initial_pose_set = false;

        
    } 

    

    //send goal_pose after 2sec
    m_wait = m_wait + 500;
    m_navigation->setTact(m_wait);

    //std::cout << "m_wait:" << m_wait << std::endl;

    if(m_wait == 2000){
        m_goal_send = true;
    }

    if(m_goal_send){
        auto goal_pose = geometry_msgs::msg::PoseStamped();
        
        //goal_pose.header.stamp = now();
        goal_pose.header.frame_id = "map";
        
        //initial position
        goal_pose.pose.position.x = m_navigation->getGoalPosX();
        goal_pose.pose.position.y = m_navigation->getGoalPosY();
        goal_pose.pose.position.z = m_navigation->getGoalPosZ();
        
        //initial orientation
        goal_pose.pose.orientation.x = m_navigation->getGoalOriX();
        goal_pose.pose.orientation.y = m_navigation->getGoalOriY();
        goal_pose.pose.orientation.z = m_navigation->getGoalOriZ();
        goal_pose.pose.orientation.w = m_navigation->getGoalOriW(); 

        //publisher_goal_pose_->publish(goal_pose);

        //std::cout << "goal pose setted:" << std::endl;
        //std::cout << "goalX:" << goal_pose.pose.position.x << std::endl;
        //std::cout << "goalY:" << goal_pose.pose.position.y << std::endl;
        m_goal_send = false;
        m_navigation->setGoalsended(m_goal_send);
        }

    m_goal_send = m_navigation->getPoseSend();
    
        
}*/