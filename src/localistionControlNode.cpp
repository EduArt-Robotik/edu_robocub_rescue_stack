#include "localisationControlNode.h"


LocalisationControlNode::LocalisationControlNode(): Node("localisation_control")
{
    //needs to wai for mapsserver to get initalized
    std::this_thread::sleep_for(2s);

     //client
    amcl_get_state = this->create_client<lifecycle_msgs::srv::GetState>(amcl_get_state_topic);
    amcl_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(amcl_change_state_topic);

    map1_server_get_state = this->create_client<lifecycle_msgs::srv::GetState>(map1_server_get_state_topic);
    map1_server_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(map1_server_change_state_topic);

    map2_server_get_state = this->create_client<lifecycle_msgs::srv::GetState>(map2_server_get_state_topic);
    map2_server_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(map2_server_change_state_topic);

    map3_server_get_state = this->create_client<lifecycle_msgs::srv::GetState>(map3_server_get_state_topic);
    map3_server_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(map3_server_change_state_topic);

    map4_server_get_state = this->create_client<lifecycle_msgs::srv::GetState>(map4_server_get_state_topic);
    map4_server_change_state = this->create_client<lifecycle_msgs::srv::ChangeState>(map4_server_change_state_topic);

    map_server_load_map = this->create_client<nav2_msgs::srv::LoadMap>(map_server_load_map_topic);

    m_amclService = new ClientService(amcl_get_state, amcl_change_state, "amcl");
    m_map1ServerService = new ClientService(map1_server_get_state, map1_server_change_state, "map1_service");
    m_map2ServerService = new ClientService(map2_server_get_state, map2_server_change_state, "map2_service");
    m_map3ServerService = new ClientService(map3_server_get_state, map3_server_change_state, "map3_service");
    m_map4ServerService = new ClientService(map4_server_get_state, map4_server_change_state, "map4_service");
    LoadMap *lp = new LoadMap(map_server_load_map);

    m_control = new Control(m_map1ServerService, m_map2ServerService, m_map3ServerService, m_map4ServerService, lp);
    m_localisation_amcl = new Localisation();
    m_localisation_odom = new Localisation();
    m_localisation_imu = new Localisation();


    //subscriber
    subscriber_odom_= this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControlNode::odom_callback, this, std::placeholders::_1));
    subscriber_velocity_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&LocalisationControlNode::velocity_callback, this, std::placeholders::_1));
    subscriber_amcl_pose_= this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 1, std::bind(&LocalisationControlNode::amcl_pose_callback, this, std::placeholders::_1));
    subscriber_imu_= this->create_subscription<sensor_msgs::msg::Imu>("/imu_plugin/out", 1, std::bind(&LocalisationControlNode::imu_callback, this, std::placeholders::_1));

    //subscriber_state_est_=this->create_subscription<std_msgs::msg::Float64MultiArray>("/state_est", 1, std::bind(&LocalisationControlNode::state_est_callback, this, std::placeholders::_1));
    //subscriber_transition_amcl = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/amcl/transition_event", 1, std::bind(&LocalisationControlNode::transition_amcl_callback, this, std::placeholders::_1));
    //subscriber_transition_map_server = this->create_subscription<lifecycle_msgs::msg::TransitionEvent>("/map_server/transition_event", 1, std::bind(&LocalisationControlNode::transition_map_server_callback, this, std::placeholders::_1));

    //publisher
    publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
    //publisher for robot angle and speed
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    publisher_initialpose_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);

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
    
    m_localisation_odom->setPosOrientation(x, y, x_orient, y_orient, z_orient, w_orient);

    std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
    obs_state_vector_x_y_yaw.data = {m_localisation_odom->getX(), m_localisation_odom->getY(), m_localisation_odom->getYawZ()};

    //Input for function publish_estimated_state
    //publish_estimated_state(obs_state_vector_x_y_yaw);
}

void LocalisationControlNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg_imu){
    float x_orient = msg_imu->orientation.x;
    float y_orient = msg_imu->orientation.y;
    float z_orient = msg_imu->orientation.z;
    float w_orient = msg_imu->orientation.w;
    
    m_localisation_imu->setOrientation(x_orient, y_orient, z_orient, w_orient);
    m_control->setPitchY(m_localisation_imu->getPichtY());
    m_control->setRollX(m_localisation_imu->getRollX());
}


void LocalisationControlNode::timer_callback()
{
    if(m_control->m_activeMapServer->getState() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE && m_control->m_activeMapServer->getStateChange() == false
    &&m_control->m_activeMapServer->getStateGet() == false) {
        m_control->m_activeMapServer->activateService();
    }
    if(m_amcl_startet){
        //calculates angle and speed and pubish them
        m_control->calculateAngleSpeed();
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = m_control->getSpeed();
        message.angular.z = m_control->getAngle();
        publisher_vel_->publish(message);
    }
    else{
        //change to timer.sleep
        m_initpose_wait++;
        m_initpose_wait = m_initpose_wait % 100;
        if(m_initpose_wait == 99){
            auto initpose = geometry_msgs::msg::Pose();
            initpose.position.x = 0.0;
            initpose.position.y = 0.0;
            initpose.orientation.w = 0.000000001;
            publish_initalpose(initpose);
        }  
    }
     
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
    //std::cout <<"velocitiy callback" << msg_vel->linear.x <<std::endl;
    //output angular/linear velocity
    //v = msg_vel->linear.x;
    //yaw_rate = msg_vel->angular.z;
}

void LocalisationControlNode::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose){

    m_amcl_startet = true;

    /*std::cout << "Amcl: x:" <<msg_amcl_pose->pose.pose.position.x << ", y: " << msg_amcl_pose->pose.pose.position.y << 
    ", x Orient: " << msg_amcl_pose->pose.pose.orientation.x<< ", y Orient: " << msg_amcl_pose->pose.pose.orientation.y
    << ", z Orient: " << msg_amcl_pose->pose.pose.orientation.z<< ", w Orient: " << msg_amcl_pose->pose.pose.orientation.w<<std::endl;*/
    /*std::cout <<", odom: x " << m_localisation_odom->getX() <<", y: "<< m_localisation_odom->getY()<< ", x Orient: " << 
    m_localisation_odom->getXOrient()<<", y Orient: " << m_localisation_odom->getYOrient()
    << ", z Orient: " << m_localisation_odom->getXOrient()<< ", w Orient: " << m_localisation_odom->getXOrient()<<std::endl;*/
    
    //Position
    float x = msg_amcl_pose->pose.pose.position.x;
    float y = msg_amcl_pose->pose.pose.position.y;
    //float z_pos = msg_odom->pose.pose.position.z;
    
    //Orientation
    float x_orient = msg_amcl_pose->pose.pose.orientation.x;
    float y_orient = msg_amcl_pose->pose.pose.orientation.y;
    float z_orient = msg_amcl_pose->pose.pose.orientation.z;
    float w_orient = msg_amcl_pose->pose.pose.orientation.w;

    
    m_localisation_amcl->setPosOrientation(x, y, x_orient, y_orient, z_orient, w_orient);

    m_control->setPosYaw(x, y , m_localisation_amcl->getYawZ());

    if( (abs(x - m_localisation_odom->getX() ) > 0.2) ||
    (abs(y - m_localisation_odom->getY() ) > 0.2)  ||
    (abs(m_localisation_amcl->getYawZ() - m_localisation_odom->getYawZ() ) > 0.2) ) {
    std::cout <<"contorl " << m_control->getNavigationStep() << std::endl;

    std::cout <<"odom: x " << m_localisation_odom->getX() <<", y: "<< m_localisation_odom->getY()
        << ", YawZ: " << m_localisation_odom->getYawZ() <<", PitchY: " << m_localisation_odom->getPichtY() <<", RollX: " << m_localisation_odom->getRollX()<< std::endl;

    std::cout << "Amcl  & imu: x:" <<msg_amcl_pose->pose.pose.position.x << ", y: " << msg_amcl_pose->pose.pose.position.y << 
        ", YawZ: " << m_localisation_amcl->getYawZ() <<", PitchY: " << m_localisation_imu->getPichtY()<<", RollX: " << m_localisation_imu->getRollX()<<std::endl;
    }
}

void LocalisationControlNode::publish_initalpose(geometry_msgs::msg::Pose pose_msg)
{
    //publishing the pose
    auto pose_with_cov_msg = geometry_msgs::msg::PoseWithCovarianceStamped();
    pose_with_cov_msg.header.frame_id = "map";
    pose_with_cov_msg.pose.pose = pose_msg;
    
    publisher_initialpose_->publish(pose_with_cov_msg);
}
