#include "localisationControlNode.h"

LocalisationControlNode::LocalisationControlNode(): Node("localisation_control")
{   
    //needs to wait for mapsserver to get initalized
    std::this_thread::sleep_for(2s);
    
    //client
    map_server_load_map = this->create_client<nav2_msgs::srv::LoadMap>(map_server_load_map_topic); // /map_server/load_map
    LoadMap *lp = new LoadMap(map_server_load_map);

    m_control = new Control();
    m_localisation = new Localisation(m_control);
    m_navigation = new Navigation(lp);
    
    //subscriber
    subscriber_odom_= this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControlNode::odom_callback, this, std::placeholders::_1));
    subscriber_laserscan_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/demo/laser/out", rclcpp::QoS(rclcpp::SensorDataQoS()), std::bind(&LocalisationControlNode::scan_callback, this, std::placeholders::_1));
    subscriber_amcl_pose_= this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>("/amcl_pose", 1, std::bind(&LocalisationControlNode::amcl_pose_callback, this, std::placeholders::_1));
    subscriber_imu_= this->create_subscription<sensor_msgs::msg::Imu>("/imu/data", rclcpp::QoS(rclcpp::SensorDataQoS()), std::bind(&LocalisationControlNode::imu_callback, this, std::placeholders::_1));
    
    //publisher
    publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
    publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    publisher_slam_scan_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
    publisher_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
    publisher_initial_pose_= this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
    publisher_clock_time_ = this->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 10);
    
    //timer
    timer_ = this->create_wall_timer(500ms, std::bind(&LocalisationControlNode::timer_callback, this));
    timer_clock_ = this->create_wall_timer(50ms, std::bind(&LocalisationControlNode::timer_clock_callback, this));
    m_initial_pose_set = false;
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
    
    //Orientation
    float x_orient = msg_odom->pose.pose.orientation.x;
    float y_orient = msg_odom->pose.pose.orientation.y;
    float z_orient = msg_odom->pose.pose.orientation.z;
    float w_orient = msg_odom->pose.pose.orientation.w;

    std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
    obs_state_vector_x_y_yaw.data = {m_localisation->getX(), m_localisation->getY(), m_localisation->getYawZ()};

    publish_estimated_state(obs_state_vector_x_y_yaw);
}

void LocalisationControlNode::timer_callback()
{
    setting_goal_pose();
    
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = m_control->getSpeed();
    message.angular.z = m_control->getAngle();
     
}

void LocalisationControlNode::timer_clock_callback()
{
    // publish sim_time on /clock topic
    auto time = this->now();
    auto message_time = rosgraph_msgs::msg::Clock();
    message_time.clock.sec = time.seconds();
    message_time.clock.nanosec = time.nanoseconds() % 1000000000ull;
    publisher_clock_time_->publish(message_time);   
}

void LocalisationControlNode::publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg)
{
    // publishing the estimated pose and orientation
    auto message_pose = std_msgs::msg::Float64MultiArray();
    message_pose.data = state_vec_msg.data;
    
    publisher_state_est_->publish(message_pose);
}

void LocalisationControlNode::scan_callback(sensor_msgs::msg::LaserScan msg_scan)
{
    auto msg_slam_scan = sensor_msgs::msg::LaserScan();
    msg_slam_scan.ranges = msg_scan.ranges;
    m_i_0 = 0;
    m_i_90 = 0;
    m_i_180 = 0;
    m_i_270 = 0;

    m_yawZ = m_localisation->getYawZ();
    m_yawZ_strich = m_yawZ + M_PI;
    m_n_laserrays = 2000;
    m_yawZ_rays = (m_yawZ_strich / (2*M_PI)) * m_n_laserrays;

    m_i_0 = 1000 - static_cast<int>(m_yawZ_rays);
    if (m_i_0 < 0){
        m_i_0 += 2000;
    }
    m_i_90 = 1500 - static_cast<int>(m_yawZ_rays);
    if (m_i_90 < 0){
        m_i_90 += 2000;
    }
    m_i_180 = 2000 - static_cast<int>(m_yawZ_rays);
    if (m_i_180 < 0){
        m_i_180 += 2000;
    }
    m_i_270 = 500 - static_cast<int>(m_yawZ_rays);
    if (m_i_270 < 0){
        m_i_270 += 2000;
    }

    m_localisation->setdist0(msg_slam_scan.ranges[m_i_0]);
    m_localisation->setdist90(msg_slam_scan.ranges[m_i_90]);
    m_localisation->setdist180(msg_slam_scan.ranges[m_i_180]);
    m_localisation->setdist270(msg_slam_scan.ranges[m_i_270]);

    m_localisation->determine_initialpose();

}

void LocalisationControlNode::amcl_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg_amcl_pose)
{
    auto amcl_pose = msg_amcl_pose->pose;
    m_navigation->setamclX(amcl_pose.pose.position.x);
    m_navigation->setamclY(amcl_pose.pose.position.y);
    m_localisation->setAmclX(amcl_pose.pose.position.x);
    m_localisation->setAmclY(amcl_pose.pose.position.y);
}

void LocalisationControlNode::setting_goal_pose(){

    m_area = m_localisation -> getMapArea();
    m_navigation -> setMapArea(m_area);
    m_pitch_rel = m_localisation -> getpitch_rel();
    m_navigation -> setPitchRel(m_pitch_rel);

    m_send_initial_pose = m_navigation -> getSendInitial();
    
    if(m_send_initial_pose == true){
        
        auto initial_pose = geometry_msgs::msg::PoseWithCovarianceStamped();
    
        initial_pose.header.stamp = this->now();
        initial_pose.header.frame_id = "map";
    
        //initial position
        initial_pose.pose.pose.position.x = m_localisation->getInitialposeX(); 
        initial_pose.pose.pose.position.y = m_localisation->getInitialposeY();
        initial_pose.pose.pose.position.z = m_localisation->getInitialposeZ();
    
        //initial orientation
        initial_pose.pose.pose.orientation.x = m_localisation->getInitialorientX();
        initial_pose.pose.pose.orientation.y = m_localisation->getInitialorientY();
        initial_pose.pose.pose.orientation.z = m_localisation->getInitialorientZ();
        initial_pose.pose.pose.orientation.w = m_localisation->getInitialorientW();

        publisher_initial_pose_->publish(initial_pose);

        m_send_initial_pose = false;
        m_initial_pose_set = true;
        
        m_navigation->setInitialsended(m_initial_pose_set);

    } 
    
    m_navigation->setTact(m_wait);
    m_wait = m_wait + 500;

    // waiting 2000ms to send goal
    if(m_wait >= 2000){
        m_send_goal = m_navigation->getSendGoal();
    }

    if(m_send_goal){
        auto goal_pose = geometry_msgs::msg::PoseStamped();
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

        publisher_goal_pose_->publish(goal_pose);

        m_send_goal = false;
        m_goal_sended = true;
        m_navigation->setGoalsended(m_goal_sended);

        }
         
}