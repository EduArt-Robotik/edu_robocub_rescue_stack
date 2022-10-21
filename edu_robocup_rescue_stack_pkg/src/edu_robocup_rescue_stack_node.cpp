#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "std_msgs/msg/float64_multi_array.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono> 
#include <cmath>
#include <vector>
#include <iostream> 

#define _USE_MATH_DEFINES

//Initialisierung Variablen
_Float32 t0;
_Float32 t1;
_Float32 t2;
_Float32 t3;
_Float32 t4;
_Float32 roll_x;
_Float32 pitch_y;
_Float32 yaw_z;
_Float32 v;
_Float32 yaw_rate;
std_msgs::msg::Float64MultiArray obs_state_vector_x_y_yaw;
_Float32 x;
_Float32 y;
rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_vel_;
_Float32 x_dest;
_Float32 y_dest;

_Float32 delta_x;
_Float32 delta_y;
_Float32 delta_dist;
_Float32 delta_phi;


using namespace std::chrono_literals; 
using namespace std;
using std::placeholders::_1;


class LocalisationControl : public rclcpp::Node 
{

//Initialisierung Subscriber
rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscriber_odom_;
rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_velocity_;
rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscriber_state_est_;

//Initialisierung Publisher
rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_state_est_;

//Initialisierung Timer
rclcpp::TimerBase::SharedPtr timer_;

public:
    LocalisationControl()
    : Node("localisation_control")
    {
        
        //subscriber
        subscriber_odom_=this->create_subscription<nav_msgs::msg::Odometry>("/odom", 1, std::bind(&LocalisationControl::odom_callback, this, _1));
        subscriber_velocity_=this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 1, std::bind(&LocalisationControl::velocity_callback, this, _1));
        subscriber_state_est_=this->create_subscription<std_msgs::msg::Float64MultiArray>("/state_est", 1, std::bind(&LocalisationControl::state_est_callback, this, _1));
        
        //publisher
        publisher_state_est_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/state_est", 10);
        publisher_vel_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        //timer
        timer_ = this->create_wall_timer(50ms, std::bind(&LocalisationControl::timer_callback, this));
    }


private:

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg_odom) 
    {   
        //Position
        _Float32 x_pos = msg_odom->pose.pose.position.x;
        _Float32 y_pos = msg_odom->pose.pose.position.y;
        _Float32 z_pos = msg_odom->pose.pose.position.z;
        
        //Orientation
        _Float32 x_orient = msg_odom->pose.pose.orientation.x;
        _Float32 y_orient = msg_odom->pose.pose.orientation.y;
        _Float32 z_orient = msg_odom->pose.pose.orientation.z;
        _Float32 w_orient = msg_odom->pose.pose.orientation.w;
        
        //Ausgabe
        //std::cout << "x_pos: " << x_pos << std::endl;
        //std::cout << "y_pos: " << y_pos << std::endl;
        //std::cout << "z: " << z_pos << std::endl;

        //convert quaternion in euler angle
        t0 = +2.0 * (w_orient * x_orient + y_orient * z_orient);  
        t1 = +1.0 - 2.0 * (x_orient * x_orient + y_orient * y_orient);
        roll_x = atan2(t0, t1);
 
        t2 = +2.0 * (w_orient * y_orient - z_orient * x_orient);
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
        pitch_y = asin(t2);
 
        t3 = +2.0 * (w_orient * z_orient + x_orient * y_orient);
        t4 = +1.0 - 2.0 * (y_orient * y_orient + z_orient * z_orient);
        yaw_z = atan2(t3, t4);
        
        //Ausgabe
        std::cout << "yaw_z:" << yaw_z << std::endl;

        obs_state_vector_x_y_yaw.data = {x_pos, y_pos, yaw_z};

        //Input for function publish_estimated_state
        publish_estimated_state(obs_state_vector_x_y_yaw);
    }


    void timer_callback()
    {
        //destination pose
        x_dest = 10.0;
        y_dest = 1.0;

        delta_x = x_dest - x;
        delta_y = y_dest - y;
        
        //Ausgabe
        //std::cout << "x:" << x << std::endl;
        //std::cout << "y:" << y << std::endl;
        //std::cout << "delta_x:" << delta_x << std::endl;
        //std::cout << "delta_y:" << delta_y << std::endl;
        
        delta_dist = sqrt((delta_x*delta_x) + (delta_y*delta_y));
        
        //Ausgabe
        std::cout << "delta_dist:" << delta_dist << std::endl;
        
        delta_phi = (atan2(delta_y, delta_x))- yaw_z; 
        
        // Begrenzung des Drehwinkels Phi
        if(delta_phi < -M_PI){
            delta_phi += 2*M_PI;
            }
        if(delta_phi > M_PI){
            delta_phi -= 2*M_PI;
            };

        std::cout << "delta_phi:" << delta_phi << std::endl;
        auto message = geometry_msgs::msg::Twist();
        
        message.angular.z = delta_phi;
     
        if(delta_phi > 0.005 ) {
            message.linear.x = 0.0;
        //} else if((delta_dist > 2) && (delta_phi < 0.01) ){
        //    message.linear.x = delta_dist;
            //message.angular.z = 0.0;
        } else if((delta_phi < 0.005) ){
            message.linear.x = (delta_dist) / 5.0;
            //message.angular.z = 0.0;
        } else if ((delta_dist < 0.1)) {
            message.linear.x = 0.0;
            message.angular.z = 0.0;
        }
        publisher_vel_->publish(message);

    };

    void state_est_callback(std_msgs::msg::Float64MultiArray::SharedPtr state_vec_msg) 
    {
        //estimated pose
        x = state_vec_msg->data[0];
        y = state_vec_msg->data[1];
        yaw_z = state_vec_msg->data[2];
    }

    void publish_estimated_state(std_msgs::msg::Float64MultiArray state_vec_msg)
    {
        //publishing the estimated pose and orientation
        auto message_pose = std_msgs::msg::Float64MultiArray();
        message_pose.data = state_vec_msg.data;
        //message.data includes x-,y-coordinate and orientation around the z-axis yaw_z
        //pitch_y and role_x are calculated and can be added
        publisher_state_est_->publish(message_pose);
    };

    void velocity_callback(const geometry_msgs::msg::Twist::SharedPtr msg_vel) 
    {
        //output angular/linear velocity
        v = msg_vel->linear.x;
        yaw_rate = msg_vel->angular.z;
    };
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<LocalisationControl>());
    rclcpp::shutdown();
    return 0;
}
