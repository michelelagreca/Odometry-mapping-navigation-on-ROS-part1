#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"
#include <time.h>
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>

#include <sstream>
#include <cmath>
#include <string.h>

#include "first_project/reset_odom.h"

enum integration_methods {EULER, RUNGE_KUTTA, EXACT};

class odom{

private:
    double omega, v;
    double vx, vy;
    double d = 2.8;
    double R, alpha;

    double current_x, current_y, current_theta;

    double t_s;

    double frequency_rate;

    integration_methods integration_mode;

    ros::NodeHandle n;

    ros::Time current_time = ros::Time::now();

    ros::Subscriber sub_steer_speed;
    ros::Publisher pub_custom_msg;
    ros::Publisher pub_odom_msg;

    ros::ServiceServer service;

    nav_msgs::Odometry odom_msg;
    first_project::Odom cust_msg;

    tf::TransformBroadcaster br;

    tf::Transform transform;
    tf::Quaternion q;


public:
    odom(){
        n.getParam("starting_x", current_x);
        n.getParam("starting_y", current_y);
        n.getParam("starting_th", current_theta);

        n.getParam("frequency_rate", frequency_rate);

        integration_mode = EXACT;

        sub_steer_speed = n.subscribe("/speed_steer", 1000, &odom::callback_sub_data, this);
        pub_custom_msg = n.advertise<first_project::Odom>("/custom_odometry", 1000); 
        pub_odom_msg = n.advertise<nav_msgs::Odometry>("/odometry", 1000);

        service = n.advertiseService("reset_odom", &odom::reset_odometry, this);
    }

    void callback_sub_data(const geometry_msgs::Quaternion& msg){
        ros::Duration time_difference =  ros::Time::now() - current_time;
        current_time = ros::Time::now();
        
        if (time_difference.toSec() < frequency_rate) 
            t_s = time_difference.toSec(); //time of sampling
        else 
            t_s = 0;
            

        alpha = -msg.y;
        R = d / tan(alpha);
        v = msg.x;
        omega = v / R;

        integrations(t_s);   

        cust_msg.x = current_x;
        cust_msg.y = current_y;
        cust_msg.th = current_theta;
        cust_msg.timestamp = std::to_string(current_time.toSec());

        odom_msg.header.stamp = current_time;
        odom_msg.header.frame_id = "odom";

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_theta);

        odom_msg.pose.pose.position.x = current_x;
        odom_msg.pose.pose.position.y = current_y;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation = odom_quat;

        odom_msg.child_frame_id = "base_link";
        odom_msg.twist.twist.linear.x = vx;
        odom_msg.twist.twist.linear.y = vy;
        odom_msg.twist.twist.linear.z = omega;

        pub_custom_msg.publish(cust_msg);
        pub_odom_msg.publish(odom_msg);

        transform.setOrigin(tf::Vector3(current_x, current_y, 0.0));
        q.setRPY(0.0, 0.0, current_theta);
        transform.setRotation(q);

        br.sendTransform(tf::StampedTransform(transform, current_time, "odom", "base_link"));
    }


    void integrations(double t_s) {

        if (integration_mode == EULER) {
            current_x = current_x + v*t_s*cos(current_theta);
            current_y = current_y + v*t_s*sin(current_theta);
            vx = v*cos(current_theta);
            vy = v*sin(current_theta);
            current_theta = current_theta + omega*t_s;
        }else if (integration_mode == RUNGE_KUTTA) {
            current_x = current_x + v*t_s*cos(current_theta + (omega * t_s / 2));
            current_y = current_y + v*t_s*sin(current_theta + (omega * t_s / 2));
            vx = v*cos(current_theta);
            vy = v*sin(current_theta);
            current_theta = current_theta + omega*t_s;
        }else if (integration_mode == EXACT) {
            if (omega != 0){
                double theta_k_1 = current_theta + omega*t_s;
                current_x = current_x + (v/omega) * ( sin(theta_k_1) - sin(current_theta) );
                current_y = current_y - (v/omega) * ( cos(theta_k_1) - cos(current_theta) );
                vx = v*cos(current_theta);
                vy = v*sin(current_theta);
                current_theta = theta_k_1;
            }else {
                current_x = current_x + v*t_s*cos(current_theta + (omega * t_s / 2));
                current_y = current_y + v*t_s*sin(current_theta + (omega * t_s / 2));
                vx = v*cos(current_theta);
                vy = v*sin(current_theta);
                current_theta = current_theta + omega*t_s;
            }
        }
            
        
    }

    bool reset_odometry(first_project::reset_odom::Request &req,first_project::reset_odom::Response &res){
        current_x = 0.0;
        current_y = 0.0;
        current_theta = 0.0;
        ROS_INFO("Resetted odometry");
        res.resetted = true;
        return true;
    }


};

int main(int argc, char **argv){

    ros::init(argc, argv, "odom_node");

    odom my_odom;

    ros::spin();
    return 0;
}