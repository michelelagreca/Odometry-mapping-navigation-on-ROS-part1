#include "ros/ros.h"

#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"
#include <time.h>
#include "std_msgs/String.h"

#include <sstream>
#include <cmath>
#include <string.h>

class odom{

private:
    double omega, v;
    double new_x, new_y, new_theta;
    double d = 2.8;
    double R, alpha;

    double current_x, current_y, current_theta;

    double t_s;

    bool reset;

    ros::NodeHandle n;

    ros::Time current_time = ros::Time::now();
    ros::Time latest_sent_time = ros::Time::now();

    ros::Subscriber sub_steer_speed;
    ros::Publisher pub_custom_msg;

    ros::Timer timer;

    nav_msgs::Odometry odom_message;
    first_project::Odom cust_msg;


public:
    odom(){
        n.getParam("pose_x", current_x);
        n.getParam("pose_y", current_y);
        n.getParam("pose_theta", current_theta);

        sub_steer_speed = n.subscribe("/speed_steer", 1000, &odom::callback_sub_data, this);
        pub_custom_msg = n.advertise<first_project::Odom>("/custom_odometry", 1000); 

        timer = n.createTimer(ros::Duration(0.02), &odom::callback_publisher_timer, this); 
    }

    void callback_sub_data(const geometry_msgs::Quaternion& msg){
        ros::Duration time_difference =  ros::Time::now() - current_time;
        current_time = ros::Time::now();
        
        if (time_difference.toSec() < 0.05) {
            t_s = time_difference.toSec(); //time of sampling
            ROS_INFO_STREAM(t_s);
        }
        else
            t_s = 0;

        alpha = -msg.y;
        R = d / tan(alpha);
        v = msg.x;
        if (alpha != 0)
            omega = v * R;
        else
            omega = 0;
        /*
        ROS_INFO_STREAM("current alpha: " << std::to_string(alpha));
        ROS_INFO_STREAM("current R: " << std::to_string(R));
        ROS_INFO_STREAM("current v: " << std::to_string(v));
        ROS_INFO_STREAM("current omega: " << std::to_string(omega));
        */


        integrations(msg, t_s);   

        cust_msg.x = current_x;
        cust_msg.y = current_y;
        cust_msg.th = current_theta;
        cust_msg.timestamp = std::to_string(current_time.toSec());

    }

    void callback_publisher_timer(const ros::TimerEvent& ev) {
        
        if ((latest_sent_time - current_time).toSec() != 0) {
            latest_sent_time = current_time;

            pub_custom_msg.publish(cust_msg);
        }

        
    }

    void integrations(const geometry_msgs::Quaternion& msg, double t_s) {
        current_x = current_x + v*t_s*cos(current_theta);
        current_y = current_y + v*t_s*sin(current_theta);
        current_theta = current_theta + omega*t_s;
        
        /*
        ROS_INFO_STREAM("current ts: " << std::to_string(t_s));
        ROS_INFO_STREAM("current x: " << std::to_string(current_x));
        ROS_INFO_STREAM("current y: " << std::to_string(current_y));
        ROS_INFO_STREAM("current theta: " << std::to_string(current_theta));
        ROS_INFO_STREAM("current v: " << std::to_string(v));
        */
        
    }


};

int main(int argc, char **argv){

    ros::init(argc, argv, "shuttle_odometry");

    odom my_odom;

    ros::spin();
    return 0;
}