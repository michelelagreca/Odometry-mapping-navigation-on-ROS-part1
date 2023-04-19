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
    double omega, vx, vy;
    double new_x, new_y, new_theta;
    double d = 2.8;
    double R;

    std::vector<double> moving_average_x;
    std::vector<double> moving_average_y;
    std::vector<double> moving_average_t;
    std::vector<double> moving_average_theta;

    double current_x, current_y, current_theta;

    bool reset;

    ros::Time current_time = ros::Time(0);
    ros::Time latest_sent_time = ros::Time(0);

    nav_msgs::Odometry odom_message;
    first_project::Odom cust_msg;

    ros::NodeHandle n;

    ros::Subscriber sub_steer_speed;
    ros::Publisher pub_custom_msg;

    ros::Timer timer;

public:
    odom(){
        n.getParam("pose_x", current_x);
        n.getParam("pose_y", current_y);
        n.getParam("pose_theta", current_theta);

        sub_steer_speed = n.subscribe("/steer_speed", 1000, &odom::callback, this);
        pub_custom_msg = n.advertise<first_project::Odom>("/custom_odometry", 1000); 

        timer = n.createTimer(ros::Duration(0.01), &odom::callback_publisher_timer, this); 
    }

    void callback(const geometry_msgs::Quaternion& msg){
        ros::Duration time_difference =  ros::Time() - current_time;
        current_time = ros::Time();
        double t_s = time_difference.toSec(); //time of sampling
        R = d / tan(msg.y);
        vx = msg.x;
        omega = vx * R;

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
        current_x = current_x + vx*t_s*cos(current_theta);
        current_y = current_y + vx*t_s*sin(current_theta);
        current_theta = current_theta + omega*t_s;
    }


};

int main(int argc, char **argv){

    ros::init(argc, argv, "shuttle_odometry");
    odom my_odom;
    ros::spin();
    return 0;
}