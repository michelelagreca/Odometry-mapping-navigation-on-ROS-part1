#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"
#include <time.h>
#include "std_msgs/String.h"

#include <sstream>

class odom{
    nav_msgs::Odometry odom_message;
    first_project::Odom cust_msg;


private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;

public:
    odom(){
        sub = n.subscribe("/steer_speed", 1000, &odom::callback, this);
        pub = n.advertise<first_project::Odom>("/custom_odometry", 1000);  
    }

    void callback(const geometry_msgs::Quaternion& msg){
        cust_msg.x = msg.x;
        cust_msg.y = msg.y;
        cust_msg.th = msg.w;
        cust_msg.timestamp = std::to_string(ros::Time::now().toSec());   
        pub.publish(cust_msg);
        
    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "shuttle_odometry");
    odom my_odom;
    ros::spin();
    return 0;
}