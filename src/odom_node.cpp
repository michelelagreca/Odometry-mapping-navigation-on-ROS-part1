#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "first_project/Odom.h"

class odom{
    nav_msgs::Odometry odom_message;



private:
    ros::NodeHandle n;

    ros::Subscriber sub;
    ros::Publisher pub;

public:
    odom(){
        sub = n.subscribe("/steer_speed", 1000, &odom::callback, this);
    }

    void callback(const geometry_msgs::Quaternion& msg){

    }
};

int main(int argc, char **argv){

    ros::init(argc, argv, "shuttle_odometry");
    odom my_odom;
    ros::spin();
    return 0;
}