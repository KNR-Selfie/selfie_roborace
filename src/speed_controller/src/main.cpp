
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <std_msgs/Float64.h>
#include <math.h>
#include <boost/bind/bind.hpp>
#define MSG_QUEUE_SIZE 10
using namespace std;
class SpeedController {
public:
    SpeedController()
    {
        pub_ = n.advertise<std_msgs::Float64>("target_speed", MSG_QUEUE_SIZE);
        lin_sub = n.subscribe("position_offset", MSG_QUEUE_SIZE, &SpeedController::callback, this);
        angu_sub = n.subscribe("heading_offset", MSG_QUEUE_SIZE, &SpeedController::callback1, this);
    }
    ros::NodeHandle n;
    ros::Publisher pub_;
    ros::Subscriber lin_sub;
    ros::Subscriber angu_sub;

    void callback1(const std_msgs::Float64& an)
    {
        //cout<<"an: "<<an.data<<endl;
        this->angu.data = an.data;
    }

    void callback(const std_msgs::Float64& lin)
    {
        std_msgs::Float64 output;
        if (n.getParam("speed", speed) && n.getParam("distance", dist) && n.getParam("angle", angle) && n.getParam("max_speed", max_speed)) {
            output.data = speed / (dist * pow(lin.data, 2) + angle * pow(angu.data, 2));
            if (output.data > max_speed)
                output.data = max_speed;
        }
        else {
            output.data = 1 / (pow(lin.data, 2) + pow(angu.data, 2));
        }
        ros::Rate rate(10);
        pub_.publish(output);
        rate.sleep();
    }
    std_msgs::Float64 angu;
    double speed = 0;
    double dist = 0;
    double angle = 0;
    double max_speed = 0;
};
int main(int argc, char** argv)
{

    ros::init(argc, argv, "speed_controller");
    SpeedController a;
    ros::spin();
    return 0;
}
