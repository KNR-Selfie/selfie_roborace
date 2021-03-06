#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Float32.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"
#include "ackermann_msgs/AckermannDriveStamped.h"
#include <selfie_stm32_bridge/usb.hpp>
#include <selfie_stm32_bridge/bridge.h>
#include <sstream>
#include <selfie_stm32_bridge/StmBridgeConfig.h>
#include <dynamic_reconfigure/server.h>

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg);
void reconfigureCB(selfie_stm32_bridge::StmBridgeConfig& config, uint32_t level);

Pub_messages pub_messages;
Sub_messages sub_messages;

USB_STM Usb;
std_msgs::Empty empty_msg;

float ackermann_offset_front;
float ackermann_offset_back;
float front_axis_offset;
float back_axis_offset;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "selfie_stm32_bridge");
    ros::NodeHandle n;
    ros::NodeHandle pnh("~");
    dynamic_reconfigure::Server<selfie_stm32_bridge::StmBridgeConfig> dr_server_;
    dynamic_reconfigure::Server<selfie_stm32_bridge::StmBridgeConfig>::CallbackType dr_server_CB_;
    dr_server_CB_ = boost::bind(&reconfigureCB, _1, _2);
    dr_server_.setCallback(dr_server_CB_);
    pnh.getParam("ackermann_offset_front", ackermann_offset_front);
    pnh.getParam("ackermann_offset_back", ackermann_offset_back);
    pnh.getParam("front_axis_offset", front_axis_offset);
    pnh.getParam("back_axis_offset", back_axis_offset);

    ros::Publisher imu_publisher = n.advertise<sensor_msgs::Imu>("imu", 100);
    ros::Publisher velo_publisher = n.advertise<std_msgs::Float32>("speed", 50);
    ros::Publisher dis_publisher = n.advertise<std_msgs::Float32>("distance", 50);
    ros::Publisher button1_publisher = n.advertise<std_msgs::Empty>("start_button1", 50);
    ros::Publisher button2_publisher = n.advertise<std_msgs::Empty>("start_button2", 50);
    ros::Publisher switch_state_publisher = n.advertise<std_msgs::UInt8>("switch_state", 50);


    ros::Subscriber ackerman_subscriber = n.subscribe("drive", 1, ackermanCallback);
    ros::Subscriber left_turn_indicator_subscriber = n.subscribe("left_turn_indicator", 1, left_turn_indicatorCallback);
    ros::Subscriber right_turn_indicator_subscriber = n.subscribe("right_turn_indicator", 1, right_turn_indicatorCallback);

    Usb.init();
    Time time;
    ros::Rate sleep_rate(200);

    while (ros::ok())
    {
        //if new data read - publish
        if(Usb.read_from_STM())
            Usb.fill_publishers(pub_messages);

        //send subscribed data
        Usb.send_frame_to_STM(time.get_ms_time(), sub_messages);

        //publishing msg
        imu_publisher.publish(pub_messages.imu_msg);
        velo_publisher.publish(pub_messages.velo_msg);
        dis_publisher.publish(pub_messages.dist_msg);
        switch_state_publisher.publish(pub_messages.futaba_state);

        if(pub_messages.button_1)
            button1_publisher.publish(empty_msg);
        if(pub_messages.button_2)
            button2_publisher.publish(empty_msg);

        ros::spinOnce();
        sleep_rate.sleep();
    }
}

void ackermanCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
{   
    sub_messages.ackerman.steering_angle_front = -msg->drive.steering_angle + ackermann_offset_front;
    sub_messages.ackerman.steering_angle_back = msg->drive.steering_angle + ackermann_offset_back;
    sub_messages.ackerman.speed = msg->drive.speed;
    sub_messages.ackerman.acceleration = msg->drive.acceleration;
    sub_messages.ackerman.jerk = msg->drive.jerk;
}


void left_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
        Usb.send_cmd_to_STM(left_indicator_on);
    else
        Usb.send_cmd_to_STM(left_indicator_off);
}

void right_turn_indicatorCallback(const std_msgs::Bool::ConstPtr& msg)
{   
    if(msg->data)
        Usb.send_cmd_to_STM(rigth_indicator_on);
    else
        Usb.send_cmd_to_STM(rigth_indicator_off);
}
void reconfigureCB(selfie_stm32_bridge::StmBridgeConfig& config, uint32_t level)
{
    if(ackermann_offset_back != (float)config.ackermann_offset_back)
    {
        ackermann_offset_back = (float)config.ackermann_offset_back;
        ROS_INFO("ackermann_offset_back new value: %f", ackermann_offset_back);
    }
    if(ackermann_offset_front != (float)config.ackermann_offset_front)
    {
        ackermann_offset_front = (float)config.ackermann_offset_front;
        ROS_INFO("ackermann_offset_front new value: %f", ackermann_offset_front);
    }
    if(front_axis_offset != (float)config.front_axis_offset)
    {
        front_axis_offset = (float)config.front_axis_offset;
        ROS_INFO("front_axis_offset end new value %f", front_axis_offset);
    }
    if(back_axis_offset != (float)config.back_axis_offset)
    {
        back_axis_offset = (float)config.back_axis_offset;
        ROS_INFO("back_axis_offset end new value %f", back_axis_offset);
    }
    
}

