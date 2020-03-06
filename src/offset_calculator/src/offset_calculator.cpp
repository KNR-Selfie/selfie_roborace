#include <iostream>
#include <cmath>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float64.h>
#define MSG_QUEUE_SIZE 10
#define APPROXIMATE_BY_PARABOLA false

class OffsetCalc
{
  //ros::NodeHandle nh;
  ros::Subscriber path_sub;
  ros::Publisher linear_offset_pub;
  ros::Publisher angular_offset_pub;
  tf::TransformListener tf_listener;
  tf::Vector3 coefficients;
  double path_yaw;
  double angular_offset;
  double linear_offset;
  double x1, y1, x2, y2; //dwa pierwsze punkty ścieżki

public:

    ros::NodeHandle nh;

    OffsetCalc()
    {
      //nh.getParam("~path_approximation_by_parabola", path_approximation_by_parabola);
      linear_offset_pub = nh.advertise< std_msgs::Float64 >("position_offset", MSG_QUEUE_SIZE);
      angular_offset_pub = nh.advertise< std_msgs::Float64 >("heading_offset", MSG_QUEUE_SIZE);
      path_sub = nh.subscribe( "/closest_path_points", MSG_QUEUE_SIZE, &OffsetCalc::pathCallBack, this);
    }

    //calculates the coefficients of approximation function
    void pathCallBack(const nav_msgs::Path::ConstPtr& path)
    {
       int n_of_points = path->poses.size();

       if(false)
       {
         //TODO
       }
       else
       {
         //bierzemy pierwsze dwa punkty i liczymy równanie prostej przechodzącej
         //przez oba z nich używając wektora normalnego do prostej
         // równanie postaci ax + by + c = 0
         x1 = path->poses[0].pose.position.x;
         y1 = path->poses[0].pose.position.y;
         x2 = path->poses[1].pose.position.x;
         y2 = path->poses[1].pose.position.y;
         //ROS_INFO("%lf %lf %lf %lf", x1, y1, x2, y2);
         //Prosta w okreslona rownaniem ax + by + c = 0
         //a
         coefficients[0] = y1 - y2;
         //b
         coefficients[1] = x2 - x1;
         //c - podstawiamy dowolny punkt, uzyskujemy wyraz wolny i przenosimy go
         //na lewą stronę z minusem
         coefficients[2] = -coefficients[0]*x1 - coefficients[1]*y1;\
         //nachylenie sciezki do osi x
         path_yaw = std::atan2((y2 - y1),(x2 - x1));
       }
    }

    double publishOffsets()
    {
      tf::StampedTransform transform;
      if(APPROXIMATE_BY_PARABOLA)
      {
        //TODO
      }
      else
      {
        try {
          tf_listener.waitForTransform("/map", "/base_link", ros::Time(0), ros::Duration(10.0) );
          tf_listener.lookupTransform("/map", "/base_link", ros::Time(0), transform);
        }
        catch(tf::TransformException ex){
          ROS_ERROR("%s\n", ex.what());
          ros::Duration(1.0).sleep();
        }

        double base_link_x, base_link_y;
        base_link_x = transform.getOrigin().x();
        base_link_y = transform.getOrigin().y();
        tf::Quaternion base_link_q = transform.getRotation();
        tfScalar yaw, pitch, roll;
        tf::Matrix3x3 rotation_mat(base_link_q);
        rotation_mat.getRPY(roll, pitch, yaw, 1);
        // na prawo dodatni, na lewo ujemny
        angular_offset = path_yaw - yaw;
        //ROS_INFO("%lf %lf %lf\n", base_link_x, base_link_y, yaw);

        double cross_product = (base_link_x - x1) * (y2 - y1) -  (x2 - x1) * (base_link_y - y1);
        ROS_INFO("%lf", cross_product);
        int sign;
        if(cross_product < 0) sign = -1;
        else sign = 1;

        linear_offset = sign * std::abs(coefficients[0]*base_link_x + coefficients[1]*base_link_y +
                         coefficients[2])/std::sqrt(std::pow(coefficients[0],2) + std::pow(coefficients[1],2));
      }
      //ROS_INFO("coefs: %lf %lf %lf", coefficients[0], coefficients[1], coefficients[2]);
      //ROS_INFO("%lf %lf\n", linear_offset, angular_offset);
      std_msgs::Float64 msg;
      msg.data = (-1)*linear_offset;
      linear_offset_pub.publish(msg);
      msg.data = (-1)*angular_offset;
      angular_offset_pub.publish(msg);
    }
  };

  int main(int argc, char** argv)
  {
    ros::init(argc, argv, "offset_calculator");
    OffsetCalc calc;
    ros::Rate rate(10);
    while(calc.nh.ok())
    {
      calc.publishOffsets();
      rate.sleep();
      ros::spinOnce();
    }

    return 0;
  }
