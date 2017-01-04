#ifndef PUBLISHER_HPP_
#define PUBLISHER_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class Publisher{
protected:
    ros::NodeHandle *nh;
    ros::Publisher pub;

public:
    std_msgs::Float64 msg;

public:
    Publisher(ros::NodeHandle *nh, std::string topic){
        this->nh = nh;
        this->pub = nh->advertise<std_msgs::Float64>(topic, 1000);
    }

    void Send(){
        this->pub.publish(this->msg);
    }

    void Send(double data){
        this->msg.data = data;
        this->pub.publish(msg);
    }

    void Send(std_msgs::Float64 msg){
        this->msg = msg;
        this->pub.publish(msg);
    }

    void QuickSend(std_msgs::Float64 msg){
        this->pub.publish(msg);
    }
};

#endif
