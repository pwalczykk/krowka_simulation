#ifndef PUBLISHER2_HPP_
#define PUBLISHER2_HPP_

#include <ros/ros.h>
#include <std_msgs/Float64.h>

class Publisher2{
protected:
    ros::NodeHandle *nh;
    ros::Publisher pub1;
    ros::Publisher pub2;

public:
    std_msgs::Float64 msg;

public:
    Publisher2(ros::NodeHandle *nh, std::string topic1, std::string topic2){
        this->nh = nh;
        this->pub1 = nh->advertise<std_msgs::Float64>(topic1, 1000);
        this->pub2 = nh->advertise<std_msgs::Float64>(topic2, 1000);

    }

    void Send(){
        this->pub1.publish(this->msg);
        this->pub2.publish(this->msg);

    }

    void Send(double data){
        this->msg.data = data;
        this->pub1.publish(msg);
        this->pub2.publish(msg);
    }

    void Send(std_msgs::Float64 msg){
        this->msg = msg;
        this->pub1.publish(msg);
        this->pub2.publish(msg);
    }

    void QuickSend(std_msgs::Float64 msg){
        this->pub1.publish(msg);
        this->pub2.publish(msg);
    }
};

#endif
