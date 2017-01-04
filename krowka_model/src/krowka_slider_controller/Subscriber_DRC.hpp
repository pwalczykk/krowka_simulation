#ifndef SUBSCRIBER_DRC_HPP_
#define SUBSCRIBER_DRC_HPP_


#include <ros/ros.h>
#include <dynamic_reconfigure/server.h>
#include <krowka_model/DRC_KrowkaConfig.h>

#include "Publisher.hpp"
#include "Publisher2.hpp"

class Subscriber_DRC{

    ros::NodeHandle *nh;

    std::vector<Publisher> pub;
    std::vector<Publisher2> pub2;

    dynamic_reconfigure::Server<krowka_model::DRC_KrowkaConfig> server;
    dynamic_reconfigure::Server<krowka_model::DRC_KrowkaConfig>::CallbackType f;

public:
    Subscriber_DRC(ros::NodeHandle *nh){
        this->nh = nh;

        pub.push_back(Publisher(nh, "/krowka/link_0_position_controller/command"));
        pub.push_back(Publisher(nh, "/krowka/link_1_position_controller/command"));
        pub.push_back(Publisher(nh, "/krowka/link_2_position_controller/command"));
        pub.push_back(Publisher(nh, "/krowka/link_3_position_controller/command"));
        pub.push_back(Publisher(nh, "/krowka/grip_0_position_controller/command"));
        pub.push_back(Publisher(nh, "/krowka/grip_1_position_controller/command"));

        pub2.push_back(Publisher2(nh, "/krowka/grip_0_position_controller/command", "/krowka/grip_1_position_controller/command"));

        f = boost::bind(&Subscriber_DRC::DRC_Callback, this,_1, _2);
        server.setCallback(f);
    };

    void DRC_Callback(krowka_model::DRC_KrowkaConfig &config, uint32_t level){
        if(pub[0].msg.data != config.link_0)    pub[0].Send(config.link_0);
        if(pub[1].msg.data != config.link_1)    pub[1].Send(config.link_1);
        if(pub[2].msg.data != config.link_2)    pub[2].Send(config.link_2);
        if(pub[3].msg.data != config.link_3)    pub[3].Send(config.link_3);
        if(pub[4].msg.data != config.grip_0)    pub[4].Send(config.grip_0);
        if(pub[5].msg.data != config.grip_1)    pub[5].Send(config.grip_1);

        if(pub2[0].msg.data != config.gripper)    pub2[0].Send(config.gripper);
    }
};

#endif
