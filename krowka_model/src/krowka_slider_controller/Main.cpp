#include <ros/ros.h>
#include "Publisher.hpp"
#include "Publisher2.hpp"
#include "Subscriber_DRC.hpp"

int main(int argc, char** argv){

    ros::init(argc, argv, "krowka_slider_controller");
    ros::NodeHandle nh;

    Subscriber_DRC krowka_slider_controller(&nh);

    ros::spin();
}
