#include "InverseKinematics.hpp"
#include "TemplatePublisher.hpp"
#include "TemplateSubscriber.hpp"

#include <ros/ros.h>
#include <std_msgs/Float64.h>

int main(int argc, char** argv){

    ros::init(argc, argv, "krowka_inverse_kinematics");
    ros::NodeHandle nh;
    ros::Rate loop_rate(3);

    InverseKinematics ik;

    TemplateSubscriber <std_msgs::Float64> sub_a (&nh, "/krowka/inverse_kinematics/a");
    TemplateSubscriber <std_msgs::Float64> sub_x (&nh, "/krowka/inverse_kinematics/x");
    TemplateSubscriber <std_msgs::Float64> sub_y (&nh, "/krowka/inverse_kinematics/y");
    TemplateSubscriber <std_msgs::Float64> sub_z (&nh, "/krowka/inverse_kinematics/z");

    TemplatePublisher <std_msgs::Float64> pub_t0 (&nh, "/krowka/link_0_position_controller/command");
    TemplatePublisher <std_msgs::Float64> pub_t1 (&nh, "/krowka/link_1_position_controller/command");
    TemplatePublisher <std_msgs::Float64> pub_t2 (&nh, "/krowka/link_2_position_controller/command");
    TemplatePublisher <std_msgs::Float64> pub_t3 (&nh, "/krowka/link_3_position_controller/command");

    // ik.update(4.7, 0, 0, 600);
    //
    // ROS_INFO("%f\t %f\t %f\t %f", ik.theta0.data, ik.theta1.data, ik.theta2.data, ik.theta3.data);

    sub_a.msg.data = 4.7;
    sub_x.msg.data = 0;
    sub_y.msg.data = 0;
    sub_z.msg.data = 600;

    while(ros::ok()){
        ros::spinOnce();
        ik.update(sub_a.msg.data, sub_x.msg.data, sub_y.msg.data, sub_z.msg.data);
        ROS_INFO("Absolute angles: %f\t %f\t %f\t %f", ik.theta0.data, ik.theta1.data, ik.theta2.data, ik.theta3.data);

        pub_t0.msg.data = ik.theta0.data;
        pub_t1.msg.data = 1.57075 - ik.theta1.data;
        pub_t2.msg.data = 1.57075 - pub_t1.msg.data - ik.theta2.data;
        pub_t3.msg.data = - ik.theta3.data;

        ROS_INFO("Relative angles: %f\t %f\t %f\t %f", pub_t0.msg.data, pub_t1.msg.data, pub_t2.msg.data, pub_t3.msg.data);
        printf("\n");

        pub_t0.publish();
        pub_t1.publish();
        pub_t2.publish();
        pub_t3.publish();

        loop_rate.sleep();

    }

    return 0;
}
