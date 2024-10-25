//@Copyright : Dongbin Kim.
//Email : dongbin.kim@unlv.edu
//Date : 2020-05-21
//Title : Unity Input test to ROS for drone in Simulation In the Loop (SITL)
// Arming, Disarming, Takeoff, Land.

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <iostream>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

double modeinput;
void mode_input(const geometry_msgs::Point mode)
{
    modeinput = mode.x;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "hivemine_node");
    ros::NodeHandle nh;
    ros::NodeHandle n;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_cb);

    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    ros::Subscriber mode_sub = n.subscribe<geometry_msgs::Point>("/HM24/SetMode", 10, mode_input);
    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(5.0);

    mavros_msgs::SetMode uav_set_mode;
//    uav_set_mode.request.custom_mode = "STABILIZED";
//    set_mode_client.call(uav_set_mode);

    mavros_msgs::CommandBool arm_cmd;

    ros::Time last_request = ros::Time::now();

    while (ros::ok())
    {
        if (modeinput == 1)
        {
            arm_cmd.request.value = true;
            arming_client.call(arm_cmd);
            ROS_INFO("Vehicle Armed");
            modeinput = 0;
        }
        else if (modeinput == 2)
        {
            uav_set_mode.request.custom_mode = "OFFBOARD";
            set_mode_client.call(uav_set_mode);
            ROS_INFO("Vehicle Offboard Mode");
            modeinput = 0;
        }
        else if (modeinput == 3)
        {
            uav_set_mode.request.custom_mode = "STABILIZED";
            set_mode_client.call(uav_set_mode);

            uav_set_mode.request.custom_mode = "AUTO.LAND";
            set_mode_client.call(uav_set_mode);
            ROS_INFO("Vehicle Landing");
            modeinput = 0;
        }
        else if (modeinput == 4)
        {
            arm_cmd.request.value = false;
            arming_client.call(arm_cmd);
            ROS_INFO("Vehicle Dis-armed");
            modeinput = 0;
        }

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
