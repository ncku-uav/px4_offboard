#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <sensor_msgs/NavSatFix.h>

// global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_position;
bool global_position_received = false;

// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    global_position_received = true;
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);

    ros::Rate rate(10);

    // wait for fcu connection
    /*while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");*/

    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    // set target position
    mavros_msgs::GlobalPositionTarget goal_position;
    goal_position.latitude =47.3998376;
    goal_position.longitude = 8.5442719;
    goal_position.altitude = 77.2;

    // send a few setpoints before starting
    for (int i=0; i<20; ++i) {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        rate.sleep();
    }

    // set mode
    /*mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.base_mode = 0;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
        ROS_INFO("OFFBOARD enabled");
    } else {
        ROS_ERROR("Failed to set OFFBOARD");
    }*/

    // arm
    /*mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
        ROS_INFO("Vehicle armed");
    } else {
        ROS_ERROR("Arming failed");
    }*/

    // take off to 5m above ground
    //goal_position.altitude = goal_position.altitude + 5.0;
    while (ros::ok()) {
        goal_position.header.stamp = ros::Time::now();
        goal_pos_pub.publish(goal_position);
        ros::spinOnce();
        ROS_INFO_THROTTLE(1, "At altitude %.2f", global_position.altitude);
        rate.sleep();
    }

    return 0;
}