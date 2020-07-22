/***********************************
 * This program grabs the waypoints set on QGroundControl via mavros
 * and guide the plane flight throungh these waypoints with "off_board"
 * flight mode.
 * 
 * Auther SHEN CHANG TE
 * Date 20200719
 **********************************/

#include <ros/ros.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <mavros_msgs/WaypointList.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <vector>
#include <iostream>
#include "wgs_conversions/WgsConversion.h"

#define MAX_WP_ARRAY 20

// global variables
mavros_msgs::State current_state;
sensor_msgs::NavSatFix global_position;
static mavros_msgs::WaypointList WP_List; 
bool global_position_received = false;
bool qg_wp_received = false;
bool home_alt_set = false;
std_msgs::Float64 homeAlt;
using namespace std;
// callback functions
void globalPosition_cb(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    global_position = *msg;
    global_position_received = true;
    ROS_INFO_ONCE("Got global position: [%.2f, %.2f, %.2f]", msg->latitude, msg->longitude, msg->altitude);
}

void state_cb(const mavros_msgs::State::ConstPtr& msg) {
    current_state = *msg;
}

void qg_wp_cb(const mavros_msgs::WaypointList::ConstPtr& msg){
    WP_List = *msg;

}

void home_gps_alt_set(void){
    homeAlt.data = global_position.altitude;
    home_alt_set = true;
}

/*void wp_mem_init(void){
    WP_List = (mavros_msgs::WaypointList *)malloc(sizeof(mavros_msgs::WaypointList)*MAX_WP_ARRAY);
}*/

// main function
int main(int argc, char **argv) {
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::ServiceClient arming_client = nh.serviceClient < mavros_msgs::CommandBool > ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient < mavros_msgs::SetMode > ("mavros/set_mode");
    ros::Subscriber state_sub = nh.subscribe < mavros_msgs::State > ("mavros/state", 10, state_cb);
    ros::Subscriber global_pos_sub = nh.subscribe < sensor_msgs::NavSatFix > ("mavros/global_position/global", 1, globalPosition_cb);
    ros::Subscriber qg_wp_sub = nh.subscribe <mavros_msgs::WaypointList> ("mavros/mission/waypoints",30, qg_wp_cb);
    ros::Publisher goal_pos_pub = nh.advertise < mavros_msgs::GlobalPositionTarget > ("mavros/setpoint_position/global", 10);
    wgs_conversions::WgsConversion srv;
    ros::ServiceClient client = nh.serviceClient<wgs_conversions::WgsConversion>("lla2enu");
    ros::Rate rate(10);
    double lla[3],ref_lla[3],enu[3];
    homeAlt.data = 0.0;  // initialize home alt.

    //wp_mem_init();  // Initilize memeory block for WPs.

    // wait for fcu connection
    while (ros::ok() && !current_state.connected) {
        ROS_INFO_ONCE("Waiting for FCU connection...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("FCU connected");

    // wait for position information
    while (ros::ok() && !global_position_received) {
        ROS_INFO_ONCE("Waiting for GPS signal...");
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("GPS position received");

    if(!home_alt_set){
        home_gps_alt_set();
        printf("home alt is now set to %f", homeAlt.data);
    }

    // set target position
    vector<mavros_msgs::GlobalPositionTarget> goal_positions;
    mavros_msgs::GlobalPositionTarget goal_tmp;
    /*mavros_msgs::GlobalPositionTarget goal_position0, goal_position1, 
                                            goal_position2, goal_position3,goal_position4;
	goal_position0.latitude =47.3977661;
    goal_position0.longitude = 8.5473461;
    goal_position0.altitude = 77.2;
	goal_position1.latitude =47.3997040;
    goal_position1.longitude = 8.5477800;
    goal_position1.altitude = 77.2;
	goal_position2.latitude =47.3998222;
    goal_position2.longitude = 8.5446148;
    goal_position2.altitude = 77.2;
	goal_position3.latitude =47.3995972;
    goal_position3.longitude = 8.5405912;
    goal_position3.altitude = 77.2;
	goal_position4.latitude =47.3981514;
    goal_position4.longitude = 8.5407629;
    goal_position4.altitude = 77.2;
    goal_positions.push_back(goal_position0);
    goal_positions.push_back(goal_position1);
    goal_positions.push_back(goal_position2);   
    goal_positions.push_back(goal_position3); 
    goal_positions.push_back(goal_position4);*/
    int wp_load_idx = 0;
    while(WP_List.waypoints[wp_load_idx].frame!=NULL){
        if(WP_List.waypoints[wp_load_idx].command == 16 || WP_List.waypoints[wp_load_idx].command == 22 || WP_List.waypoints[wp_load_idx].command == 31){
            goal_tmp.latitude = WP_List.waypoints[wp_load_idx].x_lat;
            goal_tmp.longitude = WP_List.waypoints[wp_load_idx].y_long;
            goal_tmp.altitude = WP_List.waypoints[wp_load_idx].z_alt + homeAlt.data;
            printf("WP %d recieved! Lat: %f, Long: %f, Alt: %f\n",wp_load_idx, goal_tmp.latitude, goal_tmp.longitude, goal_tmp.altitude);
        }
        

        goal_positions.push_back(goal_tmp);
        wp_load_idx++;
    }


    // take off to 5m above ground
    int i = 0;
    while (ros::ok()) {
        if (i > sizeof(goal_positions))i=0;
        goal_positions[i].header.stamp = ros::Time::now();
        srv.request.ref_lla[0] = goal_positions[i].latitude;
        srv.request.ref_lla[1] = goal_positions[i].longitude;
        srv.request.ref_lla[2] = goal_positions[i].altitude;

        srv.request.lla[0] = global_position.latitude;
        srv.request.lla[1] = global_position.longitude;
        srv.request.lla[2] = global_position.altitude;
        if (client.call(srv)){
            enu[0] = srv.response.enu[0];
            enu[1] = srv.response.enu[1];
            enu[2] = srv.response.enu[2];
            if(enu[0]*enu[0]+enu[1]*enu[1]<10*10)
                i++;
        }
        goal_pos_pub.publish(goal_positions[i]);
        ros::spinOnce();
        //ROS_INFO_THROTTLE(1, "At altitude %.2f", global_position.altitude);
        ROS_INFO("WP %d, DIST %.2f, E %.2f N %.2f U %.2f", i, enu[0]*enu[0]+enu[1]*enu[1], enu[0], enu[1], enu[2]);
        rate.sleep();
    }

    return 0;
}
