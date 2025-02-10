#include "ros/ros.h"
#include "mavros_msgs/CommandLong.h"
#include <iostream>



bool disarm_cmd(void){
    bool disarm = false;
    mavros_msgs::CommandLong disarm_cmd;
    disarm_cmd.request.broadcast = false;
    disarm_cmd.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
    disarm_cmd.request.param1 = 0;     // 0 = Disarm (1 = Arm)
    disarm_cmd.request.param2 = 0;    // normal, do safety checks
    disarm_cmd.request.param3 = 0.0;
    disarm_cmd.request.param4 = 0.0;
    disarm_cmd.request.param5 = 0.0;
    disarm_cmd.request.param6 = 0.0;
    disarm_cmd.request.param7 = 0.0;
    ros::Rate rate(10.0);

    if( _mavros_state.armed ){
        if( _mavros_command_client.call(disarm_cmd) &&  disarm_cmd.response.success){

            while(_mavros_state.armed ){
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Vehicle disarmed");
            disarm = true;
        }
    }
    return disarm;
}

bool force_disarm_cmd(void){
    bool disarm = false;
    mavros_msgs::CommandLong disarm_cmd;
    disarm_cmd.request.broadcast = false;
    disarm_cmd.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
    disarm_cmd.request.param1 = 0;     // 0 = Disarm (1 = Arm)
    disarm_cmd.request.param2 = 21196; // Force disarm override code
    disarm_cmd.request.param3 = 0.0;
    disarm_cmd.request.param4 = 0.0;
    disarm_cmd.request.param5 = 0.0;
    disarm_cmd.request.param6 = 0.0;
    disarm_cmd.request.param7 = 0.0;
    ros::Rate rate(10.0);

    if( _mavros_state.armed ){
        if( _mavros_command_client.call(disarm_cmd) &&  disarm_cmd.response.success){

            while(_mavros_state.armed ){
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO("Vehicle disarmed");
            disarm = true;
        }
    }
    return disarm;
}



int main(int argc, char **argv) {
    ros::init(argc, argv, "force_disarm_client");
    ros::NodeHandle nh;

    // Create a service client to call /mavros/cmd/command
    ros::ServiceClient _mavros_command_client = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    //wait for user
    char c;
    std::cout<<"enter:\n 'd' char to send disarm\n 'f' char to send forced disarm\n";
    std::cin>>c;

    // Create the service request
    mavros_msgs::CommandLong srv;

    if(c=='f'){
        srv.request.broadcast = false;
        srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        srv.request.param1 = 0;     // 0 = Disarm (1 = Arm)
        srv.request.param2 = 21196; // Force disarm override code
        srv.request.param3 = 0.0;
        srv.request.param4 = 0.0;
        srv.request.param5 = 0.0;
        srv.request.param6 = 0.0;
        srv.request.param7 = 0.0;
    }
    else if(c == 'd'){
        srv.request.broadcast = false;
        srv.request.command = 400;  // MAV_CMD_COMPONENT_ARM_DISARM
        srv.request.param1 = 0;     // 0 = Disarm (1 = Arm)
        srv.request.param2 = 0; // normal, do safety checks
        srv.request.param3 = 0.0;
        srv.request.param4 = 0.0;
        srv.request.param5 = 0.0;
        srv.request.param6 = 0.0;
        srv.request.param7 = 0.0;
    }
    else{
        std::cout<<"wrong input, exiting...";
        return 0;
    }
    

    // Call the service
    if (_mavros_command_client.call(srv) && srv.response.success) {
        ROS_INFO("Drone force-disarmed successfully!");
    } else {
        ROS_ERROR("Failed to disarm the drone!");
    }

    return 0;
}