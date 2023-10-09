
#include "ros/duration.h"
#include "ros/ros.h"
#include "boost/thread.hpp"
#include "std_msgs/Int32.h"
#include "mavros_msgs/RCIn.h"
#include "sensor_msgs/Joy.h"
#include <iostream>
#include <sstream>

using namespace std;

class RC_command {

	public:
		RC_command();
        void run();
	
	private:
  
        void data_publisher();
        void user_interface();
        void land_seq();
        void takeoff_seq();
        void open_seq();
        void close_seq();


		ros::NodeHandle _nh;
        ros::Publisher _joy_pub;

        // ---parameters---
        int _N_axes;
        int _N_buttons;
        int _takeoff_land_idx;
        int _tol_value;
        int _open_close_idx;
        int _oc_value;
   
};

RC_command::RC_command() {

    _joy_pub = _nh.advertise<sensor_msgs::Joy>("/joy",10);

    _N_axes = 8;
    _N_buttons = 6;
    _takeoff_land_idx = 1;
    _tol_value = 1;
    _open_close_idx = 4;
    _oc_value = 0;

}

void RC_command::land_seq(){
    _tol_value = 1;
    ros::Duration(0.6).sleep();
    _tol_value = 0;

}

void RC_command::takeoff_seq(){
    _tol_value = 1;
    ros::Duration(0.6).sleep();
    _tol_value = 2;
}

void RC_command::open_seq(){
    _oc_value = 0;
    ros::Duration(0.6).sleep();
    _oc_value = 1;
    ros::Duration(0.6).sleep();
    _oc_value = 2;

}

void RC_command::close_seq(){
    _oc_value = 5;
    ros::Duration(0.6).sleep();
    _oc_value = 4;
    ros::Duration(0.6).sleep();
    _oc_value = 3;
}




void RC_command::data_publisher(){
    ros::Rate rate(10.0);

    sensor_msgs::Joy joy_msg;
    joy_msg.header.frame_id = "/px4_radio";
    joy_msg.axes.resize(_N_axes);
    joy_msg.buttons.resize(_N_buttons);
   

    for(int i = 0;i< _N_axes;i++){
        joy_msg.axes[i] = 0.0;
    }
    for(int i = 0;i< _N_buttons;i++){
        joy_msg.buttons[i] = 0;
    }

    bool changed = false;
    int old_tol =_tol_value;
    int old_oc = _oc_value;

    while(ros::ok()){
        changed = (_tol_value != old_tol) || (_oc_value != old_oc);
        old_tol = _tol_value;
        old_oc = _oc_value;

        if(changed){
            joy_msg.header.stamp = ros::Time::now();
            joy_msg.buttons[_takeoff_land_idx] = _tol_value;
            joy_msg.buttons[_open_close_idx] = _oc_value;
            _joy_pub.publish(joy_msg);
        }
        

        rate.sleep();
    }

}

void RC_command::user_interface(){
    bool running = true;
    char cmd_ch;

    while(ros::ok() && running){
        cout<<"Koala fake RC: \n l - land command\n t - takeoff command\n o - open crawler\n c - close crawler\n x - close\n\n";
        cin>>cmd_ch;
        switch(cmd_ch) {
            case 't' : 
                cout << "takeoff cmd...\n"; 
                takeoff_seq();
                break;    

            case 'l' :
                cout << "landing cmd...\n"; 
                land_seq();
                break;    
            
            case 'o' : 
                cout << "open crawler cmd...\n"; 
                open_seq();
                break;    

            case 'c' :
                cout << "close crawler cmd...\n"; 
                close_seq();
                break;  

            case 'x' :  
                running = 0;
                exit(0);
                break; 
        
            default :
                break;
        }
    }

}

void RC_command::run(){
    ros::Rate rate(1.0);
    ROS_INFO("init fake RC");

    boost::thread sp_pub_t( &RC_command::data_publisher, this);
    boost::thread usr_iface_t( &RC_command::user_interface, this);

}

int main( int argc, char** argv ) {
	ros::init(argc, argv, "RC_command");
	RC_command rc;
	
    rc.run();
	ros::spin(); 
	//----This function will be never overcome

	return 0;
}

