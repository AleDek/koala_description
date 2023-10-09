#include <iostream>
#include <sstream>
#include <chrono>
#include <thread>
#include <boost/thread.hpp>
#include "ros/ros.h"
#include "ros/service_client.h"
#include "ros/subscriber.h"
#include "std_msgs/Int32.h"
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

#include <actionlib/server/simple_action_server.h>
#include "koala_msgs/stepperAction.h"
#include "koala_msgs/pipe_switch_state.h"

#include "gazebo_object_attacher/Attach.h"
#include "gazebo_object_attacher/AttachRequest.h"
#include "gazebo_object_attacher/AttachResponse.h"
#include "gazebo_msgs/ContactsState.h"


#define HOMING_TIMEOUT  4*60

#define STATE_CLOSED 0
#define STATE_OPEN 1
#define STATE_CLOSING 2
#define STATE_OPENING 3

class STEPPER_ACTION_SERVER  {

	private:
    
  	ros::NodeHandle _nh;
  	// NodeHandle instance must be created before this line. Otherwise strange error may occur.
  	actionlib::SimpleActionServer<koala_msgs::stepperAction> as; 
    tf2_ros::StaticTransformBroadcaster _static_broadcaster;
    ros::Publisher _pipe_state_pub;
    ros::Subscriber _gz_fwd_bump_sub;
    ros::Subscriber _gz_bwd_bump_sub;
    ros::ServiceClient _gz_attach_client;
    ros::ServiceClient _gz_detach_client;

  	// create messages that are used to published feedback/result
    koala_msgs::stepperFeedback _feedback;
    koala_msgs::stepperResult _result;

    gazebo_object_attacher::Attach _attach_srv;


    std::string _robot_name_gz;
    double _sec_clamp;
    double _sec_crawler;
    double _sec_lock;
    bool _cam_is_down;
    std::vector<double> _depth_tf_down;
    std::vector<double> _depth_tf_up;
    bool first_state;

  	std::string _action_name;
    int _clamp_state;
    int _crawler_State;
    int _lock_State;
    int _upsidedown_state;
    int _fwd_pipe_state;
    int _bwd_pipe_state;

    void load_param(){
    int ret =1;
    if( !_nh.getParam("/simu_stepper_server/robot_name_gz", _robot_name_gz)) { 
        _robot_name_gz = "koala";  //array idx
        ROS_ERROR("not loaded robot name gz !");
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/clamp_sec_delay", _sec_clamp)) { 
        _sec_clamp = 4.0;  //array idx
        
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/crawler_sec_delay", _sec_crawler)) { 
        _sec_crawler = 5.0;  //array idx
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/lock_sec_delay", _sec_lock)) { 
        _sec_lock = 2.0;  //array idx
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/cam_is_down", _cam_is_down)) { // default camera downfacing
        _cam_is_down = true;  //array idx
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/depth_tf_down", _depth_tf_down) ){
        _depth_tf_down = {0.110, 0.000, -0.040, 0.000, 1.222, 0.000};
        ROS_ERROR("not loaded tf down ");
        ret =0;
    }
    if( !_nh.getParam("/simu_stepper_server/depth_tf_up", _depth_tf_up) ){
        _depth_tf_up = {0.110, 0.000, 0.040, 3.141, -1.918, 0.000};
        ROS_ERROR("not loaded tf up");
        ret =0;
    }

    if(ret) ROS_INFO("Successfully loaded params.");
    else{
        ROS_ERROR("[STEPPER SERVER] Error in parameter loading. Assigned default value");
    }
    }

    void fwd_bump_cb( const gazebo_msgs::ContactsStateConstPtr msg){
        if(msg->states.size() > 0) _fwd_pipe_state = 1;
        else _fwd_pipe_state = 0;
    }

    void bwd_bump_cb( const gazebo_msgs::ContactsStateConstPtr msg){
        if(msg->states.size() > 0) _bwd_pipe_state = 1;
        else _bwd_pipe_state = 0;
    }

	public:
		STEPPER_ACTION_SERVER(std::string name) : as(_nh, name, boost::bind(&STEPPER_ACTION_SERVER::executeCB, this, _1), false), _action_name(name) {
            as.registerPreemptCallback(boost::bind(&STEPPER_ACTION_SERVER::preemptCB, this));
            load_param();
            _pipe_state_pub = _nh.advertise< koala_msgs::pipe_switch_state>("/stepper_driver/clamping_state",1 );
            _gz_fwd_bump_sub = _nh.subscribe("/fwd_bumper_vals", 1, &STEPPER_ACTION_SERVER::fwd_bump_cb, this);
            _gz_bwd_bump_sub = _nh.subscribe("/bwd_bumper_vals", 1, &STEPPER_ACTION_SERVER::bwd_bump_cb, this);
            _gz_attach_client = _nh.serviceClient<gazebo_object_attacher::Attach>("/attach_object");
            _gz_detach_client = _nh.serviceClient<gazebo_object_attacher::Attach>("/detach_object");
            
            //gazebo_object_attacher/Attach
            _attach_srv.request.robot_name = _robot_name_gz;
            _attach_srv.request.robot_link_name = "base_link";
            _attach_srv.request.object_name = "anchor";

            _clamp_state = STATE_OPEN ;
            _crawler_State = STATE_OPEN;
            _lock_State = STATE_OPEN;
            _fwd_pipe_state = 0;    // 1 pressed
            _bwd_pipe_state = 0; 

            if(_cam_is_down) _upsidedown_state =0;
            else _upsidedown_state = 1;

            first_state = true;

        
            boost::thread state_pub_t( &STEPPER_ACTION_SERVER::publish_state_task, this);  //run state publisher task

            while(!first_state){
                ROS_WARN("waiting first state...");
                std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            }

            ROS_INFO("upside state: %d",_upsidedown_state);
            if(_upsidedown_state == 1){
                send_tf_up();
                ROS_INFO("Detected clamping system upside");
            }
            else{
                send_tf_down();
                ROS_INFO("Detected clamping system downside"); // default, dawnfacing camera
            }
  
            as.start();
  		}

        void send_tf_up(){
            geometry_msgs::TransformStamped static_transformStamped;
            tf2::Quaternion quat;

            static_transformStamped.header.stamp = ros::Time::now();
            static_transformStamped.header.frame_id = "base_link";
            static_transformStamped.child_frame_id = "pipecam_link";
            static_transformStamped.transform.translation.x = _depth_tf_up[0];
            static_transformStamped.transform.translation.y = _depth_tf_up[1];
            static_transformStamped.transform.translation.z = _depth_tf_up[2];
            quat.setRPY(_depth_tf_up[5],_depth_tf_up[4],_depth_tf_up[3]);
            static_transformStamped.transform.rotation.x = quat.x();
            static_transformStamped.transform.rotation.y = quat.y();
            static_transformStamped.transform.rotation.z = quat.z();
            static_transformStamped.transform.rotation.w = quat.w();
            _static_broadcaster.sendTransform(static_transformStamped);
        }

        void send_tf_down(){
            geometry_msgs::TransformStamped static_transformStamped;
            tf2::Quaternion quat;

            static_transformStamped.header.stamp = ros::Time::now();
            static_transformStamped.header.frame_id = "base_link";
            static_transformStamped.child_frame_id = "pipecam_link";
            static_transformStamped.transform.translation.x = _depth_tf_down[0];
            static_transformStamped.transform.translation.y = _depth_tf_down[1];
            static_transformStamped.transform.translation.z = _depth_tf_down[2];
            quat.setRPY(_depth_tf_down[5],_depth_tf_down[4],_depth_tf_down[3]);
            static_transformStamped.transform.rotation.x = quat.x();
            static_transformStamped.transform.rotation.y = quat.y();
            static_transformStamped.transform.rotation.z = quat.z();
            static_transformStamped.transform.rotation.w = quat.w();
            _static_broadcaster.sendTransform(static_transformStamped);
        }

        int update_state(){
            // TODO use tpèic from gazebo
            return 1;
        }
        
        void publish_state_task(){
            koala_msgs::pipe_switch_state state_msg;
            ros::Rate rate(10);

            while(ros::ok()){
                if(update_state()){
                    state_msg.clamp = _clamp_state;
                    state_msg.crawler = _crawler_State;
                    state_msg.lock = _lock_State;
                    state_msg.is_upsidedown = _upsidedown_state;
                    state_msg.fwd_pipe = _fwd_pipe_state;
                    state_msg.bwd_pipe =_bwd_pipe_state;

                    _pipe_state_pub.publish(state_msg);
                    //print_state();
                    first_state = true;
                }
                
                rate.sleep();
            }
        }

        void print_state(){
            ROS_INFO("driver state: [%u %u %u %u] ",_clamp_state,_crawler_State,_lock_State,_upsidedown_state);
            //_master.printState(); //TBD remove
        }
        
        int get_state(const koala_msgs::stepperGoalConstPtr &goal){
            switch(goal->actuator_i) {
                case 1:
                    return int(_clamp_state);
                    break;
                case 2:
                    return int(_crawler_State);
                    break;
                case 3:
                    return int(_lock_State);
                    break;
                case 4:
                    return int(_upsidedown_state); 
                    break;
                default:
                    return -1;
                    break;                
            }
        }

        bool homing_done(){
            return (_clamp_state == 1 && _crawler_State == 1 && _lock_State == 1);
        }

        bool send_goal_to_can(const koala_msgs::stepperGoalConstPtr &goal){
            ROS_INFO("inner request stepper %s, to %s ", goal->actuator.c_str(),goal->command.c_str());
            if(goal->actuator == "clamp" && goal->actuator_i == 1){
                
                if(goal->command == "open" && goal->command_i == 1){
                    if(_clamp_state == STATE_CLOSED){
                        _clamp_state = STATE_OPENING;
                        ros::Duration(_sec_clamp).sleep();
                        //TODO send service to detach LIBERA DRONE
                        if(_gz_detach_client.call(_attach_srv) && _attach_srv.response.ok){
                            ROS_INFO("detach response ok");
                            _clamp_state = STATE_OPEN;
                        }
                        else{
                            ROS_INFO("failed detach srv");
                            _clamp_state = STATE_CLOSED;
                            return 0;
                        }
                    }
                    return 1;
                }
                else if(goal->command == "close" && goal->command_i == 0){
                    if(_clamp_state == STATE_OPEN){
                        _clamp_state = STATE_CLOSING;
                        ros::Duration(_sec_clamp).sleep();
                        //TODO send service to attach   BLOCCA DRONE
                        if(_gz_attach_client.call(_attach_srv) && _attach_srv.response.ok){
                            ROS_INFO("attach response ok");
                            _clamp_state = STATE_CLOSED;
                        }
                        else{
                            ROS_INFO("failed attach srv");
                            _clamp_state = STATE_OPEN;
                            return 0;
                        }
                    }
                    return 1;
                }
                else
                    return 0;
            }
            else if (goal->actuator == "crawler" && goal->actuator_i == 2){
                if(goal->command == "open" && goal->command_i == 1){
                    if(_crawler_State == STATE_CLOSED){
                        _crawler_State = STATE_OPENING;
                        ros::Duration(_sec_crawler).sleep();
                        //TODO dummy, not implemented in simulation
                        _crawler_State = STATE_OPEN;
                    }
                    return 1;
                }
                else if(goal->command == "close" && goal->command_i == 0){
                    if(_crawler_State == STATE_OPEN){
                        _crawler_State = STATE_CLOSING;
                        ros::Duration(_sec_crawler).sleep();
                        //TODO dummy, not implemented in simulation
                        _crawler_State = STATE_CLOSED;
                    }
                    return 1;
                }
                else
                    return 0;
            }
            else if (goal->actuator == "lock" && goal->actuator_i == 3){
                if(goal->command == "open" && goal->command_i == 1){
                    if(_lock_State == STATE_CLOSED){
                        _lock_State = STATE_OPENING;
                        ros::Duration(_sec_lock).sleep();
                        //TODO dummy, not implemented in simulation
                        _lock_State = STATE_OPEN;
                    }
                    return 1;
                }
                else if(goal->command == "close" && goal->command_i == 0){
                    if(_lock_State == STATE_OPEN){
                        _lock_State = STATE_CLOSING;
                        ros::Duration(_sec_lock).sleep();
                        //TODO dummy, not implemented in simulation
                        _lock_State = STATE_CLOSED;
                    }
                    return 1;
                }
                else
                    return 0;
            }
            // else if (goal->actuator == "led" && goal->actuator_i == 4){
            //     if(goal->command == "on" && goal->command_i == 1){
            //         // _master.setIrLed(1);
            //         return 1;
            //     }
            //     else if(goal->command == "off" && goal->command_i == 0){
            //         // _master.setIrLed(0);
            //         return 1;
            //     }
            //     else
            //         return 0;
            // }
            else
                return 0;
        }
	
		void preemptCB(){
			ROS_WARN("%s got preempted!", _action_name.c_str());
			_result.result = "fail";
			as.setPreempted(_result,"I got Preempted"); 
  	    }
  
		void executeCB(const koala_msgs::stepperGoalConstPtr &goal){
            ros::Rate rate(5);
            bool is_done = false;
            int st;
			if(!as.isActive() || as.isPreemptRequested()) return;
		
			ROS_INFO("request stepper %s, to %s ", goal->actuator.c_str(),goal->command.c_str());
			if(!send_goal_to_can(goal)){
                _result.result = "failed";
                as.setAborted(_result,"wrong goal !");
                ROS_ERROR("wrong action goal, abort");
                return;
            }
			while(!is_done){
				//Check for ros
				if (!ros::ok()) {
					_result.result = "failed";
					as.setAborted(_result,"I failed !");
					ROS_INFO("%s Shutting down",_action_name.c_str());
					return;
				}
                if(!as.isActive() || as.isPreemptRequested()) return;
                //update_state(); //TBD forse rimettere
                print_state();
                st = get_state(goal);
                ROS_INFO("actuator state: %d ",st);
                is_done = (st == goal->command_i);
                _feedback.state_i = st; 
                as.publishFeedback(_feedback);

		        rate.sleep();
		    }

            ROS_INFO("OK, stepper moving end");
            _result.result = "ok";
			as.setSucceeded(_result);

        }
};

int main(int argc, char** argv)
{

  ros::init(argc, argv, "demo_action");
  ROS_INFO("Starting Demo Action Server");
  STEPPER_ACTION_SERVER demo_action_obj("demo_action");
  ros::spin();
  return 0;
}