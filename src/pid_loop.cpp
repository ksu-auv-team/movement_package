#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Bool.h>
#include <std_msgs/MultiArrayDimension.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "pid.h"

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN	4
#define HIGH_PWM	1900
#define MID_PWM 	1500
#define LOW_PWM 	1100
#define PERCENT_ERR 5

ros::Publisher auv_pid_rc_override;
ros::Publisher pi_loop_check;

int x,y,mode;
double dist,desiredDist;

void poseMessage(const std_msgs::Int32MultiArray& msg){
		x=msg.data[0];
        y=msg.data[1];
        dist=msg.data[2];
        desiredDist=msg.data[3];
        mode=msg.data[4];
}

int main( int argc, char** argv ){
    ros::init(argc, argv,"Leviathan_PI_Controller");
	ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber sub_obj = nh.subscribe("pi_loop_data", 1000, &poseMessage);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    pi_loop_check = nh.advertise<std_msgs::Bool>("pi_loop_check",1000);
    mavros_msgs::OverrideRCIn MAV_MSG;
    int past_mode = 0;
    std_msgs::Bool boolVar;
    PID roll_controller(1800,1200,1);
	PID pitch_controller(1800,1200,2);
	PID throttle_controller(1800,1200,3);
	PID yaw_controller(1800,1200,4);
	while (ros::ok())
    {
		if(mode != past_mode)
		{
			roll_controller.reset();
			pitch_controller.reset();
			throttle_controller.reset();
			yaw_controller.reset();
		}
        //Set PID based on the camera used
        switch(mode)
		{
			case 1:					//1 should be forward facing camera 
                roll_controller.setPID(true,0,0,mode);
                pitch_controller.setPID(true,desiredDist,dist,mode);
                throttle_controller.setPID(true,360,y,mode);
                yaw_controller.setPID(true,640,x,mode);
				break;
			case 2:					//2 should be downard facing camera
				roll_controller.setPID(true,320,x,mode);
                pitch_controller.setPID(true,240,y,mode);
                throttle_controller.setPID(true,0,0,mode);
                yaw_controller.setPID(true,0,0,mode);
                break;
		}
        MAV_MSG.channels[ROLL_CHAN] = roll_controller.roll_command();
		MAV_MSG.channels[PITCH_CHAN] = pitch_controller.pitch_command();							
		MAV_MSG.channels[THROT_CHAN] = throttle_controller.throttle_command();
		MAV_MSG.channels[YAW_CHAN] = yaw_controller.yaw_command();
		MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
		auv_pid_rc_override.publish(MAV_MSG);
        if((roll_controller.getPercentError()<PERCENT_ERR)&&(pitch_controller.getPercentError()<PERCENT_ERR)&&(throttle_controller.getPercentError()<PERCENT_ERR)&&(yaw_controller.getPercentError()<PERCENT_ERR))
            {
                boolVar.data=true;
                pi_loop_check.publish(boolVar);
            }
        else
            {
                boolVar.data=false;
                pi_loop_check.publish(boolVar);
            }
        ros::spinOnce();
        RC_COMM_RATE.sleep();

    }
    return 0;
}