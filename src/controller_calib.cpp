#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include "pi.h"

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN	4
#define HIGH_PWM	1900
#define MID_PWM 	1500
#define LOW_PWM 	1100

using namespace std;

ros::Publisher auv_pid_rc_override;

int x,y;

void poseMessage(const std_msgs::Int32MultiArray& msg){
		x=msg.data[1];
        y=msg.data[2];
}

int main( int argc, char** argv ){
    ros::init(argc, argv,"controller_calib_node");
	ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);
    ros::Subscriber sub_obj = nh.subscribe("std_msgs/Int32MultiArray", 1000, &poseMessage);
    auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1000);
    mavros_msgs::OverrideRCIn MAV_MSG;
    PI calibrated_controller(1800,1200,4);  //PI(int inTopLimit,int inBottomLimit,int inChannel) 1-roll 2-pitch 3-throttle 4-yaw
    while (ros::ok())
    {
        //void setPID(bool inStatus,int inGoal, int inPose, int inMode);
        //inGoal should be the center of the cam so if 640x480 x_inGoal = 320 y_inGoal = 240
        //inPose is the location of the tracked object the x and y coordinates are treated in the callback function above
        //inMode is for the camera used 1- Forward Facing Camera 2- Downward Facin Camera
        calibrated_controller.setPID(true,0,1,1); 
        //Set the mavros commands here
        //We could use the PI::getCommand() function but we'll use the other control commands instead
        //It'll allow us to set the PI but also test the commands use in the actual code
        //PI::roll_command()
        //PI::yaw_command()
        //PI::throttle_command()
        //PI::pitch_command()
        MAV_MSG.channels[ROLL_CHAN] = 1500; //calibrated_controller.roll_command();
		MAV_MSG.channels[PITCH_CHAN] = 1500; //calibrated_controller.pitch_command();							
		MAV_MSG.channels[THROT_CHAN] = 1500; //calibrated_controller.throttle_command();
		MAV_MSG.channels[YAW_CHAN] = calibrated_controller.yaw_command();
		MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
		auv_pid_rc_override.publish(MAV_MSG);		
        ros::spinOnce();
        RC_COMM_RATE.sleep();
    }
}