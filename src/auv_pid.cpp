#include "ros/ros.h"
#include <ros/console>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/Mavlink.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>

#define ROLL_CHAN 	0
#define PITCH_CHAN 	1
#define THROT_CHAN 	2
#define YAW_CHAN 	3
#define MODES_CHAN	4

#define HIGH_PWM	2000
#define MID_PWM 	1500
#define LOW_PWM 	1000

void yolo_callback(const [data_type::data specific::ConstPtr& msg])
{
    for (int i=0; i < 2; i++) //recieving data type f 1x2
    {
        yolo_input[i] = msg->[label of data];
        
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "auv_pid");
    ros::NodeHandle nh;
    ros::Rate RC_COMM_RATE(45);

    ros::Subscriber yolo_input = nh.subscriber("name of yolo", 10, yolo_callback);
    ros::Publisher auv_pid_rc_override = nh.advertise<mavros_msgs::OverrideRCIn>("mavros/rc/override", 1);
    
    mavros_msgs::OverrideRCIn MAV_MSG;
    while (ros::ok())
    {
        MAV_MSG.channels[ROLL_CHAN] = ((axes_input[0])*500)+1500;
		MAV_MSG.channels[PITCH_CHAN] = ((axes_input[1])*500)+1500;
		MAV_MSG.channels[THROT_CHAN] = ((axes_input[4])*500)+1500;
		MAV_MSG.channels[YAW_CHAN] = ((axes_input[3])*500)+1500;
		MAV_MSG.channels[MODES_CHAN] = HIGH_PWM;
        auv_pid_rc_override.publish(MAV_MSG);
        ros::spinOnce();
        RC_COMM_RATE.sleep;      
    }
    return 0;
}