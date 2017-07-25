#include "controller.h"

using namespace controller;

Controller::Controller() 
    : MavrosCommunicator(new  mavcomm::MavrosCommunicator)
{
    while(!MavrosCommunicator->CommInit())
    {
        ROS_INFO("Communication initialization failed. Retrying.");
        ros::Duration(0.5).sleep();
    }
}

Controller::~Controller()
{
    this->Disarm();
    delete MavrosCommunicator;
}

void Controller::ProcessChannels()
{
    //ex. set all 6 channels to MID_PWM
    MavrosCommunicator->SetOverrideMessage(PITCH_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(ROLL_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, MID_PWM);
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, MID_PWM);
}

bool Controller::Arm()
{
    bool success = false;
    for (int i = 0; i < 20; i++)
    {
        if (MavrosCommunicator->ArmFCU())
        {
            success = true;
            break;
        }
        else
        {
            ROS_WARN("Attempt %d to arm the FCU failed.", i);
            ros::Duration(1).sleep();
        }
    }
    if (!success){
        ROS_WARN("Could not arm FCU.");
    }
  return success;    
}

bool Controller::Disarm()
{
    bool success = false;
    for (int i = 0; i < 20; i++)
    {
        if (MavrosCommunicator->DisarmFCU())
        {
            success = true;
            break;
        }
        else
        {
            ROS_WARN("Attempt %d to disarm the FCU failed.", i);
            ros::Duration(0.25).sleep();
        }
    }
    if (!success){
        ROS_ERROR("Could not disarm FCU.");
    }
    return success;    
}

void Controller::ControlLoop()
{
    while(ros::ok())
    {
        this->ProcessChannels(); //do calculations for channels
        MavrosCommunicator->PublishOverrideMessage();
        ros::spinOnce();
        MavrosCommunicator->FCUCommRate.sleep();
    }
}