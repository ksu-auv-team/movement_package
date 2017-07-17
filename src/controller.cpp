#include "manual_controller.h"

using namespace controller;

Controller::Controller()
    : roll(MID_PWM), pitch(MID_PWM), yaw(MID_PWM), 
    throttle(MID_PWM), forward(MID_PWM), lateral(MID_PWM)
{
    ;
}

void Controller::ProcessChannels()
{
    _mavrosCommunicator->SetOverrideMessage();//set all to MID_PWM
}

bool Controller::ArmFCU()
{
    bool success = false;
    for (int i = 0; i < 20; i++)
    {
        if (_mavrosCommunicator->ArmFCU())
        {
            success = true;
            break;
        }
        else
        {
            ROS_WARN("Attempt %d to arm the FCU failed.", i)
            ros::Duration(0.5).sleep()
        }
    }
    if (!success){
        ROS_WARN("Could not arm FCU.")
    }
    return success;    
}

void Controller::ControlLoop()
{
    while(ros::ok())
    {
        this->ProcessChannels();
        _mavrosCommunicator->PublishOverrideMessage();
        _mavrosCommunicator->FCUCommRate.sleep();
    }
}