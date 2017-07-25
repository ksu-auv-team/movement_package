#include "manual_controller.h"

using namespace controller;

ManualController::ManualController()
{
    MavrosCommunicator->SetModeStabilize();

    _joyStickSub = _n.subscribe("joy", 10, &ManualController::JoyStickCallback, this);
    
    _n.setParam("joy_node/dev", "/dev/input/js0");

    _joyMsg.axes = {-0.0, -0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    _joyMsg.buttons = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

    this->Disarm();
    _manualArmed = false;
}

void ManualController::JoyStickCallback(const sensor_msgs::Joy& msg)
{
    _lastMsgRecieved = ros::Time::now().toSec();
    _joyMsg = msg;
}

void ManualController::SafeArm()
{
    int messageTime(ros::Time::now().toSec() - _lastMsgRecieved);
    ROS_INFO("%d", messageTime);
    if (!_manualArmed) //not armed
    {
        if (_joyMsg.buttons[9] == 1 && messageTime < 10) //trigger pressed
        {
            this->Arm();
            _manualArmed = true;
        }
    }
    else //armed
    {
        if (_joyMsg.buttons[9] != 1) //trigger not pressed
        {
            this->Disarm();
            _manualArmed = false;
        }
        else if (messageTime > 10)
        {
            this->Disarm();
            _manualArmed = false;
        }
    }
}

void ManualController::ProcessChannels()
{
    SafeArm();//trigger-arm
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _joyMsg.axes[2]*-500 + MID_PWM);//right stick left-right
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _joyMsg.axes[3]*500 + MID_PWM);//right stick up-down
    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _joyMsg.axes[1]*500 + MID_PWM);//left stick up-down
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _joyMsg.axes[0]*-500 + MID_PWM);//left stick left-right
}