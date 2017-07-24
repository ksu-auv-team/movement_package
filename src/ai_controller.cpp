#include "ai_controller.h"

using namespace controller;

AIController::AIController()
        : PERCENT_ERROR(5),
        _throttleController(HIGH_PWM,LOW_PWM,THROTTLE_CHAN),
        _yawController(HIGH_PWM,LOW_PWM,YAW_CHAN),
        _forwardController(HIGH_PWM,LOW_PWM,FORWARD_CHAN),
        _lateralController(HIGH_PWM,LOW_PWM,LATERAL_CHAN)
{
    _targetSub = _nh.subscribe("pi_loop_data", 10, &AIController::TargetCallback, this);

    _setpointReachedPub = _nh.advertise<std_msgs::Bool>("pid_loop_check",10);

    _pastMode = 0;
}

void AIController::TargetCallback(const std_msgs::Float32MultiArray& msg)
{
    _x=msg.data[0];
    _y=msg.data[1];
    _dist=msg.data[2];
    _yOffset=msg.data[3];
    _mode=msg.data[4];
}

void AIController::ProcessChannels()
{
    if(_mode != _pastMode)
    {
        _throttleController.reset();
        _yawController.reset();
        _forwardController.reset();
        _lateralController.reset();
    }
    switch(_mode)
    {
        case 1:	//1 should be forward facing camera 
            _throttleController.setPID(true, 0, _y+_yOffset, _mode);
            _yawController.setPID(true, 0, _x, _mode);
            _forwardController.setPID(true, 0, _dist,_mode);
            _lateralController.setPID(true, 0, 0, _mode);
            break;
        case 2:	//2 should be downard facing camera
            _throttleController.setPID(true, 0, 0, _mode);
            _yawController.setPID(true, 0, 0, _mode);
            _forwardController.setPID(true, 0, _y, _mode);
            _lateralController.setPID(true, 0, _x, _mode);
            break;
    }

    MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _throttleController.throttle_command());
    MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _yawController.yaw_command());
    MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _forwardController.forward_command());
    MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _lateralController.lateral_command());
    if((_throttleController.getPercentError()<PERCENT_ERROR)&&
        (_yawController.getPercentError()<PERCENT_ERROR)&&
        (_forwardController.getPercentError()<PERCENT_ERROR)&&
        (_lateralController.getPercentError()<PERCENT_ERROR))
    {
        _setpointReached.data = true;
        _setpointReachedPub.publish(_setpointReached);
    }
    else
    {
        _setpointReached.data = false;
        _setpointReachedPub.publish(_setpointReached);
    }
}