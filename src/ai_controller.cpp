#include "ai_controller.h"

using namespace controller;

AIController::AIController()
        : PERCENT_ERROR(5),
        _throttleController(HIGH_PWM, LOW_PWM),
        _yawController(HIGH_PWM, LOW_PWM),
        _forwardController(HIGH_PWM, LOW_PWM),
        _lateralController(HIGH_PWM, LOW_PWM)
{
    _targetSub = _nh.subscribe("pi_loop_data", 10, &AIController::TargetCallback, this);

    _setpointReachedPub = _nh.advertise<std_msgs::Bool>("pid_loop_check",10);

    _pastMode = -1;
}

void AIController::TargetCallback(const std_msgs::Float32MultiArray& msg)
{
    _controlMsg[0] = msg.data[0];
    _controlMsg[1] = msg.data[1];
    _controlMsg[2] = msg.data[2];
    _controlMsg[3] = msg.data[3];
    _mode = round(msg.data[4]);
}

void AIController::UpdatePIDs()
{
    _throttleController.reset();
    _yawController.reset();
    _forwardController.reset();
    _lateralController.reset();
    switch(_mode)
    {
        case 0:
            //do things
            break;
        case 2:
            //do
            break;
    }
}

void AIController::ProcessChannels()
{
    if(_mode != _pastMode)
    {
        UpdatePIDs();
    }
    switch(_mode)
    {
        case 0: //0 should be depth hold,with control channel2 acting as depth 
            _throttleController.setPID(true, 0, _controlMsg[1], 0);//depth hold, y is depth
            _yawController.setPID(true, 0, _controlMsg[0], 0);
            _forwardController.setPID(true, 0, _dist,0);
            _lateralController.setPID(true, 0, 0, 0);
            break;
        case 1:	//1 should be forward facing camera 
            _throttleController.setPID(true, 0, _y+_yOffset, 1);
            _yawController.setPID(true, 0, _x, 1);
            _forwardController.setPID(true, 0, _dist,1);
            _lateralController.setPID(true, 0, 0, 1);
            break;
        case 2:	//2 should be downard facing camera
            _throttleController.setPID(true, 0, 0, 2);
            _yawController.setPID(true, 0, 0, 2);
            _forwardController.setPID(true, 0, _y, 2);
            _lateralController.setPID(true, 0, _x, 2);
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
    }
    else
    {
        _setpointReached.data = false;
    }

    _setpointReachedPub.publish(_setpointReached);

}