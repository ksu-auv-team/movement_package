#include "ai_controller.h"

using namespace controller;

AIController::AIController()
        : PERCENT_ERROR(5)
{
    _targetSub = _nh.subscribe("pi_loop_data", 10, &AIController::TargetCallback, this);

    _setpointReachedPub = _nh.advertise<std_msgs::Bool>("pid_loop_check",10);

    _pastMode = -1;
}

AIController::~AIController()
{
    delete _throttleController;
    delete _yawController;
    delete _forwardController;
    delete _lateralController;
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
    delete _throttleController;
    delete _yawController;
    delete _forwardController;
    delete _lateralController;
    switch(_mode)
    {
        case TRACK_FRONT_AT_DEPTH:
            _throttleController = new PID(HIGH_PWM, LOW_PWM, 900, 0, 0);
            break;
        case TRACK_FRONT:
            //todo
            break;
        case TRACK_BOTTOM_AT_DEPTH:
            //todo
            break;
    }
}

void AIController::ProcessChannels()
{
    if(_mode != _pastMode)
    {
        UpdatePIDs();
        _pastMode = _mode;
    }
    switch(_mode)
    {
        case TRACK_FRONT_AT_DEPTH: //0 should be depth hold,with control channel2 acting as depth 
            _throttleController.setPID(true, 0, _controlMsg[1], 0);//depth hold, y is depth
            _yawController.setPID(true, 0, _controlMsg[0], 0);
            _forwardController.setPID(true, 0, _dist,0);
            _lateralController.setPID(true, 0, 0, 0);
            break;
        case TRACK_FRONT:	//1 should be forward facing camera 
            _throttleController.setPID(true, 0, _y+_yOffset, 1);
            _yawController.setPID(true, 0, _x, 1);
            _forwardController.setPID(true, 0, _dist,1);
            _lateralController.setPID(true, 0, 0, 1);
            break;
        case TRACK_BOTTOM_AT_DEPTH:	//2 should be downard facing camera
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