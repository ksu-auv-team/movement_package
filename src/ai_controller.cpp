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
    _mode = round(msg.data[0]);
    for(int i(1); i < message.data.size(); i++)
        _controlMsg[i-1] = msg.data[i];
    
}

void AIController::UpdatePIDs()
{
    delete _throttleController;
    delete _yawController;
    delete _forwardController;
    delete _lateralController;
    switch(_mode)
    {
        case TRACK_FRONT_HOLD_DEPTH:
            /*
                Throttle: (error in meters depth)
                p : 500/0.762:  full throt at 2.5 ft. err, 2.5ft~=.76 meters
                i : 100: 3 sec, 4 in error -> -100 pwm
                d : 600 : (1/2 ft delta error)/sec  -> -100 pwm
            */
            _throttleController = new PID(HIGH_PWM, LOW_PWM, MID_PWM, 500/0.762, 100, 600);
            /*
                Yaw:
                p : 500: 1/2 throttle (250) in max image error (0.5)
                i : 25: should not be much continuous error over time...
                d : 300
            */
            _yawController = new PID(HIGH_PWM, LOW_PWM, MID_PWM, 500, 25, 300);

            break;
        case TRACK_FRONT:
            //todo
            break;
        case TRACK_BOTTOM_HOLD_DEPTH:
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
            _throttleController->UpdatePID(true, 0, _controlMsg[1], 0);//depth hold, y is depth
            _yawController.setPID(true, _controlMsg[2], _currentDepth, 0);
            MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _throttleController->throttle_command());
            MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _yawController->yaw_command());
            MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _controlMsg[3]);
            MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _controlMsg[4]);
            break;
        case TRACK_FRONT:	//1 should be forward facing camera 
            _throttleController.setPID(true, 0, _y+_yOffset, 1);
            _yawController.setPID(true, 0, _x, 1);
            _forwardController.setPID(true, 0, _dist,1);
            _lateralController.setPID(true, 0, 0, 1);
            MavrosCommunicator->SetOverrideMessage(THROTTLE_CHAN, _throttleController->throttle_command());
            MavrosCommunicator->SetOverrideMessage(YAW_CHAN, _yawController.yaw_command());
            MavrosCommunicator->SetOverrideMessage(FORWARD_CHAN, _forwardController.forward_command());
            MavrosCommunicator->SetOverrideMessage(LATERAL_CHAN, _lateralController.lateral_command());
            break;
        case TRACK_BOTTOM_AT_DEPTH:	//2 should be downard facing camera
            _throttleController.setPID(true, 0, 0, 2);
            _yawController.setPID(true, 0, 0, 2);
            _forwardController.setPID(true, 0, _y, 2);
            _lateralController.setPID(true, 0, _x, 2);
            break;
    }

    
    
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