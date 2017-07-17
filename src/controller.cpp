#include "controller.h"

Controller::Controller()
    : INFO_RATE(10), FCU_COM_RATE(45)
{
    // Services
    _modeSrv = _n.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    _armSrv = _n.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    _paramSrv = _n.serviceClient<mavros_msgs::ParamSet>("/mavros/param/set");
    _streamRateSrv = _n.serviceClient<mavros_msgs::StreamRate>("/mavros/set_stream_rate");
    
    _sysidMsg.request.param_id = "SYSID_MYGCS";
    _sysidMsg.request.value.integer = 1;
    _streamRateMsg.request.stream_id = 0;
    _streamRateMsg.request.message_rate = INFO_RATE;
    _streamRateMsg.request.on_off = true;
    _stabilizeModeMsg.request.base_mode = 0;
    _stabilizeModeMsg.request.custom_mode = "STABILIZE";
    _acroModeMsg.request.base_mode = 0;
    _acroModeMsg.request.custom_mode = "ACRO";
    _armMsg.request.value = true;
    _disarmMsg.request.value = false;
}

bool Controller::CommInit()
{
    if (_streamRateSrv.call(_streamRateMsg))
    {
        ROS_INFO("Stream rate set to %d.", _streamRateMsg.request.message_rate);
    }
    else
    {
        ROS_WARN("Failed to use mavros/set_stream_rate service. Rate not set.");
    }

    if (_paramSrv.call(_sysidMsg))
    {
        if (_sysidMsg.response.success)
        {
            ROS_INFO("Set SYSID_MYGCS Succeeded.");
        }
        else
        {
            ROS_WARN("Set SYSID_MYGCS Failed.");
        }
        return _sysidMsg.response.success;
    }

    ROS_WARN("Failed to use mavros/param/set service. SYSID_MYGCS not set. RC Override will FAIL.");
    return false;
}

bool Controller::ArmFCU()
{
    if (_armSrv.call(_armMsg))
    {
        if (_armMsg.response.success)
        {
            ROS_INFO("Arm succeeded.");
        }
        else
        {
            ROS_WARN("Arm failed.");
        }
        return _armMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/cmd/arming serivce. Arm failed.");
    return false;
}

bool Controller::DisarmFCU()
{
    if (_armSrv.call(_disarmMsg))
    {
        if (_disarmMsg.response.success)
        {
            ROS_INFO("Disarm succeeded.");
        }
        else
        {
            ROS_WARN("Disarm failed.");
        }
        return _disarmMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/cmd/arming serivce. Disarm failed.");
    return false;
}

bool Controller::SetModeAcro()
{
    if (_modeSrv.call(_acroModeMsg))
    {
        if (_acroModeMsg.response.success)
        {
            ROS_INFO("Set mode to ACRO succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to ACRO failed.");
        }
        return _acroModeMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to ACRO failed.");
    return false;
}

bool Controller::SetModeStabilize()
{
    if (_modeSrv.call(_stabilizeModeMsg))
    {
        if (_stabilizeModeMsg.response.success)
        {
            ROS_INFO("Set mode to STABILIZE succeeded.");
        }
        else
        {
            ROS_WARN("Set mode to STABILIZE failed.");
        }
        return _stabilizeModeMsg.response.success;
    }
    ROS_WARN("Failed to use /mavros/set_mode serivce. Set mode to STABILIZE failed.");
    return false;
}