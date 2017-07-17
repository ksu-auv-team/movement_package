/**
https://github.com/ksu-auv-team/movement_package
movement_package_main.cpp
Purpose: Provides methods and functions for communicating to mavros

@author shadySource
@version 0.0.1
*/
#include <chrono>
#include <string.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ParamSet.h>
#include <mavros_msgs/StreamRate.h>
#include <sub_control_definitions.h>
#include <pid.h>


class MavComm
{
    private:
        // Basic NodeHandles
        ros::NodeHandle _n;

        // FCU Communication Rates
        const int INFO_RATE;
        const int FCU_COMM_RATE;

        // Service Params
        ros::ServiceClient _streamRateSrv, _modeSrv, _armSrv, _paramSrv;
        mavros_msgs::SetMode _stabilizeModeMsg, _acroModeMsg, _manualModeMsg;
        mavros_msgs::CommandBool _armMsg, _disarmMsg;
        mavros_msgs::ParamSet _sysidMsg;
        mavros_msgs::StreamRate _streamRateMsg;

        // Publishers
        ros::Publisher _overridePub;


    public:
        /**
        Sets SYSID_MYGCS to 1 and FCU stream rate to 10.
        SYSID_MYGCS must be set to 1 for OverriceRCIn to function.

        @return true if set SYSID_MYGCS succeeds,
        false if set SYSID_MYGCS fails
        */
        bool CommInit();


        /**
        Arms the FCU

        @return boolean success
        */
        bool ArmFCU();


        /**
        Disarms the FCU

        @return boolean success
        */
        bool DisarmFCU();


        /**
        Sets FCU mode to "ACRO"

        @return boolean success
        */
        bool SetModeAcro();


        /**
        Sets FCU Mode to "STABILIZE"

        @return boolean success
        */
        bool SetModeStabilize();


        /**
        Sets FCU Mode to "MANUAL"

        @return boolean success
        */
        bool SetModeManual();

        /**
        Checks the motors by spinning each motor for one second

        @param num_motors: number of motors connected to the pixhawk.
        ex. 6 will spin all motors 1-6.
        @return bool success
        */
        bool MotorTest(int num_motors);

 
    MavComm();

    ~MavComm();

};