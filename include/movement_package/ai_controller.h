
#ifndef AI_CONTROLLER_DEF
#define AI_CONTROLLER_DEF

#include "controller.h"
#include "pid.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/FluidPressure.h>
#include <math.h>

namespace controller
{
class AIController : public Controller
{
    private:
        /**
        0 : TRACK_FRONT_HOLD_DEPTH
            message[0] = This Mode (0)
            message[1] = x camera position
            message[2] = depth(meters)
            message[3] = forward throttle
            message[4] = lateral throttle
        1 : TRACK_FRONT
            message[0] = This Mode (0)
            message[1] = x camera position
            message[2] = depth(meters)
            message[3] = forward throttle
            message[4] = lateral throttle
        */
        enum Modes
        {
            TRACK_FRONT_HOLD_DEPTH,
            TRACK_FRONT,
            TRACK_BOTTOM_HOLD_DEPTH,
            TRACK_BOTTOM
        };

        const int PERCENT_ERROR;

        float _controlMsg[10];

        double _startTime;

        int  _mode, _pastMode;

        bool _pressureCollected;
        vector<double> _pressureData;
        double _surfacePressure;
        float _currentDepth;

        std_msgs::Bool _setpointReached;

        ros::NodeHandle _nh;

        ros::Subscriber _targetSub, _pressureSub;

        ros::Publisher _setpointReachedPub;

        PID *_throttleController, *_yawController, *_forwardController, *_lateralController;

        void TargetCallback(const std_msgs::Float32MultiArray& msg);

        void DepthCallback(const sensor_msgs::FluidPressure& msg);

        void NewMode();

    public:

        void ProcessChannels();

    AIController();
    ~AIController();
};

}

#endif