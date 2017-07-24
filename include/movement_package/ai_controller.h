
#ifndef AI_CONTROLLER_DEF
#define AI_CONTROLLER_DEF

#include "controller.h"
#include "pid.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
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
            
        int  _mode, _pastMode;

        std_msgs::Bool _setpointReached;

        ros::NodeHandle _nh;

        ros::Subscriber _targetSub;

        ros::Publisher _setpointReachedPub;

        PID *_throttleController, *_yawController, *_forwardController, *_lateralController;

        void TargetCallback(const std_msgs::Float32MultiArray& msg);

        void DepthCallback();//TODO: monitor depth

    public:

        void ProcessChannels();

    AIController();

};

}

#endif