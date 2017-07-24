
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
        enum Modes
        {
            TRACK_FRONT_AT_DEPTH,
            TRACK_FRONT,
            TRACK_BOTTOM_AT_DEPTH,
            TRACK_BOTTOM
        };

        const int PERCENT_ERROR;

        float _controlMsg[4];
            
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