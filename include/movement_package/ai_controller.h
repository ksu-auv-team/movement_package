
#ifndef AI_CONTROLLER_DEF
#define AI_CONTROLLER_DEF
#include "controller.h"
#include "pid.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Int32MultiArray.h>
namespace controller
{
class AIController : public Controller
{
    private:
        const int PERCENT_ERROR;

        int _x, _y, _dist, _yOffset, _mode, _pastMode;

        std_msgs::Bool _setpointReached;

        ros::NodeHandle _nh;

        ros::Subscriber _targetSub;

        ros::Publisher _setpointReachedPub;

        PID _throttleController, _yawController, _forwardController, _lateralController;

        void TargetCallback(const std_msgs::Int32MultiArray& msgs);

    public:

        void ProcessChannels();

    AIController();

};

}

#endif