
#ifndef AI_CONTROLLER_DEF
#define AI_CONTROLLER_DEF
#include "controller.h"
#include "pid.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
namespace controller
{
class AIController : public Controller
{
    private:
        const int PERCENT_ERROR;

        float _x, _y, _dist, _yOffset;
        
        int  _mode, _pastMode;

        std_msgs::Bool _setpointReached;

        ros::NodeHandle _nh;

        ros::Subscriber _targetSub;

        ros::Publisher _setpointReachedPub;

        PID _throttleController, _yawController, _forwardController, _lateralController;

        void TargetCallback(const std_msgs::Float32MultiArray& msg);

    public:

        void ProcessChannels();

    AIController();

};

}

#endif