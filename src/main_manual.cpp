#include "manual_controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "manual_control_node");

    auto manualController = new controller::ManualController();

    manualController->Arm();

    manualController->ControlLoop();

}