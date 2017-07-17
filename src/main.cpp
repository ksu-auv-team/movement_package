#include "controller.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_package_node");

    auto sub_controller = new Controller();

    sub_controller->CommInit();

    sub_controller->ArmFCU();

    sub_controller->SetModeStabilize();
}