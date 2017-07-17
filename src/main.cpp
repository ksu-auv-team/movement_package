#include "mavros_communicator.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "movement_package_node");

    auto sub_controller = new mavcomm::MavrosCommunicator();

    sub_controller->CommInit();

    sub_controller->ArmFCU();

    sub_controller->MotorTest();

    delete sub_controller;
}