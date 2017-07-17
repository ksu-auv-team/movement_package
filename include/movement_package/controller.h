/*
https://github.com/ksu-auv-team/movement_package
controller.h
Purpose: provides parent class for creating a controller. 
Controller.ProcessChannels should be overloaded for a new type ofcontroller.
I envision this being the parent of a manual controler object and
an AI/pid based controller object.

@author shadySource
@version 0.0.1
*/
#ifndef CONTROLLER_DEF
#define CONTROLLER_DEF

#include "mavros_communicator.h"

namespace controller
{

class Controller
{
    private:
        //@var _mavrosCommunicator Object to interface with mavros
        auto mavcomm::MavrosCommunicator _mavrosCommunicator;

    
        /**
        Process channel inputs. Should be "hidden" for different controller types.
        Uses _mavrosCommunicator->SetOverrideMessage to set rc channel outputs.
        
        @note default behavior is to set all inputs to MID_PWM.
        */
        void ProcessChannels();

    public:
        /**
        Attempts to arm the FCU

        @return boolean success
        */
        bool ArmFCU();


        /**
        sets the override message in _mavrosCommunicator to the class's channel vars.
        */
        void SetOverrideMessage();


        /**
        Listens for teleop twist message and updates motors accordingly.
        */
        void ControlLoop();
    

    Controller();

};//end Controller


}//end namespace controller

#endif