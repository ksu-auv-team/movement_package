#include <iostream>
#include "pi.h"

using namespace std;

int main()
{
	PI roll_controller(1900,1100,1);
	while(1)
	{
		roll_controller.setPID(true,0,1,1);
		cout <<roll_controller.getError() <<" " <<roll_controller.getCommand() <<" " <<roll_controller.roll_command() <<endl;
	}
}
