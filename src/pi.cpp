#include <iostream>
#include "pi.h"

//3.1415926
PI::PI()
{
}

PI::PI(int inTopLimit,int inBottomLimit,int inChannel)
{	
	topLimit = inTopLimit;
	bottomLimit = inBottomLimit;
	channel = inChannel;
	cl = clock();
}
	
double PI::getError()
{
	return goal-pose;
}

int PI::getCommand()
{
	dt = (clock() - cl)/(double)CLOCKS_PER_SEC;
	//cout <<dt;
	error = getError();
	/*
		Program integrator anti windup here
	*/
	int command = 1500-(kp*error+ki*sigma);
	if(command>topLimit)
		command = topLimit;
	if(command<bottomLimit)
		command = bottomLimit;
	setSigma();
	cl = clock();
	return command;
}

void PI::setPID(bool inStatus,int inGoal, int inPose, int inMode)
{
	status = inStatus;				//inStatus has to be true
	setGoal(inGoal);
	setPose(inPose);
	setMode(inMode);
	setGains();
}

void PI::setSigma()
{
	if(status)
		sigma += error*dt;
	else
	    sigma = 0;
}

void PI::setGains()
{
	switch(channel)
	{
			case 1:							 //roll
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0; ki = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0.9375; ki = 0;
						break;
				}
				break;
			case 2:							 //pitch
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0; ki = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 1.25; ki = 0;
						break;				
				}
				break;
			case 3:							 //throttle
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0.8333; ki = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0; ki = 0;
						break;				
				}
				break;
			case 4:							 //yaw
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = .46875; ki = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0; ki = 0;
						break;				
				}
				break;
	}
}

void PI::setGoal(int inGoal)
{
	goal = inGoal;
}

void PI::setPose(int inPose)
{
	pose = inPose;
}

void PI::setMode(int inMode)
{
	mode = inMode;
}

int PI::roll_command()
{
	switch(mode)
	{
		case 1:					//1 should be forward facing camera 
			return 1500;
			break;
		case 2:					//2 should be downard facing camera
			return getCommand();
			break;						
	}
}

int PI::yaw_command()
{
	switch(mode)
	{
		case 1:					//1 should be forward facing camera 
			return getCommand();
			break;
		case 2:					//2 should be downard facing camera
			return 1500;
			break;						
	}

}

int PI::throttle_command()
{
	switch(mode)
	{
		case 1:					//1 should be forward facing camera 
			return getCommand();
			break;
		case 2:					//2 should be downard facing camera
			return 1500;
			break;						
	}

}

int PI::pitch_command()
{
	switch(mode)
	{
		case 1:					//1 should be forward facing camera 
			return 1500;
			break;
		case 2:					//2 should be downard facing camera
			return getCommand();
			break;						
	}

}

double PI::getPercentError()
{
	return 100*getError()/goal;
}

void PI::reset()
{
	status = false;
	setSigma();
}



