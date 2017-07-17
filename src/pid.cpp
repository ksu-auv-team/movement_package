#include <iostream>
#include "pid.h"

//3.1415926

PID::PID()
{
}

PID::PID(int inTopLimit,int inBottomLimit,int inChannel)
{	
	topLimit = inTopLimit;
	bottomLimit = inBottomLimit;
	channel = inChannel;
	cl = clock();
}
	
double PID::getError()
{
	return goal-pose;
}

int PID::getCommand()
{
	dt = (clock() - cl)/(double)CLOCKS_PER_SEC;
	//cout <<dt;
	error = getError();
	/*
		Program integrator anti windup here
	*/
	int command = 1500-(kp*error+ki*sigma+kd*(error-last_error)/dt);
	if(command>topLimit)
		command = topLimit;
	if(command<bottomLimit)
		command = bottomLimit;
	setSigma();
	cl = clock();
	return command;
}

void PID::setPID(bool inStatus,int inGoal, int inPose, int inMode)
{
	status = inStatus;				//inStatus has to be true
	setGoal(inGoal);
	setPose(inPose);
	setMode(inMode);
	setGains();
}

void PID::setSigma()
{
	if(status)
		sigma += error*dt;
	else
	    sigma = 0;
}

void PID::setGains()
{
	switch(channel)
	{
			case 1:							 //roll
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0; ki = 0; kd = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0.9375; ki = 0; kd = 0;
						break;
				}
				break;
			case 2:							 //pitch
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0; ki = 0; kd = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 1.25; ki = 0; kd = 0;
						break;				
				}
				break;
			case 3:							 //throttle
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = 0.8333; ki = 0; kd = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0; ki = 0; kd = 0;
						break;				
				}
				break;
			case 4:							 //yaw
				switch(mode)
				{
					case 1:					//1 should be forward facing camera 
						kp = .46875; ki = 0; kd = 0;
						break;
					case 2:					//2 should be downard facing camera
						kp = 0; ki = 0; kd = 0;
						break;				
				}
				break;
	}
}

void PID::setGoal(int inGoal)
{
	goal = inGoal;
}

void PID::setPose(int inPose)
{
	pose = inPose;
}

void PID::setMode(int inMode)
{
	mode = inMode;
}

int PID::roll_command()
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

int PID::yaw_command()
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

int PID::throttle_command()
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

int PID::pitch_command()
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

double PID::getPercentError()
{
	if (goal !=0)
		return 100*abs(getError())/goal;
	else
		return 100*abs(getError())/eps;
}

void PID::reset()
{
	status = false;
	setSigma();
}



