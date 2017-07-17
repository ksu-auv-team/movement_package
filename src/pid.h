#include <iostream>	
#include <string>
#include <fstream>
#include <cmath>

using namespace std;

#define eps 0.01
/*
We will use a class and create PID objects that we can call inside of our main loop
The meaning and use of the following variables and functions are explained in the 
implementation "pid.cpp"
*/
class PID{
private: 
	double goal;
	double pose;
	int mode; 		
	int channel; //2-throttle 3-yaw 4-forward 5-lateral
	bool status;
	double error;
	double last_error;
	long sigma;
	double kp;
	double ki;
	double kd;
	double dt;
	clock_t cl; 	
public:
	PID();
	PID(int inTopLimit,int inBottomLimit,int inChannel);
	double getError();
	int topLimit;
	int bottomLimit;
	int getCommand();
	void setPID(bool inStatus,int inGoal, int inPose, int inMode);
	void setSigma();
	void setGains();
	void setGoal(int inGoal);
	void setPose(int inPose);
	void setMode(int inMode);
	int throttle_command();
	int yaw_command();
	int forward_command();
	int lateral_command();
	double getPercentError();
	void reset();
};
