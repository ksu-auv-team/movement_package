#include <iostream>	
#include <string>
#include <fstream>
#include <cmath>
#include<math.h>

using namespace std;

#define eps 0.0001
/*
We will use a class and create PID objects that we can call inside of our main loop
The meaning and use of the following variables and functions are explained in the 
implementation "pid.cpp"
*/
class PID{
private: 
	float goal;
	float pose;
	int mode; 		
	int channel; //2-throttle 3-yaw 4-forward 5-lateral
	bool status;
	float error;
	float last_error;
	long sigma;
	double kp;
	double ki;
	double kd;
	double dt;
	clock_t cl; 	
public:
	PID();
	PID(int inTopLimit,int inBottomLimit,int inChannel);
	float getError();
	int topLimit;
	int bottomLimit;
	int getCommand();
	void setPID(bool inStatus,float inGoal, float inPose, int inMode);
	void setSigma();
	void setGains();
	void setGoal(float inGoal);
	void setPose(float inPose);
	void setMode(int inMode);
	int throttle_command();
	int yaw_command();
	int forward_command();
	int lateral_command();
	double getPercentError();
	void reset();
};
