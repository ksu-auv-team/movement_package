#include <iostream>	
#include <string>
#include <fstream>
#include <cmath>

using namespace std;

#define eps 0.01

//We will use a class and create PI objects that we can call inside of our main loop

class PI{
private: 
	double goal;
	double pose;
	int mode; 		
	int channel; //1-roll 2-pitch 3-throttle 4-yaw
	bool status;
	double error;
	long sigma;
	double kp;
	double ki;
	double dt;
	clock_t cl; 	
public:
	PI();
	PI(int inTopLimit,int inBottomLimit,int inChannel);
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
	int roll_command();
	int yaw_command();
	int throttle_command();
	int pitch_command();
	double getPercentError();
	void reset();
};
