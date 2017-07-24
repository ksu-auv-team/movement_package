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
		float _topLimit;
		float _bottomLimit;
		float _center;
		clock_t _cl;

		float _goal;
		float _pose;
		bool _status;
		float _error;
		float _lastError;
		float _sigma;
		float _command;

		double _kp;
		double _ki;
		double _kd;

		double _dt;

		void UpdateSigma();

	public:
		float GetCommand();
		void UpdatePID(bool inStatus,float inGoal, float inPose);
		void SetGains(int kp, int ki, int kd);
		void SetGoal(float inGoal);
		void SetPose(float inPose);
		float GetError();
		double GetPercentError();
		void Reset();

	PID();
	PID(int inTopLimit,int inBottomLimit);
	PID(int inTopLimit,int inBottomLimit, int kp, int ki, int kd);

};
