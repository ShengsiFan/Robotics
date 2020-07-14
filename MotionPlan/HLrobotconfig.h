
#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
const double PI = 3.1415926;
namespace HLRobot
{
	void robotBackwardHJQ(const double* TransVector, bool* config, double* Tool, bool* Turns, double* theta);	//逆解	
	void robotForwardHJQ(const double* q, const double* Tool, double* TransVector, bool* config, bool* turns);	//正解
	void GetJointAngles(double* EMatrix, double* angle);	//简化逆解
}

#endif
