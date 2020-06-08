
#ifndef ROBOTCONFIG_H_
#define ROBOTCONFIG_H_
const double PI = 3.1415926;
namespace HLRobot
{ 
		//double qlimP[6] = { 170 * PI / 180,120 * PI / 180,48 * PI / 180, 180 * PI / 180, 120 * PI / 180,360 * PI / 180 };
		//double qlimN[6] = { -170 * PI / 180,-120 * PI / 180,-228 * PI / 180, -180 * PI / 180, -120 * PI / 180,-360 * PI / 180 };

		void robotBackwardHJQ(const double* TransVector, bool* config, double* Tool, bool* Turns, double* theta);
		void robotForwardHJQ(const double* q, const double* Tool, double* TransVector, bool* config, bool* turns);

		//void SetRobotPos(double Posx, double Posy, double Posz, double Posyaw, double Pospitch, double Posroll, bool Posconfig[3]);
		void GetJointAngles(double* EMatrix, double* angle);
}

#endif
