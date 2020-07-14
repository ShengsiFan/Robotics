#ifndef MOTIONPLAN_H_
#define MOTIONPLAN_H_
#include <vector>

using namespace std;

typedef struct
{
	double x;				// x坐标，单位mm
	double y;				// y坐标，单位mm
	double z;				// z坐标，单位mm
	double yaw;				// yaw坐标，单位度
	double pitch;			// pitch坐标，单位度
	double roll;			// roll坐标，单位度
}PosStruct;

class CHLMotionPlan
{
private:
	double mJointAngleBegin[6];					//起始点位的关节角度,单位度
	double mJointAngleEnd[6];					//结束点位的关节角度，单位度
	double mStartMatrixData[16];				//起始点位的转换矩阵数组
	double mEndMatrixData[16];					//结束点位的转换矩阵数组
	double mSampleTime;							//采样点位，单位S
	double mVel;								//角速度，单位°/s
	double mAcc;								//加速度，单位°/s/s
	double mDec;								//减速度，单位°/ s / s
	double LVel;                                //线速度 ，单位m/s
	double LAcc;                                //线加速度，单位m/s/s
	bool   mConfig[3];							//机器人姿态

public:
	CHLMotionPlan();
	virtual ~CHLMotionPlan();
	void Init(double sampleTime, int VLevel);		//初始化
	void JointSpacePlan(PosStruct startPos, PosStruct endPos);		//关节空间规划											 
	void CartesianLinePlan(PosStruct startPos, PosStruct endPos);	    //笛卡尔空间规划
	void CirclePlan(PosStruct startPos, PosStruct endPos);   //码垛规划
};

#endif