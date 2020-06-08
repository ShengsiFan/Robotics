#include <iostream>
#include <fstream>
#include "MotionPlan.h"
#include "HLrobotconfig.h"
#include <algorithm>
#include <Windows.h>
#include "Eigen/Dense"

using namespace std;
using namespace HLRobot;
using namespace Eigen;

/********************************************************************
ABSTRACT:	构造函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/

CHLMotionPlan::CHLMotionPlan()
{
	for (int i = 0; i < 6; i++)
	{
		mJointAngleBegin[i] = 0;
		mJointAngleEnd[i] = 0;
	}

	for (int i = 0; i < 16; i++)
	{
		mStartMatrixData[i] = 0;
		mEndMatrixData[i] = 0;
	}
	for (int i = 0; i < 3; i++)
	{
		mConfig[i] = 0;
	}

	mSampleTime = 0.001;
	mVel = 0;
	mAcc = 0;
	mDec = 0;
}

/********************************************************************
ABSTRACT:	析构函数

INPUTS:		<none>

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
CHLMotionPlan::~CHLMotionPlan()
{

}

/********************************************************************
ABSTRACT:	设置采样时间

INPUTS:		sampleTime			采样时间，单位S

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetSampleTime(double sampleTime)
{
	if (sampleTime < 0.001)
	{
		mSampleTime = 0.001;
	}
	else
	{
		mSampleTime = sampleTime;
	}
}

/********************************************************************
ABSTRACT:	设置运动参数

INPUTS:		vel			速度，单位m/s
			acc			加速度，单位m/s/s
			dec			减速度，单位m / s / s

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetProfile(double vel, double acc, double dec)
{
	mVel = vel;
	mAcc = acc;
	mDec = dec;
}

void CHLMotionPlan::SetLine(double v, double a)
{
	LVel = v;
	LAcc = a;
}

/********************************************************************
ABSTRACT:	设置规划的起始单位和结束点位

INPUTS:		startPos			起始点位笛卡尔坐标
			endPos				结束点位笛卡尔坐标

OUTPUTS:	<none>

RETURN:		<none>
***********************************************************************/
void CHLMotionPlan::SetPlanPoints(PosStruct startPos, PosStruct endPos)
{
	double startAngle[3], endAngle[3];

	startAngle[0] = startPos.yaw   * PI / 180;
	startAngle[1] = startPos.pitch * PI / 180;
	startAngle[2] = startPos.roll  * PI / 180;

	endAngle[0] = endPos.yaw   * PI / 180;
	endAngle[1] = endPos.pitch * PI / 180;
	endAngle[2] = endPos.roll  * PI / 180;

	mStartMatrixData[0] =  cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[2] =  cos(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[3] = startPos.x / 1000;

	mStartMatrixData[4] =  sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
	mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
	mStartMatrixData[6] =  sin(startAngle[0]) * sin(startAngle[1]);
	mStartMatrixData[7] = startPos.y / 1000;

	mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
	mStartMatrixData[9] =  sin(startAngle[1]) * sin(startAngle[2]);
	mStartMatrixData[10] = cos(startAngle[1]);
	mStartMatrixData[11] = startPos.z / 1000;

	mStartMatrixData[12] = 0;
	mStartMatrixData[13] = 0;
	mStartMatrixData[14] = 0;
	mStartMatrixData[15] = 1;

	mEndMatrixData[0] =  cos(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) - sin(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[1] = -cos(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) - sin(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[2] =  cos(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[3] = endPos.x / 1000;

	mEndMatrixData[4] =  sin(endAngle[0]) * cos(endAngle[1]) * cos(endAngle[2]) + cos(endAngle[0]) * sin(endAngle[2]);
	mEndMatrixData[5] = -sin(endAngle[0]) * cos(endAngle[1]) * sin(endAngle[2]) + cos(endAngle[0]) * cos(endAngle[2]);
	mEndMatrixData[6] =  sin(endAngle[0]) * sin(endAngle[1]);
	mEndMatrixData[7] = endPos.y / 1000;

	mEndMatrixData[8] = -sin(endAngle[1]) * cos(endAngle[2]);
	mEndMatrixData[9] =  sin(endAngle[1]) * sin(endAngle[2]);
	mEndMatrixData[10] = cos(endAngle[1]);
	mEndMatrixData[11] = endPos.z / 1000;

	mEndMatrixData[12] = 0;
	mEndMatrixData[13] = 0;
	mEndMatrixData[14] = 0;
	mEndMatrixData[15] = 1;

	double angle[6] = { 0 };
	//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
	HLRobot::GetJointAngles(mStartMatrixData, angle);

	mJointAngleBegin[0] = angle[0];
	mJointAngleBegin[1] = angle[1];
	mJointAngleBegin[2] = angle[2];
	mJointAngleBegin[3] = angle[3];
	mJointAngleBegin[4] = angle[4];
	mJointAngleBegin[5] = angle[5];

	//HLRobot::SetRobotEndPos(endPos.x, endPos.y, endPos.z, endPos.yaw, endPos.pitch, endPos.roll, endPos.config);
	HLRobot::GetJointAngles(mEndMatrixData, angle);
	mJointAngleEnd[0] = angle[0];
	mJointAngleEnd[1] = angle[1];
	mJointAngleEnd[2] = angle[2];
	mJointAngleEnd[3] = angle[3];
	mJointAngleEnd[4] = angle[4];
	mJointAngleEnd[5] = angle[5];

}

/********************************************************************
ABSTRACT:	运动轨迹规划部分

INPUTS:		pos						二维位置向量

OUTPUTS:	pos						二维位置向量（第一维：位置个数，第二维：每个轴的关节角度，单位度）

RETURN:		<none>
***********************************************************************/

/******
 * 参考步骤
 * 步骤1：创建文件并写入初始角度
 * 步骤2：计算每个轴旋转的角度
 * 步骤3：计算每个轴移动到终止点所需要时间
 * 步骤4：根据采样时间计算离散点位数
 * 步骤5：根据离散点位数得到每刻的关节角数值
 * 步骤6：将每时刻的关节角数值写入到文件
 */
void CHLMotionPlan::GetPlanPoints()
{
	/* 步骤1：创建文件并写入初始角度 */
	ofstream outfile;               			//创建文件
	outfile.open("data.txt");
	outfile << mJointAngleBegin[0] << "  "
		    << mJointAngleBegin[1] << "  "
		    << mJointAngleBegin[2] << "  "
	    	<< mJointAngleBegin[3] << "  "
		    << mJointAngleBegin[4] << "  "
		    << mJointAngleBegin[5] << "  ";
	outfile << endl;                           //保存初始的时间、六个关节角度

	double Tour[6] = { 0 };     //每个关节需要走的角度
	double Time[6] = { 0 };     //每个关节走的时间 
	int PointNum[6] = { 0 };     //离散点位数
	double Acc[6] = { 0 };      //各关节的加速度方向
	for (int i = 0; i < 6; i++)
	{
		/* 步骤2：计算每个轴旋转的角度 */
		
		Tour[i] = fabs(mJointAngleBegin[i] - mJointAngleEnd[i]);

		if (mJointAngleBegin[i] > mJointAngleEnd[i])
		{
			Acc[i] = -mAcc;
		}
		else
		{
			Acc[i] = mAcc;
		}

		/* 步骤3：计算每个轴移动到终止点所需要时间 */
		//转角太小则退化为三角形
		if (Tour[i] <= mVel * mVel/mAcc)
		{
			Time[i] = 2 * sqrt(Tour[i] / mAcc);
		}
		//转角足够则梯形规划
		else
		{
			Time[i] = 2 * mVel / mAcc + (Tour[i] / mVel - mVel / mAcc);
		}

		/* 步骤4：根据采样时间计算离散点位数 */
		PointNum[i] = Time[i] / mSampleTime;
	}

	for (int k = 0; k < 6; k++)
	{
		cout << mJointAngleBegin[k] << "   ";
	}
	cout << endl;
	for (int k = 0; k < 6; k++)
	{
		cout << mJointAngleEnd[k] << "   ";
	}
	cout << endl;
	for (int k = 0; k < 6; k++)
	{
		cout << PointNum[k] << "   ";
	}


	/* 找六个关节的点数最大者 */
	int MaxNum = 0;
	for (int i = 0; i < 6; i++)
	{
		if (PointNum[i] > MaxNum)
			MaxNum = PointNum[i];
	}


	/************************ 计算各关节角 ***********************/
	
	double TempPos[6] = { 0 };  //每个轴当前关节角
	for (int i = 0; i < MaxNum; i++)
	{//时间
		//轴
		for (int j = 0; j < 6; j++)
		{

			//若转角太小，则按三角形规划
			if (Tour[j] <= mVel * mVel / mAcc)
			{
				if (i < 1000 * sqrt(Tour[j] / mAcc))
				{
					//0.5at^2
					TempPos[j] = mJointAngleBegin[j] + 0.5 * Acc[j] * i * i * 0.001 * 0.001;
				}
				else if(i >= 1000 * sqrt(Tour[j] / mAcc) && i < PointNum[j])
				{
					TempPos[j] = mJointAngleEnd[j] - 0.5 * Acc[j] * (PointNum[j] - i) * (PointNum[j] - i) * 0.001 * 0.001;
				}
				else
				{
					TempPos[j] = TempPos[j];
				}
			}
			//转角足够，则用梯形规划
			else
			{
				if (i < 1000 * mVel / mAcc)
				{
					TempPos[j] = mJointAngleBegin[j] + 0.5 * Acc[j] * i * i * 0.001 * 0.001;
				}
				else if (i >= 1000 * mVel / mAcc && i < PointNum[j] - 1000 * mVel / mAcc)
				{
					TempPos[j] = mJointAngleBegin[j] + Acc[j] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
				}
				else if (i >= PointNum[j] - 1000 * mVel / mAcc && i < PointNum[j])
				{
					TempPos[j] = mJointAngleEnd[j] - 0.5 * Acc[j] * (PointNum[j] - i) * (PointNum[j] - i) * 0.001 * 0.001;
				}
				else
				{
					TempPos[j] = TempPos[j];
				}
			}
		}

		//输出到文件
		outfile << TempPos[0] << "  "
				<< TempPos[1] << "  "
				<< TempPos[2] << "  "
				<< TempPos[3] << "  "
				<< TempPos[4] << "  "
				<< TempPos[5] << "  ";
		outfile << endl;
	}
	outfile.close();
}


void CHLMotionPlan::Cartesian(PosStruct startPos, PosStruct endPos)
{
	ofstream outfile;               			//创建文件
	outfile.open("data2.txt");
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	outfile << endl;                           //保存初始的时间、六个关节角度

	double Tour[6] = { 0 };    //每个笛卡尔空间维度的增量
	double Acc[6] = { 0 };     //每个笛卡尔空间维度的加速度
	//double Vel[6] = { 0 };     //各维的速度
	double Time[6] = { 0 };    //各维的时间
	int PointNum[6] = { 0 };   //各维的点数

	/* 计算每个维度的增量 */

	Tour[0] = fabs(startPos.x - endPos.x);
	Tour[1] = fabs(startPos.y - endPos.y);
	Tour[2] = fabs(startPos.z - endPos.z);
	Tour[3] = fabs(startPos.yaw - endPos.yaw);
	Tour[4] = fabs(startPos.pitch - endPos.pitch);
	Tour[5] = fabs(startPos.roll - endPos.roll);

	/* 笛卡尔空间各维加速度赋值 */
	{
		if (startPos.x > endPos.x)
		{
			Acc[0] = -LAcc;
		}
		else
		{
			Acc[0] = LAcc;
		}

		if (startPos.y > endPos.y)
		{
			Acc[1] = -LAcc;
		}
		else
		{
			Acc[1] = LAcc;
		}

		if (startPos.z > endPos.z)
		{
			Acc[2] = -LAcc;
		}
		else
		{
			Acc[2] = LAcc;
		}

		if (startPos.yaw > endPos.yaw)
		{
			Acc[3] = -mAcc;
		}
		else
		{
			Acc[3] = mAcc;
		}
		if (startPos.pitch > endPos.pitch)
		{
			Acc[4] = -mAcc;
		}
		else
		{
			Acc[4] = mAcc;
		}
		if (startPos.roll > endPos.roll)
		{
			Acc[5] = -mAcc;
		}
		/*else
		{
			Acc[5] = mAcc;
		}*/
	}

	/* 各维速度赋值 */
	//for (int i = 0; i < 3; i++)
	//{
	//	Vel[i] = LVel;
	//	Vel[i + 3] = mVel;
	//}

	/* 计算各维的时间 */
	{
		//x,y,z
		for (int i = 0; i < 3; i++)
		{
			if (Tour[i]/1000 < LVel * LVel / LAcc)
			{
				Time[i] = 2 * sqrt(Tour[i]*0.001 / LAcc);
			}
			else
			{
				Time[i] = 2 * LVel / LAcc + (Tour[i]*0.001/LVel - LVel * LVel / LAcc);
			}
		}
		
		//yaw,pitch,roll
		for (int i = 3; i < 6; i++)
		{
			if (Tour[i] < mVel * mVel / mAcc)
			{
				Time[i] = 2 * sqrt(Tour[i] / mAcc);
			}
			else
			{
				Time[i] = 2 * mVel / mAcc + (Tour[i] - mVel * mVel / mAcc) / mVel;
			}
		}
	}

	
	/* 计算各维的点数 */
	for (int i = 0; i < 6; i++)
	{
		PointNum[i] = Time[i] / mSampleTime;
	}


	
	/* 找六个维度的点数最大者 */
	int MaxNum = 0;
	for (int i = 0; i < 6; i++)
	{
		if (PointNum[i] > MaxNum)
			MaxNum = PointNum[i];
	}

	//cout << endl << Acc[5];

	/* 各维的规划 */
	double TempPos[6] = { 0 };
	double Decar[6]  = { 0 };  

	for (int i = 0; i < MaxNum; i++)
	{//时间
		cout << i << endl;
		//x			
		if (Tour[0]/1000 <= LVel * LVel / LAcc)
		{
			if (i < 1000 * sqrt(Tour[0]*0.001 / LAcc))
			{
				//0.5at^2
				Decar[0] = startPos.x/1000 + 0.5 * Acc[0] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[0]*0.001 / LAcc) && i < PointNum[0])
			{
				Decar[0] = endPos.x/1000 - 0.5 * Acc[0] * (PointNum[0] - i) * (PointNum[0] - i) * 0.001 * 0.001;
			}
			else
			{
				Decar[0] = Decar[0];
			}
		}
		else
		{
			if (i < 1000 * LVel / LAcc)
			{
				Decar[0] = startPos.x/1000 + 0.5 * Acc[0] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * LVel / LAcc && i < PointNum[0] - 1000 * LVel / LAcc)
			{
				Decar[0] = startPos.x/1000 + Acc[0] * (LVel / LAcc) * (i * 0.001 - 0.5 * LVel / LAcc);
			}
			else if (i >= PointNum[0] - 1000 * LVel / LAcc && i < PointNum[0])
			{
				Decar[0] = endPos.x/1000 - 0.5 * Acc[0] * (PointNum[0] - i) * (PointNum[0] - i) * 0.001 * 0.001;
			}
		}
		//y			
		if (Tour[1]/1000 <= LVel * LVel / LAcc)
		{
			if (i < 1000 * sqrt(Tour[1]*0.001 / LAcc))
			{
				//0.5at^2
				Decar[1] = startPos.y/1000 + 0.5 * Acc[1] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[1]*0.001 / LAcc) && i < PointNum[1])
			{
				Decar[1] = endPos.y/1000 - 0.5 * Acc[1] * (PointNum[1] - i) * (PointNum[1] - i) * 0.001 * 0.001;
			}
		}
		else
		{
			if (i < 1000 * LVel / LAcc)
			{
				Decar[1] = startPos.y/1000 + 0.5 * Acc[1] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * LVel / LAcc && i < PointNum[1] - 1000 * LVel / LAcc)
			{
				Decar[1] = startPos.y/1000 + Acc[1] * (LVel / LAcc) * (i * 0.001 - 0.5 * LVel / LAcc);
			}
			else if (i >= PointNum[1] - 1000 * LVel / LAcc && i < PointNum[1])
			{
				Decar[1] = endPos.y/1000 - 0.5 * Acc[1] * (PointNum[1] - i) * (PointNum[1] - i) * 0.001 * 0.001;
			}
		}
		//z
		if (Tour[2]/1000 <= LVel * LVel / LAcc)
		{
			if (i < 1000 * sqrt(Tour[2]*0.001 / LAcc))
			{
				//0.5at^2
				Decar[2] = startPos.z/1000 + 0.5 * Acc[2] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[2]*0.001 / LAcc) && i < PointNum[2])
			{
				Decar[2] = endPos.z/1000 - 0.5 * Acc[2] * (PointNum[2] - i) * (PointNum[2] - i) * 0.001 * 0.001;
			}
		}
		else
		{
			if (i < 1000 * LVel / LAcc)
			{
				Decar[2] = startPos.z/1000 + 0.5 * Acc[2] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * LVel / LAcc && i < PointNum[2] - 1000 * LVel / LAcc)
			{
				Decar[2] = startPos.z/1000 + Acc[2] * (LVel / LAcc) * (i * 0.001 - 0.5 * LVel / LAcc);
			}
			else if (i >= PointNum[2] - 1000 * LVel / LAcc && i < PointNum[2])
			{
				Decar[2] = endPos.z/1000 - 0.5 * Acc[2] * (PointNum[2] - i) * (PointNum[2] - i) * 0.001 * 0.001;
			}
		}
		//yaw
		if (Tour[3] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(Tour[3] / mAcc))
			{
				//0.5at^2
				Decar[3] = startPos.yaw + 0.5 * Acc[3] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[3] / mAcc) && i < PointNum[3])
			{
				Decar[3] = endPos.yaw - 0.5 * Acc[3] * (PointNum[3] - i) * (PointNum[3] - i) * 0.001 * 0.001;
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				Decar[3] = startPos.yaw + 0.5 * Acc[3] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < PointNum[3] - 1000 * mVel / mAcc)
			{
				Decar[3] = startPos.yaw + Acc[3] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= PointNum[3] - 1000 * mVel / mAcc && i < PointNum[3])
			{
				Decar[3] = endPos.yaw - 0.5 * Acc[3] * (PointNum[3] - i) * (PointNum[3] - i) * 0.001 * 0.001;
			}
		}
		//pitch
		if (Tour[4] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(Tour[4] / mAcc))
			{
				//0.5at^2
				Decar[4] = startPos.pitch + 0.5 * Acc[4] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[4] / mAcc) && i < PointNum[4])
			{
				Decar[4] = endPos.pitch - 0.5 * Acc[4] * (PointNum[4] - i) * (PointNum[4] - i) * 0.001 * 0.001;
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				Decar[4] = startPos.pitch + 0.5 * Acc[4] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < PointNum[4] - 1000 * mVel / mAcc)
			{
				Decar[4] = startPos.pitch + Acc[4] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= PointNum[4] - 1000 * mVel / mAcc && i < PointNum[4])
			{
				Decar[4] = endPos.pitch - 0.5 * Acc[4] * (PointNum[4] - i) * (PointNum[4] - i) * 0.001 * 0.001;
			}
		}
		//roll
		if (Tour[5] <= mVel * mVel / mAcc)
		{
			if (i < 1000 * sqrt(Tour[5] / mAcc))
			{
				//0.5at^2
				Decar[5] = startPos.roll + 0.5 * Acc[5] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * sqrt(Tour[5] / mAcc) && i < PointNum[5])
			{
				Decar[5] = endPos.roll - 0.5 * Acc[5] * (PointNum[5] - i) * (PointNum[5] - i) * 0.001 * 0.001;
			}
		}
		else
		{
			if (i < 1000 * mVel / mAcc)
			{
				cout << "  1 " << endl;
				Decar[5] = startPos.roll + 0.5 * Acc[5] * i * i * 0.001 * 0.001;
			}
			else if (i >= 1000 * mVel / mAcc && i < PointNum[5] - 1000 * mVel / mAcc)
			{
				cout << "  2 " << endl;
				Decar[5] = startPos.roll + Acc[5] * (mVel / mAcc) * (i * 0.001 - 0.5 * mVel / mAcc);
			}
			else if (i >= PointNum[5] - 1000 * mVel / mAcc && i < PointNum[5])
			{
				cout << "  3 " << endl;
				Decar[5] = endPos.roll - 0.5 * Acc[5] * (PointNum[5] - i) * (PointNum[5] - i) * 0.001 * 0.001;
			}
			else
			{
				Decar[5] = Decar[5];
			}
		}

		double startAngle[3], endAngle[3];

		startAngle[0] = Decar[3] * PI / 180;
		startAngle[1] = Decar[4] * PI / 180;
		startAngle[2] = Decar[5] * PI / 180;


		mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);
		mStartMatrixData[3] = Decar[0];

		mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);
		mStartMatrixData[7] = Decar[1];

		mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
		mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
		mStartMatrixData[10] = cos(startAngle[1]);
		mStartMatrixData[11] = Decar[2];

		mStartMatrixData[12] = 0;
		mStartMatrixData[13] = 0;
		mStartMatrixData[14] = 0;
		mStartMatrixData[15] = 1;

		double angle[6] = { 0 };
		//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
		HLRobot::GetJointAngles(mStartMatrixData, angle);

		//输出到文件
		outfile << angle[0] << "  "
				<< angle[1] << "  "
				<< angle[2] << "  "
				<< angle[3] << "  "
				<< angle[4] << "  "
				<< angle[5] << "  ";
		outfile << endl;
	}





	
}

void CHLMotionPlan::CartesianLine(PosStruct startPos, PosStruct endPos)
{
	ofstream outfile;               			//创建文件
	outfile.open("data2.txt");
	outfile << mJointAngleBegin[0] << "  "
			<< mJointAngleBegin[1] << "  "
			<< mJointAngleBegin[2] << "  "
			<< mJointAngleBegin[3] << "  "
			<< mJointAngleBegin[4] << "  "
			<< mJointAngleBegin[5] << "  ";
	outfile << endl;

	double Tour[3] = { 0 };     //x,y,z的增量向量，单位m
	double direction[3] = { 0 };//方向单位向量
	
	int PointNum = 0;          //点数

	Tour[0] = (endPos.x - startPos.x) / 1000;
	Tour[1] = (endPos.y - startPos.y) / 1000;
	Tour[2] = (endPos.z - startPos.z) / 1000;

	double distance = sqrt(Tour[0] * Tour[0] + Tour[1] * Tour[1] + Tour[2] * Tour[2]);   //运动距离
	double Time = 0;           //时间
	double delta = 0;          //每一小段的增量

	direction[0] = Tour[0] / distance;
	direction[1] = Tour[1] / distance;
	direction[2] = Tour[2] / distance;

	/* 计算时间 */
	if (distance < LVel * LVel / LAcc)
	{
		Time = 2 * sqrt(LVel / LAcc);
	}
	else
	{
		Time = 2 * LVel / LAcc + (distance / LVel -  LVel / LAcc);
	}

	PointNum = Time / mSampleTime;

	cout << endl << PointNum;

	for (int i = 0; i < PointNum; i++)
	{
		cout << i << endl;
		if (distance <= LVel * LVel / LAcc)
		{
			if (i < 1000 * sqrt(distance / LAcc))
			{
				//0.5at^2
				delta = 0.5 * LAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * direction[2];
			}
			else if (i >= 1000 * sqrt(distance / LAcc) && i < PointNum)
			{
				delta = distance - 0.5 * LAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * direction[2];
			}
		}
		else
		{
			if (i < 1000 * LVel / LAcc)
			{
				delta = 0.5 * LAcc * i * i * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * direction[2];
			}
			else if (i >= 1000 * LVel / LAcc && i < PointNum - 1000 * LVel / LAcc)
			{
				delta = LAcc * (LVel / LAcc) * (i * 0.001 - 0.5 * LVel / LAcc);
				mStartMatrixData[3] = startPos.x / 1000 + delta * direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * direction[2];
			}
			else if (i >= PointNum - 1000 * LVel / LAcc && i < PointNum)
			{
				delta = distance - 0.5 * LAcc * (PointNum - i) * (PointNum - i) * 0.001 * 0.001;
				mStartMatrixData[3] = startPos.x / 1000 + delta * direction[0];
				mStartMatrixData[7] = startPos.y / 1000 + delta * direction[1];
				mStartMatrixData[11] = startPos.z / 1000 + delta * direction[2];
			}
		}
		

		double startAngle[3], endAngle[3];

		startAngle[0] = startPos.yaw * PI / 180;
		startAngle[1] = startPos.pitch * PI / 180;
		startAngle[2] = startPos.roll * PI / 180;


		mStartMatrixData[0] = cos(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) - sin(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[1] = -cos(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) - sin(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[2] = cos(startAngle[0]) * sin(startAngle[1]);

		mStartMatrixData[4] = sin(startAngle[0]) * cos(startAngle[1]) * cos(startAngle[2]) + cos(startAngle[0]) * sin(startAngle[2]);
		mStartMatrixData[5] = -sin(startAngle[0]) * cos(startAngle[1]) * sin(startAngle[2]) + cos(startAngle[0]) * cos(startAngle[2]);
		mStartMatrixData[6] = sin(startAngle[0]) * sin(startAngle[1]);

		mStartMatrixData[8] = -sin(startAngle[1]) * cos(startAngle[2]);
		mStartMatrixData[9] = sin(startAngle[1]) * sin(startAngle[2]);
		mStartMatrixData[10] = cos(startAngle[1]);

		mStartMatrixData[12] = 0;
		mStartMatrixData[13] = 0;
		mStartMatrixData[14] = 0;
		mStartMatrixData[15] = 1;

		double angle[6] = { 0 };
		//HLRobot::SetRobotPos(startPos.x, startPos.y, startPos.z, startPos.yaw, startPos.pitch, startPos.roll, startPos.config);
		HLRobot::GetJointAngles(mStartMatrixData, angle);

		outfile << angle[0] << "  "
				<< angle[1] << "  "
				<< angle[2] << "  "
				<< angle[3] << "  "
				<< angle[4] << "  "
				<< angle[5] << "  ";
		outfile << endl;
	}
	outfile.close();

}