#include <iostream>
#include "HLrobotconfig.h"
#include "MotionPlan.h"

using namespace std;

/****
 * 实验四: 轨迹规划
 * 要 求：使用C/C++完成梯型速度规划，生成data.txt文件
 * 规划类型：关节空间、笛卡尔空间（直线）
 * 点位数量：起始点和终止点
 * 给定条件：Vel，Acc，Dec
 *
 */

int main()
{   //起始点
    PosStruct Start,pos1,pos2,pos3;
    Start.x = 509.892; Start.y = 99.918; Start.z = 796.366;
    Start.yaw = -178.631; Start.pitch = 179.920; Start.roll = 1.378;

    pos1.x = 509.892; pos1.y = -100.082;pos1.z = 796.366;
    pos1.yaw = -178.631; pos1.pitch = 179.9205; pos1.roll = 1.378;

    pos2.x = 309.892; pos2.y = -100.082; pos2.z = 796.366;
    pos2.yaw = -178.631; pos2.pitch = 179.920; pos2.roll = 1.378;

    pos3.x = 309.892; pos3.y = 99.918; pos3.z = 796.366;
    pos3.yaw = -178.631; pos3.pitch = 179.920; pos3.roll = 1.378;

    //终止点
    PosStruct End;
    End.x = 441.78; End.y = 86.9; End.z = 877.035;
    End.yaw = 103.388; End.pitch = 170.195; End.roll = -90.995;

    //梯型速度规划
    CHLMotionPlan trajectory1;
    trajectory1.SetProfile(10, 10, 10);    //vel °/s， acc °/s.s, dec °/s.s
    trajectory1.SetLine(0.1, 0.1);
    trajectory1.SetSampleTime(0.001);      //s

    

    //trajectory1.GetPlanPoints();
    trajectory1.SetPlanPoints(pos3, Start);
    trajectory1.CartesianLine(pos3, Start);

    return 0;
}