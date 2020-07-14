#include <iostream>
#include "HLrobotconfig.h"
#include "MotionPlan.h"

using namespace std;

int main()
{  
    /************* 使用示例 *************/
    //起点终点
    PosStruct startPoint, endPoint;

    //规划
    CHLMotionPlan trajectory;

    trajectory.Init(0.001, 3);      //先初始化

    /* 再调用规划函数 */
    trajectory.JointSpacePlan(startPoint, endPoint);     //关节空间
    trajectory.CartesianLinePlan(startPoint, endPoint);  //笛卡尔空间直线规划
    trajectory.CirclePlan(startPoint, endPoint);

    return 0;
}