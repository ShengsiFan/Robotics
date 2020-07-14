# MotionPlan

## Brief

一种六轴操作臂的正逆运动学及轨迹规划，机器人外观及结构示意图如下

<img src="C:\Users\fanss\AppData\Roaming\Typora\typora-user-images\image-20200714205430597.png" alt="image-20200714205430597" style="zoom:50%;" />

<img src="C:\Users\fanss\AppData\Roaming\Typora\typora-user-images\image-20200714205517421.png" alt="image-20200714205517421" style="zoom:50%;" />

目前可实现的轨迹种类：

1. 关节空间梯形速度规划
2. 笛卡尔空间直线轨迹梯形速度规划
3. 笛卡尔空间起点终点等高、且竖直的半圆弧形轨迹梯形速度规划

## About

工程环境：VS2019

编程语言：C++

附加依赖：eigen矩阵库，每次改变路径需重新配置

作用文件：MotionPlan.cpp，MotionPlan.h

使用：先创建一个CHLMotionPlan类的对象，调用Init()进行初始化，再调用三种轨迹规划函数

输出：data.txt	关节空间速度规划点集
