/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	Description: User Module for CyberCruise							█
█	作者: 杨辰兮 & ChatGPT												█
█	联系方式: yangchenxi@sjtu.edu.cn										█
█	日期: 2023.02.13				    						█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	贴士:	您可以折叠 #pragma region 和	#pragma endregion 之间的代码		█
█	这可以使您获得一次性折叠完成的程序块而不是一个函数的能力					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	DLL接口部分，您可以跳过这部分不阅读									█
█	不要修改这个 #pragma region 中的任何代码!								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< 不要修改这个 region 中的任何代码!
#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"
#include "stdio.h"
#include <ostream>
#include <fstream>
#include<algorithm>

#include "class_Visualization.h"
using namespace std;
#define PI 3.141592653589793238462643383279

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void* pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo * modInfo)
{
	memset(modInfo, 0, 10 * sizeof(tModInfo));
	modInfo[0].name = "driver_cruise";	// name of the module (short).
	modInfo[0].desc = "user module for CyberCruise";	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId = 0;
	modInfo[0].index = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void* pt)
{
	tUserItf* itf = (tUserItf*)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

//Global variables for vehicle states
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm)
{
	for (int i = 0; i < 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	上下确界约束函数									 					█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Boundaries of control	
double constrain(double lowerBoundary, double upperBoundary, double input)
{
	if (input > upperBoundary)
		return upperBoundary;
	else if (input < lowerBoundary)
		return lowerBoundary;
	else
		return input;
}
#pragma endregion

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	赛道曲率半径计算函数													█
█	您需要理解它的功能，建议不要修改										█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Radius of curvature
//		Given three points ahead, outputs a struct circle.				
//		{radius:[1,1000], sign{-1:left,1:right}							
typedef struct Circle
{
	double r;
	int sign;
}circle;

circle getR(float x1, float y1, float x2, float y2, float x3, float y3)
{
	double a, b, c, d, e, f;
	double r, x, y;

	a = 2 * (x2 - x1);
	b = 2 * (y2 - y1);
	c = x2 * x2 + y2 * y2 - x1 * x1 - y1 * y1;
	d = 2 * (x3 - x2);
	e = 2 * (y3 - y2);
	f = x3 * x3 + y3 * y3 - x2 * x2 - y2 * y2;
	x = (b * f - e * c) / (b * d - e * a);
	y = (d * c - a * f) / (b * d - e * a);
	r = sqrt((x - x1) * (x - x1) + (y - y1) * (y - y1));
	x = constrain(-1000.0, 1000.0, x);
	y = constrain(-1000.0, 1000.0, y);
	r = constrain(1.0, 500.0, r);
	int sign = (x > 0) ? 1 : -1;
	return { r,sign };
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	以下是核心控制程序													█
█	主要输入: _midline, _speed											█
█	次要输入: _yaw, _yawrate, _acc, _width, _rpm,	_gearbox			█
█	主要输出: *cmdAcc, *cmdBrake, *cmdSteer  							█
█	次要输出: *cmdGear 【本样例中已实现】									█
█	详细信息请参见用户手册												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/

//基于OpenCV的可视化工具，详情请见文档
cls_VISUAL cls_visual;

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	手动换挡程序															█
█	可以不用看懂，建议不要修改，除非您是学(Juan)霸(Wang) :P					█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
#pragma region <<< Manual Gear
const float fGearShift[2][7] = //0 for downshift, 1 for upshift
{
	0,92,128,167,208,233,255,
	0,105,145,182,224,249,272
};
void updateGear(int* cmdGear)
{

	if (_speed > fGearShift[1][_gearbox] && _gearbox < 7) //upshift
	{
		*cmdGear = _gearbox + 1;
	}
	else if (_speed < fGearShift[0][_gearbox - 1] && _gearbox > 1) //downshift
	{
		*cmdGear = _gearbox - 1;
	}
	else
	{
		*cmdGear = _gearbox;
	}
}
#pragma endregion >>>

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	PID控制器，由ChatGPT生成												█
█	可选择性修改，需要完全理解												█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
class PIDController
{
private:
	double kp, ki, kd;		// PID控制器的参数
	double targetValue;		// 目标值
	double lastError;		// 上一次误差值
	double errorIntegral;	// 误差积分值

public:
	void initial(double p, double i, double d, double target)
	{
		kp = p;
		ki = i;
		kd = d;
		targetValue = target;
		lastError = 0;
		errorIntegral = 0;
	}

	double calculate(double input)
	{
		double error = targetValue - input;
		double derivative = error - lastError;
		errorIntegral += error;
		lastError = error;
		return kp * error + ki * errorIntegral + kd * derivative;
	}
};

/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
█	车辆控制主程序，由ChatGPT自动生成助教完善								█
█	样例代码仅供参考，请在下方设计实现您的算法								█
\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
PIDController speedController;	//速度PID控制
PIDController angleController;	//舵角PID控制
double lastTargetSpeed = 999.0;	//上一帧目标速度
bool isFirstFrame = true;		//第一帧标志
bool isCementRoad = true;      //mark：增加路面判断
int timecounter = 0;           //mark：计时器用来判断路面
bool flag = false;             //mark：
double length(float x1, float y1, float x2, float y2) {   //mark:
	return sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2));
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	timecounter++;
	if (timecounter < 68)
	{
		*cmdAcc = 1.0;
		*cmdBrake = 0.0;
	}
	if (timecounter == 68 && _speed < 40)
	{
		isCementRoad = false;             //mark:若初始加速度很低，判断为非城市路面
		flag = true;
	}
	else if (timecounter == 68 && _speed >= 40)
	{
		isCementRoad = true;
		flag = true;
	}
	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	舵角控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetAngleError = 0.0; //目标误差
	double currentAngleError = atan2(_midline[4][0], _midline[4][1]); //当前误差
                                                                      //mark:拉近舵角判断点位，灵敏度提升
	//第一帧初始化舵角控制参数，清空积分器和微分器，因为控制目标为恒零，所以只需要初始化一次
	if (isFirstFrame)
	{
		isFirstFrame = false;
		angleController.initial(6.0, 0, 12.0, targetAngleError);
	}
	if (timecounter == 69 && !isCementRoad)       //mark：沙地路面舵角pid参数
	{
		angleController.initial(6.0, 0.0013, 10.1, targetAngleError);
	}
	else if (timecounter == 69 && isCementRoad) //mark:水泥路面舵角pid参数
	{
		angleController.initial(8.0, 0, 12.0, targetAngleError);	
	}


	//舵角PID控制
	*cmdSteer = constrain(-1.0, 1.0, angleController.calculate(currentAngleError));
	double lfAngleError = -atan2(
		_midline[5][0] - _midline[4][0],
		_midline[5][1] - _midline[4][1]
	);

	double lfDistance = \
		_midline[0][0] < 0 ? length(_midline[0][0], _midline[0][1], 0, 0) :
		-length(_midline[0][0], _midline[0][1], 0, 0);

	double lfStanley_K;
	circle Curve4Stanley = getR(
		_midline[0][0], _midline[0][1],
		_midline[5][0], _midline[5][1],
		_midline[10][0], _midline[10][1]
	);

	if (Curve4Stanley.r < 50)
		lfStanley_K = 200;
	else if (Curve4Stanley.r < 100)
		lfStanley_K = 45;
	else if (Curve4Stanley.r < 200)
		lfStanley_K = 25;
	else
		lfStanley_K = 10;

	double lfDistanceError = atan(lfStanley_K * lfDistance / _speed);
	if (!isCementRoad)
	{
	    *cmdSteer = (constrain(-1, 1, lfAngleError + lfDistanceError) * 0.1 + constrain(-1.0, 1.0, angleController.calculate(currentAngleError)) * 0.9);
	}
	else
	{
		*cmdSteer = (constrain(-1, 1, lfAngleError + lfDistanceError) * 0.3 + constrain(-1.0, 1.0, angleController.calculate(currentAngleError)) * 0.7);
	}
	

	/*▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼▼*\
	█	速度控制																█
	\*▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲▲*/
	double targetSpeed;  //目标车速
	double currentSpeed = _speed;	//当前误差

	//计算前方是直道还是弯道
	circle myCurve;
	float minCruve = 500.0;
	int fStep;
	int pin;               //mark:应该有什么作用，不确定，再看看
	if (0.4 * _speed <= 35) pin = 35;
	else if (0.5 * _speed >= 150) pin = 150;//mark:高速时需要取前方更多的点
	else pin = 0.45 * _speed;
	int step_high = 0.1*_speed;
	int step_low = 0.05 * _speed;


	if (_speed <= 150)              //mark: 原本的计算前方曲率是计算前方10米，20米的点，改进后取为随速度变化
	{                                 
		for (fStep = 0; fStep < pin; fStep++)
		{
			myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + step_low][0], _midline[fStep + step_low][1], _midline[fStep + step_high][0], _midline[fStep + step_high][1]);
			if (myCurve.r < minCruve)
			{
				minCruve = myCurve.r;
			}
		}
	}
	else   //mark:有点怪，应该是用来区分高低速不同取点，但还没动手
	{
		for (fStep = 0; fStep < pin; fStep++)
		{
			myCurve = getR(_midline[fStep][0], _midline[fStep][1], _midline[fStep + step_low][0], _midline[fStep + step_low][1], _midline[fStep + step_high][0], _midline[fStep + step_high][1]);
			if (myCurve.r < minCruve)
			{
				minCruve = myCurve.r;
			}
		}
	}
	//设定目标速度，如果前方弯道就设定低，直道就设定高
	if (!isCementRoad)       //mark:稍微提升了最高速度
	{                        //mark:沙地路面跑的还是比较慢的
		if (minCruve > 300)
			targetSpeed = 340;
		else if (minCruve > 240)
			targetSpeed = constrain(275, 335, minCruve + 25);
		else if (minCruve > 160)
			targetSpeed = constrain(225, 275, 0.625 * minCruve + 125);
		else if (minCruve > 100)
			targetSpeed = constrain(165, 225, minCruve + 65);
		else if (minCruve > 60)
			targetSpeed = constrain(115, 165, 1.25 * minCruve + 40);
		else if (minCruve > 20)
			targetSpeed = constrain(70, 115, 1.125 * minCruve + 47.5);
		else targetSpeed = 57;
	}
	/*if (!isCementRoad)
	{
		targetSpeed = constrain(62, 270, 0.4 * minCruve + 20);
	}*/
	else if (isCementRoad)         //mark:水泥路面可以跑的很快
	{
		if (minCruve > 300)
			targetSpeed = 360; // mark:用canstrain函数来控制目标速度平滑
		else if (minCruve > 240)
			targetSpeed = constrain(320, 360, minCruve + 80);
		else if (minCruve > 160)
			targetSpeed = constrain(290, 320, 0.75 * minCruve + 160);
		else if (minCruve > 100)
			targetSpeed = constrain(215, 285, 1.1667*minCruve + 98.333);
		else if (minCruve > 60)
			targetSpeed = constrain(150, 215, 1.625 * minCruve + 52.5);
		else if (minCruve > 20)
			targetSpeed = constrain(70, 150, 2 * minCruve + 30);
		else targetSpeed = 60;
	}
	
	//每当目标速度变化时初始化PID控制器，重设参数，清空积分器和微分器
	if (targetSpeed != lastTargetSpeed)
	{
		if (isCementRoad)
		{
			speedController.initial(4, 0.01, 20.0, targetSpeed);//mark:找到一组适合的参数来控制速度
			lastTargetSpeed = targetSpeed;
		}
		else if (!isCementRoad)//mark:沙地跟水泥暂时没区别，后续再看
		{
			speedController.initial(4, 0.001, 20.0, targetSpeed);
			lastTargetSpeed = targetSpeed;
		}
	}

	//控制油门刹车来调节速度
	if (!isCementRoad && flag)    //mark：如果判断路面成功，且是沙地路面采用此参数
	{
		if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{
			if (_speed > 250 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.5, speedController.calculate(currentSpeed));
			}
			if (_speed > 200 && abs(*cmdSteer) < 0.1)//mark:目测speed>200跟250的参数一样
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.5, speedController.calculate(currentSpeed));
			}
			if (_speed > 130 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.7, speedController.calculate(currentSpeed));
			}
			else if (_speed > 60 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.4, speedController.calculate(currentSpeed));
			}
			else if (_speed > 20 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.2, speedController.calculate(currentSpeed));
			}
			else
			{
				//除此之外的情况进一步限定油门
				*cmdAcc = 0.2;
			}
			//加速情况下，刹车为0
			*cmdBrake = 0;
		}
		else
		{
			//减速情况下，刹车
			*cmdAcc = 0;//mark:速度越快，刹车踩得越多
			if (currentSpeed - targetSpeed > 180 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.7;
			}
			else if (currentSpeed - targetSpeed > 120 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.5;
			}
			else if (currentSpeed - targetSpeed > 60 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.3;
			}
			else
			{
				*cmdBrake = 0.2;
			}
		}
	}
	else if (isCementRoad && flag)
	{
		if (currentSpeed < targetSpeed)  //当前速度低于目标，需要加速
		{
			if (_speed > 250 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.65, speedController.calculate(currentSpeed));
			}
			if (_speed > 200 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.8, speedController.calculate(currentSpeed));
			}
			if (_speed > 130 && abs(*cmdSteer) < 0.1)
			{
				//速度较快且舵角较小时，使用PID控制
				*cmdAcc = constrain(0.0, 0.95, speedController.calculate(currentSpeed));
			}
			else if (_speed > 60 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.65, speedController.calculate(currentSpeed));
			}
			else if (_speed > 20 && abs(*cmdSteer) < 0.2)
			{
				//速度较慢且舵角较小时，限定油门
				*cmdAcc = constrain(0.0, 0.4, speedController.calculate(currentSpeed));
			}
			else
			{
				//除此之外的情况进一步限定油门
				*cmdAcc = 0.2;
			}
			//加速情况下，刹车为0
			*cmdBrake = 0;
		}
		else
		{
			//减速情况下，刹车
			*cmdAcc = 0;
		if (currentSpeed - targetSpeed > 200 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.7;
			}
			else if (currentSpeed - targetSpeed > 140 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.5;
			}
			else if (currentSpeed - targetSpeed > 80 && abs(*cmdSteer) < 0.2)
			{
				*cmdBrake = 0.3;
			}
			else
			{
				*cmdBrake = 0.2;
			}
		}
	}

	//更新档位
	updateGear(cmdGear);

	//窗口可视化
	cls_visual.Fig2Y(1, 0, 300, 0, 500, 10, "Target V", targetSpeed, "Curvature", minCruve, "Current V", _speed);
	//cls_visual.Fig2Y(2, -0.3, 0.3, -0.5, 0.5, 10, "yaw", _yaw, "yawrate", _yawrate);
	cls_visual.Fig2Y(2, -1, 1, -1, 1, 10, "Acc", *cmdAcc, "Brake", *cmdBrake);
}