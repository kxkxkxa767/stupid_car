#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/model.h"
#include "tensorflow/lite/kernels/register.h"
#include "tensorflow/lite/delegates/xnnpack/xnnpack_delegate.h"
using namespace cv;
using namespace std;

typedef signed char         int8;   //有符号  8 bits
typedef unsigned char       uint8;  //无符号  8 bits
typedef signed short int    int16;  //有符号 16 bits
typedef unsigned short int  uint16; //无符号 16 bits
typedef signed int          int32;  //有符号 32 bits
typedef unsigned int        uint32; //无符号 32 bits

//压缩的图像质量
#define LCDW 160            //宽度
#define LCDH 128            //高度
#define YunTai_X 72       //云台X轴初始位置
#define YunTai_Y 73       //云台Y轴初始位置
#define Steer_Center 88   //舵机中值
#define Steer_min 78      //舵机限幅 右转 min 62
#define Steer_max 98      //舵机限幅 左转 max 102   
float Steer_P = 0.12;       //转向环参数赋值
float Steer_D = 0.25;    

uint8 Start;            //启动标志位 0放置挡板启动 3直接启动 
uint8 Block_Flag;       //避障标志位 0未执行 1正在执行
uint8 Zebra_Flag;       //斑马线标志位 0未执行 1正在执行 2完成后变道 3变道完成
uint8 Lane_Change;      //变道标志位 0未执行 1开始变道 2锥桶引导阶段 3发现黄锥桶 4锥桶引导结束
uint8 Stop_Flag;        //停车标志位 0未执行 1开始执行 2识别到底线
uint8 Block_Done;	   //避障完成标志位 0未完成 1完成

Rect Blue_Block;          		//避障蓝锥桶矩形
Rect Yellow_Parking;			//停车黄色区域
vector<Rect> Yellow_Block;		//全部黄色锥桶矩形
uint8 Left_Right_Flag;    	//左右避障,0右转避障 1左转避障
uint8 Lane_Change_Flag;		//变道标志,0右转变道 1左转变道
uint8 Stop_left_right;		//停车方向标志，0右停车，1左停车
uint8 Yaw_step;				//打角步骤，0打死转向，1回正，2完成

int last_left_boundary = 0;  // 上一帧左边界位置
int last_right_boundary = 0; // 上一帧右边界位置
bool left_jump_flag = false;   // 左边界跳变标志
bool right_jump_flag = false;  // 右边界跳变标志

Mat Original_image; 	//原始图像
Mat Cropped_image;  	//裁剪图像
Mat Lab_image;      	//Lab图像
Mat Compress_image; 	//压缩图像
Mat Binary_image;
Mat Blue_image;
Mat Yellow_image;
Mat Stop_image;
Mat Zebra_image;
Mat LineDisplay_image;	//用于画边线和中线的显示图像

struct TfLiteSegModel {
    std::unique_ptr<tflite::Interpreter> interpreter;
    TfLiteTensor* in_tensor = nullptr;
    TfLiteTensor* out_tensor = nullptr;
    int in_idx = -1, out_idx = -1;
    float in_scale = 1.0f;
    int in_zero = 0;
    float out_scale = 1.0f;
    int out_zero = 0;
    int OUT_THR = 0;
};

TfLiteSegModel seg_model;

struct ImageDealDatatypedef 
{
	int Wide;               //车道宽度
	int Center;             //车道中线
	//左右手法则扫线数据
	int LeftBoundary_First; //左边界第一次出现位置
	int RightBoundary_First;//右边界第一次出现位置
	int LeftBoundary;       //左边界位置
	int RightBoundary;      //右边界位置
};

struct ImageStatustypedef
{
	//图像信息
	int16_t OFFLineBoundary;//八邻域截止行
	int Det_True;
	float MU_P;
	float MU_D;
	//左右手法则扫线数据
	int16_t WhiteLine_L;
	int16_t WhiteLine_R;
};
ImageDealDatatypedef ImageDeal[LCDH];//记录单行数据
ImageStatustypedef ImageStatus;      //全图的全局变量

void Search_Bottom_Line_OTSU(Mat Image, int Row, int Col, int Bottonline)
{
	for (int Xsite = Col / 2 - 2; Xsite > 1; Xsite--)
	{
		if (Image.at<uchar>(Bottonline, Xsite) == 0 && Image.at<uchar>(Bottonline, Xsite - 1) == 255)
		{
			ImageDeal[Bottonline].LeftBoundary = Xsite;
			break;
		}
	}
	for (int Xsite = Col / 2 + 2; Xsite < LCDW - 1; Xsite++)
	{
		if (Image.at<uchar>(Bottonline, Xsite) == 0 && Image.at<uchar>(Bottonline, Xsite + 1) == 255)
		{
			ImageDeal[Bottonline].RightBoundary = Xsite;
			break;
		}
	}
}

void Search_Left_and_Right_Lines(Mat Image, int Row, int Col, int Bottonline)
{
	int Left_Rule[2][8] = {
		{0,-1,1,0,0,1,-1,0 },
		{-1,-1,1,-1,1,1,-1,1}
	};
	int Right_Rule[2][8] = {
		{0,-1,1,0,0,1,-1,0 },
		{1,-1,1,1,-1,1,-1,-1}
	};
	int num = 0;
	int Left_Ysite = Bottonline;
	int Left_Xsite = ImageDeal[Bottonline].LeftBoundary;
	int Left_Rirection = 0;
	int Pixel_Left_Ysite = Bottonline;
	int Pixel_Left_Xsite = 0;
	int Right_Ysite = Bottonline;
	int Right_Xsite = ImageDeal[Bottonline].RightBoundary;
	int Right_Rirection = 0;
	int Pixel_Right_Ysite = Bottonline;
	int Pixel_Right_Xsite = 0;
	int Ysite = Bottonline;
	ImageStatus.OFFLineBoundary = 5;
	while (1)
	{
		num++;
		if (num > 500)
		{
			ImageStatus.OFFLineBoundary = Ysite;
			break;
		}
		if (Ysite >= Pixel_Left_Ysite && Ysite >= Pixel_Right_Ysite)
		{
			if (Ysite < ImageStatus.OFFLineBoundary)
			{
				ImageStatus.OFFLineBoundary = Ysite;
				break;
			}
			else
			{
				Ysite--;
			}
		}
		if ((Pixel_Left_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)
		{
			Pixel_Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
			Pixel_Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
			if (Image.at<uchar>(Pixel_Left_Ysite, Pixel_Left_Xsite) == 255)
			{
				if (Left_Rirection == 3)
					Left_Rirection = 0;
				else
					Left_Rirection++;
			}
			else
			{
				Pixel_Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
				Pixel_Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
				if (Image.at<uchar>(Pixel_Left_Ysite, Pixel_Left_Xsite) == 255)
				{
					Left_Ysite = Left_Ysite + Left_Rule[0][2 * Left_Rirection + 1];
					Left_Xsite = Left_Xsite + Left_Rule[0][2 * Left_Rirection];
					if (ImageDeal[Left_Ysite].LeftBoundary_First == 6)
						ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
					ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
				}
				else
				{
					Left_Ysite = Left_Ysite + Left_Rule[1][2 * Left_Rirection + 1];
					Left_Xsite = Left_Xsite + Left_Rule[1][2 * Left_Rirection];
					if (ImageDeal[Left_Ysite].LeftBoundary_First == 6)
						ImageDeal[Left_Ysite].LeftBoundary_First = Left_Xsite;
					ImageDeal[Left_Ysite].LeftBoundary = Left_Xsite;
					if (Left_Rirection == 0)
						Left_Rirection = 3;
					else
						Left_Rirection--;
				}
			}
		}
		if ((Pixel_Right_Ysite > Ysite) || Ysite == ImageStatus.OFFLineBoundary)
		{
			Pixel_Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
			Pixel_Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
			if (Image.at<uchar>(Pixel_Right_Ysite, Pixel_Right_Xsite) == 255)
			{
				if (Right_Rirection == 0)
					Right_Rirection = 3;
				else
					Right_Rirection--;
			}
			else
			{
				Pixel_Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
				Pixel_Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
				if (Image.at<uchar>(Pixel_Right_Ysite, Pixel_Right_Xsite) == 255)
				{
					Right_Ysite = Right_Ysite + Right_Rule[0][2 * Right_Rirection + 1];
					Right_Xsite = Right_Xsite + Right_Rule[0][2 * Right_Rirection];
					if (ImageDeal[Right_Ysite].RightBoundary_First == Col - 6)
						ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
					ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
				}
				else
				{
					Right_Ysite = Right_Ysite + Right_Rule[1][2 * Right_Rirection + 1];
					Right_Xsite = Right_Xsite + Right_Rule[1][2 * Right_Rirection];
					if (ImageDeal[Right_Ysite].RightBoundary_First == Col - 6)
						ImageDeal[Right_Ysite].RightBoundary_First = Right_Xsite;
					ImageDeal[Right_Ysite].RightBoundary = Right_Xsite;
					if (Right_Rirection == 3)
						Right_Rirection = 0;
					else
						Right_Rirection++;
				}
			}
		}
		if (abs(Pixel_Right_Xsite - Pixel_Left_Xsite) < 6)
		{
			ImageStatus.OFFLineBoundary = Ysite;
			break;
		}
	}
}

void Search_Border_OTSU(Mat Image, int Row, int Col, int Bottonline)
{
	ImageStatus.WhiteLine_L = 0;
	ImageStatus.WhiteLine_R = 0;
	for (int Xsite = 0; Xsite < LCDW; Xsite++)
	{
		Image.at<uchar>(0, Xsite) = 255;
		Image.at<uchar>(Bottonline + 1, Xsite) = 255;
	}
	for (int Ysite = 0; Ysite < LCDH; Ysite++)
	{
		ImageDeal[Ysite].LeftBoundary_First = 6;
		ImageDeal[Ysite].RightBoundary_First = Col-6;
		ImageDeal[Ysite].LeftBoundary = 6;
		ImageDeal[Ysite].RightBoundary = Col - 6;
		Image.at<uchar>(Ysite, 6) = 255;
		Image.at<uchar>(Ysite, LCDW - 6) = 255;
	}
	Search_Bottom_Line_OTSU(Image, Row, Col, Bottonline);
	Search_Left_and_Right_Lines(Image, Row, Col, Bottonline);
}

void Center_Line()
{
	if(Block_Flag == 1 && Block_Done == 0 && Blue_Block.empty())
    {
        for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
            // 左避障：用八邻域中心线作为右边界辅助
            if(Left_Right_Flag == 1)
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBoundary_First + (ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2) / 2;
            // 右避障：用八邻域中心线作为左边界辅助
            else if(Left_Right_Flag == 0)
                ImageDeal[Ysite].Center = ((ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2 + ImageDeal[Ysite].RightBoundary_First) / 2;
        }
    }
	else if(Block_Flag == 1 && Block_Done == 0 && !Blue_Block.empty())
	{
        int blue_center = Blue_Block.x + Blue_Block.width / 2;
        for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
            // 左避障：用蓝色锥桶中心线作为右边界辅助
            if(Left_Right_Flag == 1)
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBoundary_First + blue_center) / 2;
            // 右避障：用蓝色锥桶中心线作为左边界辅助
            else if(Left_Right_Flag == 0)
                ImageDeal[Ysite].Center = (blue_center + ImageDeal[Ysite].RightBoundary_First) / 2;
        }
	}
	else if(Stop_Flag == 1)
	{
        for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
            // 左停车: 用八邻域中心线作为右边界辅助
            if(Stop_left_right == 1)
                ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBoundary_First + (ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2) / 2;
            // 右停车：用八邻域中心线作为左边界辅助
            else if(Stop_left_right == 0)
                ImageDeal[Ysite].Center = ((ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2 + ImageDeal[Ysite].RightBoundary_First) / 2;
        }
	}
	else//正常扫线
	{
		for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
		{
			ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2; 
		}
	}
}

//PD控制舵机
void SteerPID_Realize_Twice()
{
	//定义为寄存器变量，只能用于整型和字符型变量，提高运算速度
	int32 iError, SteerErr;  //当前误差 
	static int32 LastError;  //前次误差
	int PWM;
	float Kp;  //动态P
	float Kd;

	iError = ImageStatus.Det_True;  //计算当前误差

	Kp = Steer_P;
	Kd = Steer_D;
	if(LastError!=0)
	 SteerErr = (Kp * iError + (Kd) * (iError - LastError));  //只用PD
	else
	  SteerErr = (Kp * iError);  //只用PD
	LastError = iError;//更新上次误差

	PWM = Steer_Center - SteerErr;
	if(PWM>Steer_max)  PWM = Steer_max;
	if(PWM<Steer_min)  PWM = Steer_min;

	//printf("Error:%d\n",iError);
	//printf("PWM:%d\n",PWM);
	gpioPWM(12, PWM);  //舵机
}

void motorSet(int data)//电机限幅
{
	if (data > 4000) data = 4000;
	if (data < -4000) data = -4000;
	gpioPWM(13, 10000 + data);//10200正转起点，9800反转起点
}

void ClearAllGPIO()
{
	//引脚重置
	int pins[] = {12, 13, 22 ,23};
	for (int i = 0; i < sizeof(pins)/sizeof(pins[0]); ++i) {
		gpioSetMode(pins[i], PI_OUTPUT);
		gpioWrite(pins[i], 0); // 拉低
	}
}

void PWMInit()
{
	//舵机初始化
	gpioSetMode(12, PI_OUTPUT);
	gpioSetPWMfrequency(12, 50);
	gpioSetPWMrange(12, 1000);
	gpioPWM(12, Steer_Center);
	//云台x
	gpioSetMode(22, PI_OUTPUT);
	gpioSetPWMfrequency(22, 50);
	gpioSetPWMrange(22, 1000);
	gpioPWM(22, YunTai_X);
	//云台y
	gpioSetMode(23, PI_OUTPUT);
	gpioSetPWMfrequency(23, 50);
	gpioSetPWMrange(23, 1000);
	gpioPWM(23, YunTai_Y);
	//电机初始化
	gpioSetMode(13, PI_OUTPUT);
	gpioSetPWMfrequency(13, 200);
	gpioSetPWMrange(13, 40000);
	gpioPWM(13, 10000);
	time_sleep(1);                                           
}

vector<Rect> YellowBlock() 
{
    vector<vector<Point>> contours;
    findContours(Yellow_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Yellow_Block.clear();
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        if (r.area() > 2) {
            Yellow_Block.push_back(r);
        }
    }
    return Yellow_Block;
}

void DetectParkingArea()
{
    vector<vector<Point>> contours;
    findContours(Stop_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Yellow_Parking = Rect(); // 先清空

    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        double area = r.area();
        if (area > 5 && area > max_area) { // 面积阈值为5
            max_area = area;
            Yellow_Parking = r;
        }
    }
}

void Car_start()
{
	Start = 3;
}

void BlueBlock() // 识别到蓝色锥桶
{
    // 在Blue_image中寻找面积最大的轮廓
    vector<vector<Point>> contours;
    findContours(Blue_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Rect max_rect;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        double area = r.area();
        if (area > max_area) {
            max_area = area;
            max_rect = r;
        }
    }

    if (max_area > 10) // 对识别到的蓝色物体限幅
    {
        Blue_Block = max_rect;
    }
}

bool DetectZebraCrossing()
{
    // 直接在全局变量 zebra_image 中寻找面积最大的轮廓
    vector<vector<Point>> contours;
    findContours(Zebra_image, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Rect max_rect;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        double area = r.area();
        if (area > max_area) {
            max_area = area;
            max_rect = r;
        }
    }

    // 判断最大矩形底部是否低于某个阈值
    if (max_area > 10) {
        int bottom = max_rect.y + max_rect.height;
        printf("Zebra max_rect: x=%d y=%d w=%d h=%d bottom=%d\n", max_rect.x, max_rect.y, max_rect.width, max_rect.height, bottom);
        if (bottom > 40) {
            return true;
        }
    }
    return false;
}

bool DetectBoundaryJump(int threshold)
{
    int left_now = 0, right_now = 0, count = 0;
    for (int y = LCDH - 1; y > LCDH - 10; y--) 
    {
        left_now += ImageDeal[y].LeftBoundary_First;
        right_now += ImageDeal[y].RightBoundary_First;
        count++;
    }
    left_now /= count;
    right_now /= count;
    //printf("abs(left_now - last_left_boundary): %d\n", abs(left_now - last_left_boundary));
    //printf("abs(right_now - last_right_boundary): %d\n", abs(right_now - last_right_boundary));

    if (last_left_boundary > 0 && abs(left_now - last_left_boundary) > threshold && !left_jump_flag)
    {
        left_jump_flag = true;
        printf("Left Boundary Jump Detected! Change: %d\n", abs(left_now - last_left_boundary));
    }
    if (last_right_boundary > 0 && abs(right_now - last_right_boundary) > threshold && !right_jump_flag)
    {
        right_jump_flag = true;
        printf("Right Boundary Jump Detected! Change: %d\n", abs(right_now - last_right_boundary));
    }
    last_left_boundary = left_now;
    last_right_boundary = right_now;
    return left_jump_flag && right_jump_flag;
}

void CalcError()
{
	if(Lane_Change == 1 || Lane_Change == 3)
	{
		// 截止行下30行的中心线平均值作为误差
		int sum = 0, count = 0;
		int base = ImageStatus.OFFLineBoundary + 1;
		for(int i = 0; i < 30; ++i)
		{
			int y = base + i;
			if(y >= LCDH) break;
			sum += ImageDeal[y].Center;
			count++;
		}
		if(count > 0)
			ImageStatus.Det_True = sum / count - LCDW / 2 + 1;
	}
	else
	{
		if (ImageStatus.OFFLineBoundary <= 60)
		{
			ImageStatus.Det_True = (ImageDeal[62].Center + ImageDeal[64].Center+ ImageDeal[66].Center) / 3 - LCDW / 2 + 1;
		}
		else
		{
			ImageStatus.Det_True = ImageDeal[ImageStatus.OFFLineBoundary + 10].Center - LCDW / 2 + 1;//5
		}
	}
}

void SteerControl()
{
	// 1. 初始状态回正
	if (Start != 3)
	{
		gpioPWM(12, Steer_Center);
		return;
	}
	// 2. 避障
	/*
	if(Block_Flag == 1 && Block_Done == 0)
	{
		Steer_P = 0.13;
		Steer_D = 0.14;
		SteerPID_Realize_Twice();
		return;
	}
	*/
	if(Block_Flag == 1 && !Blue_Block.empty())
	{
		Steer_P = 0.20;
		Steer_D = 0.22;
		SteerPID_Realize_Twice();
		return;
	}
	if(Block_Flag == 1 && Blue_Block.empty())
	{
		Steer_P = 0.20;
		Steer_D = 0.22;
		SteerPID_Realize_Twice();
		return;
	}
	if(Block_Done == 1 && Zebra_Flag == 0)
	{
		Steer_P = 0.20;
		Steer_D = 0.22;
		SteerPID_Realize_Twice();
		return;
	}
	// 3. 斑马线停车
	if (Zebra_Flag == 1)
	{
		Steer_P = 0.20;
		Steer_D = 0.22;
		SteerPID_Realize_Twice();
		return;
	}
	// 4. 强制变道固定打角
	if (Lane_Change == 1)//0右变道
	{	
		if (Yaw_step == 0)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, Steer_min);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, Steer_max);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, Steer_max);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, Steer_min);
			return;
		}
	}
	if(Lane_Change == 2)
	{
		Steer_P = 0.25;
		Steer_D = 0.15;
		SteerPID_Realize_Twice();
		return;
	}
	// 5. 锥桶引导阶段
	if (Lane_Change == 3)
	{
    	if (Yaw_step == 0)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, Steer_max);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, Steer_min);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, Steer_min);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, Steer_max);
			return;
		}
	}
	//7. 停车状态
	if(Stop_Flag == 1) 
	{
		Steer_P = 0.18;
		Steer_D = 0.14;
		SteerPID_Realize_Twice();
		return;
	}
	if(Stop_Flag == 2)
	{
		Steer_P = 0.55;
		Steer_D = 0.20;
		SteerPID_Realize_Twice();
		return;
	}
	if(Stop_Flag == 3)
	{
		Steer_P = 0.50;
		Steer_D = 0.20;
		SteerPID_Realize_Twice();
		return;
	}
	// 8. 正常巡线
	Steer_P = 0.12;
	Steer_D = 0.13;
	SteerPID_Realize_Twice();
}

//750勉强起步
void MotorControl()//电机调控
{
	static int Zebra_time = 0;
	static int Stop_time = 0;
	// 1. 避障
	if (Block_Flag == 1) 
	{
		motorSet(750); // 避障速度
		return;
	}
	// 2. 斑马线完全停车
	if (Zebra_Flag == 1) 
	{
		Zebra_time++;
		if(Zebra_time < 50)
        	motorSet(-2000);
		else
			motorSet(0);
		return;
    }
	//3. 强制变道
	if(Lane_Change == 1)
	{
		motorSet(800);
		return;
	}
	//4. 锥桶引导变道
	if (Lane_Change == 2) 
	{
		motorSet(800);
		return;
	}
	//5. 停车区域引导
	if(Stop_Flag == 1 || Stop_Flag == 2)
	{
		motorSet(800);
		return;
	}
	if(Stop_Flag ==	3)
	{
		Stop_time++;
		if(Stop_time < 50)
        	motorSet(-2000);
		else
			motorSet(0);
        return;
	}
	// 6. 正常巡线
	if (Start == 3) 
	{
		motorSet(1000);//800
		return;
	}
	// 8. 其它情况
	motorSet(0);
}

void ZoomImage(const Mat& src, int scale)
{
	Mat dst;
	resize(src, dst, Size(src.cols * scale, src.rows * scale), 0, 0, INTER_NEAREST);
	imshow("Show", dst);
}

void ShowBoundaryAndCenterLine() 
{
    // 图像放大2倍
    resize(Binary_image, LineDisplay_image, Size(2 * LCDW, 2 * LCDH));
    cvtColor(LineDisplay_image, LineDisplay_image, COLOR_GRAY2BGR);
    for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
    {
        // 左边线 红色
        Rect leftRect(ImageDeal[Ysite].LeftBoundary_First * 2, Ysite * 2, 4, 4);
        rectangle(LineDisplay_image, leftRect, Scalar(0,0,255), -1);
        // 右边线 蓝色
        Rect rightRect(ImageDeal[Ysite].RightBoundary_First * 2, Ysite * 2, 4, 4);
        rectangle(LineDisplay_image, rightRect, Scalar(255,0,0), -1);
        // 中线 绿色
        Rect centerRect(ImageDeal[Ysite].Center * 2, Ysite * 2, 4, 4);
        rectangle(LineDisplay_image, centerRect, Scalar(0,255,0), -1);
    }
    // 绘制八邻域截止行（黄色横线）
    int y = ImageStatus.OFFLineBoundary * 2;
    line(LineDisplay_image, Point(0, y), Point(LineDisplay_image.cols-1, y), Scalar(0,255,255), 2);
}

void SaveImageToVideo(const Mat& frame) {
	static VideoWriter writer;
	static bool inited = false;
	if (frame.empty()) return; // 避免写入空帧
	if (!inited) {
		writer.open("output.avi", VideoWriter::fourcc('M','J','P','G'), 30, Size(frame.cols, frame.rows));
		inited = writer.isOpened();
	}
	if (inited) {
		Mat out;
		if (frame.channels() == 1)
			cvtColor(frame, out, COLOR_GRAY2BGR);
		else
			out = frame;
		writer.write(out);
	}
}

void handle_sigint(int sig)
{
	printf("Ctrl+C detected, stopping motors and cleaning up...\n");
	ClearAllGPIO();
	gpioTerminate();
	destroyAllWindows();
	exit(0);
}

int main() 
{
	Start=3;
	Block_Flag=0;
	Zebra_Flag=0;		//0执行斑马线,1检测到斑马线，2斑马线停车结束,3执行变道
	Lane_Change=0;		//0从头测试,1强制变道,2锥桶引导,3开始引导，4变道结束
	Stop_Flag=0;
	uint8 warmup=0;		//相机预热计数
	Block_Done=0;	//避障完成标志
	Yaw_step = 0;		//避障阶段 0打死转向 1回正 2避障完成
	static uint8 Time_num = 0;     		//时间计数器

	signal(SIGINT, handle_sigint);//中断信号接收

	//gpio引脚初始化
	if (gpioInitialise() < 0) 
	{
		std::cerr << "pigpio faild" << std::endl;
		return -1;
	}
	ClearAllGPIO();             // 清除全部引脚状态           
	PWMInit();                  //舵机 电机初始化

	string tflite_path = "/home/5G/5G/main/studyall/src/together.tflite";
	auto model = tflite::FlatBufferModel::BuildFromFile(tflite_path.c_str());
	tflite::ops::builtin::BuiltinOpResolver resolver;
	tflite::InterpreterBuilder(*model, resolver)(&seg_model.interpreter, 4);
	TfLiteXNNPackDelegateOptions xnnpack_options = TfLiteXNNPackDelegateOptionsDefault();
	xnnpack_options.num_threads = 4;
	auto xnnpack_delegate = TfLiteXNNPackDelegateCreate(&xnnpack_options);
	seg_model.interpreter->ModifyGraphWithDelegate(xnnpack_delegate);
	seg_model.interpreter->AllocateTensors();
	seg_model.in_idx = seg_model.interpreter->inputs()[0];
	seg_model.out_idx = seg_model.interpreter->outputs()[0];
	seg_model.in_tensor = seg_model.interpreter->tensor(seg_model.in_idx);
	seg_model.out_tensor = seg_model.interpreter->tensor(seg_model.out_idx);
	seg_model.in_scale = seg_model.in_tensor->params.scale;
	seg_model.in_zero = seg_model.in_tensor->params.zero_point;
	seg_model.out_scale = seg_model.out_tensor->params.scale;
	seg_model.out_zero = seg_model.out_tensor->params.zero_point;
	seg_model.OUT_THR = static_cast<int>(round(0.5 / seg_model.out_scale + seg_model.out_zero));

	//VideoCapture cap("/home/5G/video/5.avi");
	//VideoCapture cap("/home/5G/video/1.10.mp4");
	VideoCapture cap(0, CAP_V4L2); // 用V4L2后端打开摄像头
	if (!cap.isOpened()) 
	{
		std::cerr << "Can not open camera" << std::endl;
		return -1;
	}
	// 设置分辨率、帧率、格式（MJPG）
	cap.set(CAP_PROP_FRAME_WIDTH, 320);
	cap.set(CAP_PROP_FRAME_HEIGHT,240);
	cap.set(CAP_PROP_FPS, 120);
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

	Binary_image = Mat::zeros(LCDH, LCDW, CV_8U);
	Blue_image = Mat::zeros(LCDH, LCDW, CV_8U);
	Yellow_image = Mat::zeros(LCDH, LCDW, CV_8U);
	Stop_image = Mat::zeros(LCDH, LCDW, CV_8U);
	Zebra_image = Mat::zeros(LCDH, LCDW, CV_8U);

	while (cap.read(Original_image))
	{
		if(warmup<20){warmup ++; continue;}//预热10帧
		//auto start = std::chrono::high_resolution_clock::now(); // 记录起始时间
		//上下镜像翻转
		//flip(Original_image, Original_image, 0);
		//左右镜像
		//flip(Original_image, Original_image, 1);
		//裁剪
		Rect roi_rect(0, Original_image.rows / 2, Original_image.cols, Original_image.rows / 2);
		Cropped_image = Original_image(roi_rect);
		//压缩
		resize(Cropped_image,Compress_image , Size(LCDW, LCDH), 0, 0, INTER_NEAREST);
		cvtColor(Compress_image, Compress_image, COLOR_BGR2RGB);

		Car_start();

		if(Start==3)
		{
			Binary_image.setTo(0);
			Blue_image.setTo(0);
			Zebra_image.setTo(0);
			Yellow_image.setTo(0);
			Stop_image.setTo(0);
			Mat norm_img;
			Compress_image.convertTo(norm_img, CV_32FC3, 1.0 / 255.0);
			vector<int8_t> x_buf(LCDH * LCDW * 3);
			const float* src = reinterpret_cast<const float*>(norm_img.ptr<float>(0));
			for (int i = 0; i < LCDH * LCDW * 3; ++i) {
				int quant = static_cast<int>(round(src[i] / seg_model.in_scale + seg_model.in_zero));
				x_buf[i] = static_cast<int8_t>(max(-128, min(127, quant)));
			}
			memcpy(seg_model.interpreter->typed_tensor<int8_t>(seg_model.in_idx), x_buf.data(), x_buf.size());
			seg_model.interpreter->Invoke();
			int out_h = seg_model.out_tensor->dims->data[1];
			int out_w = seg_model.out_tensor->dims->data[2];
			int out_c = seg_model.out_tensor->dims->data[3];
			int8_t* y = seg_model.interpreter->typed_tensor<int8_t>(seg_model.out_idx);
			for (int i = 0; i < out_h; ++i) {
				for (int j = 0; j < out_w; ++j) {
					int idx = i * out_w * out_c + j * out_c;
					int max_val = y[idx];
					int max_cls = 0;
					for (int k = 1; k < out_c; ++k) {
						if (y[idx + k] > max_val) {
							max_val = y[idx + k];
							max_cls = k;
						}
					}
					if (Block_Done == 0) 
					{
						if (max_cls == 1) Blue_image.at<uchar>(i, j) = 255;
					}
					if (Block_Done == 1 && Zebra_Flag == 0)
					{
						if (max_cls == 5) Zebra_image.at<uchar>(i, j) = 255;
					}
					if (Zebra_Flag == 3 && Lane_Change !=4)
					{
						if (max_cls == 4) Yellow_image.at<uchar>(i, j) = 255;
					}
					if (Lane_Change ==4 && Stop_Flag==0)
					{
						if (max_cls == 3) Stop_image.at<uchar>(i, j) = 255;
					}
					if (max_cls == 2) Binary_image.at<uchar>(i, j) = 255;
				}
			}
			if(Stop_Flag != 0)
			{
				bitwise_or(Binary_image, Stop_image, Binary_image);
			}
			// 边界和中线检测流程
			Search_Border_OTSU(Binary_image, LCDH, LCDW, LCDH - 2);

			if(Block_Done == 0)
			{
				BlueBlock();
				if(!Blue_Block.empty() && Block_Flag == 0)
				{
					int Blue_Block_Xsite = Blue_Block.x + Blue_Block.width / 2; // 蓝色锥桶中心X坐标
					int Blue_Block_Ysite = Blue_Block.y + Blue_Block.height/ 2;    // 蓝色
					// 检查中心是否在指定的左右限制内以及Y坐标
					if (Blue_Block_Xsite > ImageDeal[Blue_Block_Ysite].LeftBoundary_First && 
						Blue_Block_Xsite <= ImageDeal[Blue_Block_Ysite].RightBoundary_First && 
						Blue_Block_Ysite > 5 && Blue_Block_Ysite< 110)
					{
						Block_Flag = 1; // 进入避障状态
						//Left_Right_Flag = 0; //0右避障 1左避障	
						if (Blue_Block_Xsite < LCDW/2)
						{
							Left_Right_Flag = 0; // 右避障
						} 
						else 
						{
							Left_Right_Flag = 1; // 左避障
						}	
						printf("Obstacle Detected! Left_Right_Flag:%d\n",Left_Right_Flag);
					}
				}
				if(Block_Flag == 1)
				{
					Time_num++;
					printf("Time_num:%d\n",Time_num);
					if(Time_num > 20) //连续15帧未检测到蓝色
					{
						Block_Flag = 0; // 退出避障状态
						Block_Done = 1; // 避障完成
						Time_num = 0;
						printf("Obstacle Avoidance Completed\n");
					}
				}
			}
			//斑马线检测
			if(Zebra_Flag== 0 && Block_Done == 1)
			{
				if (DetectZebraCrossing()) 
				{
					printf("Zebra Crossing Detected!\n");
					Zebra_Flag = 1;
				}
			}

			//斑马线停车时间计数
			if(Zebra_Flag == 1)
			{
				Time_num ++;
				if(Time_num == 30)
				{
					//system("amixer -c 2 sset 'PCM' 100%");
					//system("aplay -D plughw:2,0 /home/5G/5G/voice/1.wav"); // 播放语音
				}
				printf("Zebra Time_num:%d\n",Time_num);
				if(Time_num >= 90)
				{
					Time_num = 0;
					Zebra_Flag = 2;
				}
			}

			if(Zebra_Flag ==2)
			{
				Time_num++;
				if(Time_num > 10) // 延时30帧后直接赋值为3
				{
					Zebra_Flag = 3;
					Time_num = 0;
					printf("Zebra line gone, start lane change!\n");
				}
			}

			// 斑马线后变道
			if(Zebra_Flag == 3)
			{
				if(Lane_Change == 0)
				{
					//检测转向函数
					Lane_Change_Flag = 1;//0右转变道 1左转变道
					Lane_Change = 1; // 标记变道中
				}
				else if (Lane_Change == 1)
				{
					if(Yaw_step == 0) // 打死转向阶段
					{
						// 只要两者发生过跳变，就进入回正
						if (DetectBoundaryJump(30))
						{
							Time_num++;
						}
						if(Time_num >= 10)
						{
							Yaw_step = 1; // 进入回正
							left_jump_flag = false;
							right_jump_flag = false;
							Time_num = 0;
							printf("左右边界均已跳变，准备回正\n");
						}
					}
					else if(Yaw_step == 1)
					{
						if (abs(ImageStatus.Det_True) <= 20) // 阈值可根据实际调整
    					{
							Time_num++;
							if(Time_num >= 1 )
							{
								printf("Time_num:%d\n",Time_num);
        						Yaw_step = 0;        // 变道完成
        						Lane_Change = 2;     // 进入下一阶段
								Time_num = 0;
        						printf("回正完成，变道结束\n");
							}
    					}
					}
				}
				else if(Lane_Change == 2)
				{
					BlueBlock(); // 继续检测蓝色锥桶
					int Blue_Block_Xsite = Blue_Block.x + Blue_Block.width / 2; // 蓝色锥桶中心X坐标
					int Blue_Block_Ysite = Blue_Block.y + Blue_Block.height/ 2;    // 蓝色
					//printf("Blue Block Xsite:%d Ysite:%d\n",Blue_Block_Xsite,Blue_Block_Ysite);
					//printf("Blue Block.empty():%d\n",Blue_Block.empty());
					YellowBlock();
    				// 遍历所有黄色区域，寻找满足条件的矩形
    				for (const auto& rect : Yellow_Block) 
					{
        				int center_x = rect.x + rect.width / 2;
        				int center_y = rect.y + rect.height / 2;
						//printf("Yellow Block Center: (%d, %d)\n", center_x, center_y);
        				if (center_x > ImageDeal[center_y].LeftBoundary_First + 20 &&
            			center_x <= ImageDeal[center_y].RightBoundary_First - 20 &&
            			center_y > 5 && center_y < 30)
						{
							printf("Find Yellow Block Center: (%d, %d)\n", center_x, center_y);
							if(Blue_Block.empty() || Blue_Block_Ysite > 60)
							{
								Yaw_step = 0;
								Lane_Change = 3; // 进入锥桶引导阶段
								printf("Blue Block Ysite:%d\n",Blue_Block_Ysite);
								printf("Blue loss\n");
								break;
							}
						}	
    				}
				}
				else if(Lane_Change == 3)
				{
					if(Yaw_step == 0) // 打死转向阶段（锥桶引导）
    				{
						DetectBoundaryJump(10);
						YellowBlock();
						// 只要两者都发生过跳变（顺序不限），就进入回正
						if (Yellow_Block.empty())
						{
							Time_num++;
						}
						else
						{
							Time_num = 0;
						}
						//if(Time_num >2 && (left_jump_flag && right_jump_flag))
						//if(Time_num > 0 && (left_jump_flag || right_jump_flag))
						if(left_jump_flag && right_jump_flag)
						//if(left_jump_flag || right_jump_flag)
						{
							Yaw_step = 1; // 进入回正
							left_jump_flag = false;
							right_jump_flag = false;
							Time_num = 0;
							printf("锥桶引导：准备回正\n");
						}
					}
					else if(Yaw_step == 1)
					{
						if (abs(ImageStatus.Det_True) < 20) // 阈值可根据实际调整
						{
							Time_num++;
							if(Time_num >= 6)
							{
								printf("Time_num:%d\n",Time_num);
								Yaw_step = 0;
								Lane_Change = 4;     // 进入下一阶段（停车）
								Time_num = 0;
								printf("锥桶引导：回正完成，进入停车\n");
							}
						}
					}
				}
			}
			//执行停车
			if(Lane_Change == 4)
			{
				//左右转控制，0右停车，1左停车
				Stop_left_right = 1;
				DetectParkingArea();
				if (!Yellow_Parking.empty() && Stop_Flag == 0 && (LCDH - (Yellow_Parking.y + Yellow_Parking.height) < 90))
    			{
        			Stop_Flag = 1;
					Yaw_step = 0;
					printf("Parking Area Detected!\n");
    			}
				//printf("ImageStatus.OFFLineBoundary:%d\n",ImageStatus.OFFLineBoundary);
				if(Stop_Flag == 1 && ImageStatus.OFFLineBoundary >80)
				{
					Stop_Flag = 2;
					printf("Entering Parking Area!\n");
				}
				if(Stop_Flag == 2)
				{
					Time_num++;
					if(Time_num >10 && ImageStatus.OFFLineBoundary > 40)
					{
						Stop_Flag = 3;
						printf("Strat Parking\n");
					}
				}
				//printf("Stop_Flag:%d\n",Stop_Flag);
			}

			Center_Line();	//中心线修正
			CalcError();	//误差计算
			//printf("Error:%d\n",ImageStatus.Det_True);
			//SteerControl();	//舵机调控
			//MotorControl();	//电机调控

			ShowBoundaryAndCenterLine(); //中线边线显示
			//SaveImageToVideo(Original_image);
			//SaveImageToVideo(LineDisplay_image);
			//ZoomImage(Binary_image, 1);//放大指定图像
			//ZoomImage(Blue_image, 1);//放大指定图像
			ZoomImage(LineDisplay_image, 1);//放大指定图像
			//ZoomImage(Compress_image, 4);//放大指定图像
			//Original_image Cropped_image HSV_image Compress_image Binary_image LineDisplay_image
			
			//auto end = std::chrono::high_resolution_clock::now();   // 记录结束时间
			//double elapsed_ms = std::chrono::duration<double, std::milli>(end - start).count();
			//printf("Loop time: %.2f ms\n", elapsed_ms);
		}
		if (waitKey(30) == 'q') break; // 按q退出
	}
	ClearAllGPIO();
	gpioTerminate();
	cap.release();
	destroyAllWindows();
	return 0;
}