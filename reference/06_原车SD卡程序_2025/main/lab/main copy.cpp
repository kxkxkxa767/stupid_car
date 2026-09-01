#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <ctime>
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
#define LCDH 120            //高度
#define YunTai_X 75       //云台X轴初始位置
#define YunTai_Y 73       //云台Y轴初始位置
#define Steer_Center 75   //舵机中值
#define Steer_min 74      //舵机限幅 右转 min 62
#define Steer_max 94      //舵机限幅 左转 max 102   
float Steer_P = 0.12;       //转向环参数赋值
float Steer_D = 0.25;  

//斑马线检测区域
int minY = 0;
int maxY = 50;

uint8 Start;            //启动标志位 0放置挡板启动 3直接启动 
uint8 Block_Flag;       //避障标志位 0未执行 1正在执行
uint8 Zebra_Flag;       //斑马线标志位 0未执行 1正在执行 2完成后变道 3变道完成
uint8 Lane_Change;      //变道标志位 0未执行 1开始变道 2锥桶引导阶段 3发现黄锥桶 4锥桶引导结束
uint8 Stop_Flag;        //停车标志位 0未执行 1开始执行 2识别到底线
uint8 Block_Done;	   //避障完成标志位 0未完成 1完成
uint8 SlowSpeed_Flag;   //减速标志位 0未执行 1斑马线前减速 2停车前减速

Rect Blue_Block;          		//避障蓝锥桶矩形
Rect Yellow_Parking;			//停车黄色区域
vector<Rect> Yellow_Block;		//全部黄色锥桶矩形
uint8 Left_Right_Flag;    	//左右避障,0右转避障 1左转避障
uint8 Lane_Change_Flag;		//变道标志,0右转变道 1左转变道
uint8 Stop_left_right;		//停车方向标志，0右停车，1左停车
uint8 Yaw_step;				//打角步骤，0打死转向，1回正，2完成

Mat Original_image; 	//原始图像
Mat Cropped_image;  	//裁剪图像
Mat Lab_image;      	//Lab图像
Mat Compress_image; 	//压缩图像
Mat Binary_image;   	//二值图像
Mat Road_image;
Mat LineDisplay_image;	//用于画边线和中线的显示图像

double CenterLineSlope = 0.0; // 全局中线斜率
double LeftBoundarySlope = 0.0;   // 左边界斜率
double RightBoundarySlope = 0.0;  // 右边界斜率

typedef struct {
	int Wide;               //车道宽度
	int Center;             //车道中线
	//左右手法则扫线数据
	int LeftBoundary_First; //左边界第一次出现位置
	int RightBoundary_First;//右边界第一次出现位置
	int LeftBoundary;       //左边界位置
	int RightBoundary;      //右边界位置
} ImageDealDatatypedef;

typedef struct {
	//图像信息
	int16_t OFFLineBoundary;//八邻域截止行
	int Det_True;
	float MU_P;
	float MU_D;
	//左右手法则扫线数据
	int16_t WhiteLine_L;
	int16_t WhiteLine_R;
} ImageStatustypedef;
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
	/*
	if(Block_Flag == 0 && Block_Done == 0)
	{
		for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
            ImageDeal[Ysite].Center = ((ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2 + ImageDeal[Ysite].RightBoundary_First) / 2 - 25;
        }
	}
		*/
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

Rect Blue() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Scalar lower_Blue(0, 130, 40); // 蓝色Lab下界
    Scalar upper_Blue(255, 255, 100); // 蓝色Lab上界
    Mat mask;
    inRange(Lab_image, lower_Blue, upper_Blue, mask);
    //erode(mask, mask, kernel_3, Point(-1, -1), 2);
    //dilate(mask, mask, kernel_3, Point(-1, -1), 2);
    threshold(mask, mask, 127, 255, THRESH_BINARY);
    vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

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
    return max_rect;
}

vector<Rect> YellowBlock() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
	Mat hsv, mask;
 	cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);

    // 常用黄色 HSV 范围（可根据实际场景调节）
    Scalar lower_Y(20, 10, 100);   // H:20 S:100 V:100
    Scalar upper_Y(35, 255, 255);   // H:35 S:255 V:255

    inRange(hsv, lower_Y, upper_Y, mask);
    //erode(mask, mask, kernel_3, Point(-1, -1), 2);
    //dilate(mask, mask, kernel_3, Point(-1, -1), 2);
    threshold(mask, mask, 127, 255, THRESH_BINARY);
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Yellow_Block.clear();
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        if (r.area() > 15) {
            int x = (int)(r.x / (float)Lab_image.cols * LCDW + 0.5);
            int y = (int)(r.y / (float)Lab_image.rows * LCDH + 0.5);
            int w = (int)(r.width / (float)Lab_image.cols * LCDW + 0.5);
            int h = (int)(r.height / (float)Lab_image.rows * LCDH + 0.5);
            Yellow_Block.push_back(Rect(x, y, w, h));
			//printf("r.area(): %f\n", r.area());
        }
    }
    return Yellow_Block;
}

void DetectParkingArea()
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
	Mat hsv, mask;
 	cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);

    // 常用黄色 HSV 范围（可根据实际场景调节）
    Scalar lower_Y(20, 40, 100);   // H:20 S:100 V:100
    Scalar upper_Y(35, 255, 255);   // H:35 S:255 V:255

    inRange(hsv, lower_Y, upper_Y, mask);
	threshold(mask, mask, 127, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

	Yellow_Parking = Rect(); // 先清空

	double max_area = 0;
	for (size_t i = 0; i < contours.size(); ++i) {
		Rect r = boundingRect(contours[i]);
		double area = r.area();
		if (area > 50 && area > max_area) { // 面积阈值为5
			max_area = area;
			// 压缩到 LCDW × LCDH 坐标系
			int x = (int)(r.x / (float)Lab_image.cols * LCDW + 0.5);
			int y = (int)(r.y / (float)Lab_image.rows * LCDH + 0.5);
			int w = (int)(r.width / (float)Lab_image.cols * LCDW + 0.5);
			int h = (int)(r.height / (float)Lab_image.rows * LCDH + 0.5);
			Yellow_Parking = Rect(x, y, w, h);
		}
	}
}

void Car_start()
{
    static uint8 Start_num = 0;
    if(Start==0)
    {
        if(Blue().width > 50 && Blue().height > 50)
        {
            Start=1;
            printf("Barrier Placed\n");
            return;
        }
    }
    if(Start==1)
    {
        if(!(Blue().width > 50 && Blue().height > 50))
        {
            Start=2;
            Start_num=0;
            printf("Barrier Removed\n");
        }
        return;
    }
    if(Start==2)
    {
        if(Start_num<20)
        {
            Start_num++;
            printf("Waiting to Start:%d\n",Start_num);
            return;
        }
        else
        {
            Start=3;
            printf("Car Start!\n");
        }
    }
}

void Blue_Obstacles() // 识别到蓝色锥桶
{
	Blue_Block = Rect();
    Rect obstacle_object = Blue(); // 识别到的蓝色物体
    //printf("width: %d\n", obstacle_object.width);
    //printf("height: %d\n", obstacle_object.height);
    if (obstacle_object.area()>10) // 对识别到的蓝色物体限幅
    {
        // 压缩到 LCDW × LCDH 坐标系
        int x = (int)(obstacle_object.x / (float)Lab_image.cols * LCDW + 0.5);
        int y = (int)(obstacle_object.y / (float)Lab_image.rows * LCDH + 0.5);
        int w = (int)(obstacle_object.width / (float)Lab_image.cols * LCDW + 0.5);
        int h = (int)(obstacle_object.height / (float)Lab_image.rows * LCDH + 0.5);
        Blue_Block = Rect(x, y, w, h);
    }
}

bool DetectZebraCrossing()
{
    int net = 0;
    int NUM = 0;
    if(ImageStatus.OFFLineBoundary < 20)
    {
        NUM = 0;
        for (int Ysite = 20; Ysite < 30; Ysite++)//30 40 
        {
            for (int Xsite = ImageDeal[Ysite].LeftBoundary_First + 10; Xsite < ImageDeal[Ysite].RightBoundary_First - 10; Xsite++)
            {
                if(Binary_image.at<uchar>(Ysite, Xsite) == 0 && Binary_image.at<uchar>(Ysite, Xsite + 1) == 255)
                {
                    int Xstart = Xsite;
                    net =0;
                    while (Binary_image.at<uchar>(Ysite, Xstart + 1) == 255)
                    {
                        if (Binary_image.at<uchar>(Ysite, Xstart) == 255)
                        {
                            net++;
                        }
                        Xstart++;
                        //printf("net:%d\n", net);
                    }
                    printf("net:%d\n", net);
                    if(net >= 5)
                    {
                        NUM++;
                        Xsite = Xstart;
                    }
                }
            }
        }
    }
	printf("Zebra NUM: %d\n", NUM);
    if (NUM >= 8) 
    {
        printf("find zebra\n");
        return true;
    }
    return false;
}

bool DetectZebraCrossing_Hough()
{
    // 将压缩后的图像放大，便于处理
    Mat zebra_img;
    resize(Cropped_image, zebra_img, Size(800, 400));

    // 灰度化
    Mat gray_zebra;
    cvtColor(zebra_img, gray_zebra, COLOR_BGR2GRAY);

    // 高斯模糊
    Mat blur_zebra;
    GaussianBlur(gray_zebra, blur_zebra, Size(5, 5), 5);

    // 二值化
    Mat thresh_zebra;
    threshold(blur_zebra, thresh_zebra, 200, 255, THRESH_BINARY);

    // 腐蚀
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(3, 1));
    Mat img_ero;
    erode(thresh_zebra, img_ero, kernel_ero, Point(-1, -1), 3);

    // 膨胀
    Mat kernel_dia = getStructuringElement(MORPH_RECT, Size(5, 1));
    Mat img_dia;
    dilate(img_ero, img_dia, kernel_dia, Point(-1, -1), 1);

    // 查找轮廓
    vector<vector<Point>> contours;
    findContours(img_dia, contours, RETR_TREE, CHAIN_APPROX_NONE);

    //imshow("Zebra Contours", img_dia);

    // 统计宽度和高度符合要求且下边界小于200的轮廓数量
    int count = 0;
    bool has_valid_rect = false;
    int max_bottom = 0; // 记录最靠下的底线行数
    for (size_t i = 0; i < contours.size(); ++i)
    {
        Rect rect = boundingRect(contours[i]);
        if (rect.width > 10 && rect.height > 20)
        {
            count++;
            int bottom = rect.y + rect.height;
            if (bottom > max_bottom)
                max_bottom = bottom;
            if (bottom > 100)
                has_valid_rect = true;
        }
    }
    printf("Zebra stripe count: %d, Max bottom line: %d\n", count, max_bottom);
    // 条纹数达到7且有一个矩形下边界小于200行
    return (count >= 7) && has_valid_rect;
}

bool CheckBoundarySlopeForYawStep()
{
    static int slope_count = 0;
    static double last_left_slope = 0.0;
    static double last_right_slope = 0.0;
    static bool first_run = true;
    static bool Left_Change = false;
    static bool Right_Change = false;
    static int frame_count = 0; // 新增帧计数

    int y_start = LCDH - 50;
    int y_end = LCDH - 5;

    if (ImageStatus.OFFLineBoundary > y_start)
        y_start = ImageStatus.OFFLineBoundary + 1;
    if (y_start > y_end)
        return false;

    // 左边界斜率拟合
    double sum_y_l = 0, sum_x_l = 0, sum_xy_l = 0, sum_xx_l = 0;
    int N_l = 0;
    for (int y = y_start; y <= y_end; ++y)
    {
        double x = y;
        double y_val = ImageDeal[y].LeftBoundary_First;
        sum_x_l += x;
        sum_y_l += y_val;
        sum_xy_l += x * y_val;
        sum_xx_l += x * x;
        N_l++;
    }
    double left_slope = 0.0;
    if (N_l > 1)
        left_slope = (N_l * sum_xy_l - sum_x_l * sum_y_l) / (N_l * sum_xx_l - sum_x_l * sum_x_l);

    // 右边界斜率拟合
    double sum_y_r = 0, sum_x_r = 0, sum_xy_r = 0, sum_xx_r = 0;
    int N_r = 0;
    for (int y = y_start; y <= y_end; ++y)
    {
        double x = y;
        double y_val = ImageDeal[y].RightBoundary_First;
        sum_x_r += x;
        sum_y_r += y_val;
        sum_xy_r += x * y_val;
        sum_xx_r += x * x;
        N_r++;
    }
    double right_slope = 0.0;
    if (N_r > 1)
        right_slope = (N_r * sum_xy_r - sum_x_r * sum_y_r) / (N_r * sum_xx_r - sum_x_r * sum_x_r);

    // 首帧初始化，不做跳变判断
    if (first_run)
    {
        last_left_slope = left_slope;
        last_right_slope = right_slope;
        first_run = false;
        frame_count = 1; // 首帧后计数为1
        return false;
    }
	frame_count++; // 增加帧计数
    // 右变道
	if(frame_count >= 15)
	{
		if (Lane_Change_Flag == 0)
		{
			if (Lane_Change == 1)
			{
				if (!Right_Change && fabs(right_slope - last_right_slope) >= 0.2)
					Right_Change = true;
				if (Right_Change && !Left_Change && fabs(left_slope - last_left_slope) >= 0.25)
					Left_Change = true;
			}
			else if (Lane_Change == 3)
			{
				if (!Left_Change && fabs(left_slope - last_left_slope) >= 0.2)
					Left_Change = true;
				if (Left_Change && !Right_Change && fabs(right_slope - last_right_slope) >= 0.25)
					Right_Change = true;
			}
		}
		// 左变道
		else if (Lane_Change_Flag == 1)
		{
			if (Lane_Change == 1)
			{
				if (!Left_Change && fabs(left_slope - last_left_slope) >= 0.2)
					Left_Change = true;
				if (Left_Change && !Right_Change && fabs(right_slope - last_right_slope) >= 0.25)
					Right_Change = true;
			}
			else if (Lane_Change == 3)
			{
				if (!Right_Change && fabs(right_slope - last_right_slope) >= 0.2)
					Right_Change = true;
				if (Right_Change && !Left_Change && fabs(left_slope - last_left_slope) >= 0.25)
					Left_Change = true;
			}
		}
	}
    printf("LeftSlope: %.3f, RightSlope: %.3f, LeftChange: %d, RightChange: %d\n",
        left_slope, right_slope, Left_Change, Right_Change);

    last_left_slope = left_slope;
    last_right_slope = right_slope;

	if ((Lane_Change == 1 && Left_Change && Right_Change) ||
		(Lane_Change == 3 && (Left_Change && Right_Change)))
	{
		slope_count++;
		int slope_delay = 1; // 默认延时
		if (Lane_Change == 1 && Lane_Change_Flag == 0)
			slope_delay = 1; // 右变道第一阶段
		else if (Lane_Change == 1 && Lane_Change_Flag == 1)
			slope_delay = 1; // 左变道第一阶段
		else if (Lane_Change == 3 && Lane_Change_Flag == 0)
			slope_delay = 1; // 右变道锥桶阶段
		else if (Lane_Change == 3 && Lane_Change_Flag == 1)
			slope_delay = 1; // 左变道锥桶阶段

		if (slope_count >= slope_delay)
		{
			slope_count = 0;
			first_run = true; // 变道完成后重新初始化
			frame_count = 0;   // 重新计数
			Left_Change = false;
			Right_Change = false;
			return true;
		}
	}
    return false;
}

void UpdateCenterLineSlope()
{
    // 中线计算范围：截止行以下开始，到 LCDH-10 结束
    int c_y_start = ImageStatus.OFFLineBoundary + 5;
    int c_y_end   = LCDH - 40;

    if (c_y_start > c_y_end) {
        CenterLineSlope = 0.0;
    } else {
        int N = c_y_end - c_y_start + 1;
        double sum_x = 0, sum_y = 0, sum_xy = 0, sum_xx = 0;
        for (int y = c_y_start; y <= c_y_end; ++y) {
            double x = y;
            double y_val = ImageDeal[y].Center;
            sum_x += x;
            sum_y += y_val;
            sum_xy += x * y_val;
            sum_xx += x * x;
        }
        if (N > 1 && (N * sum_xx - sum_x * sum_x) != 0.0)
            CenterLineSlope = (N * sum_xy - sum_x * sum_y) / (N * sum_xx - sum_x * sum_x);
        else
            CenterLineSlope = 0.0;
    }

    // 左右边界计算范围：固定为 LCDH-50 .. LCDH-5，但只保留截止行以下的行
	int b_y_start = ImageStatus.OFFLineBoundary + 5;
    int b_y_end   = LCDH - 10;
    //int b_y_start = LCDH - 50;
    //int b_y_end   = LCDH - 5;
    if (ImageStatus.OFFLineBoundary > b_y_start)
        b_y_start = ImageStatus.OFFLineBoundary + 1;

    if (b_y_start > b_y_end) {
        LeftBoundarySlope = 0.0;
        RightBoundarySlope = 0.0;
    } else {
        int Nl = b_y_end - b_y_start + 1;
        double sum_x_l = 0, sum_y_l = 0, sum_xy_l = 0, sum_xx_l = 0;
        double sum_x_r = 0, sum_y_r = 0, sum_xy_r = 0, sum_xx_r = 0;
        for (int y = b_y_start; y <= b_y_end; ++y) {
            double x = y;
            double y_left  = ImageDeal[y].LeftBoundary_First;
            double y_right = ImageDeal[y].RightBoundary_First;
            sum_x_l  += x; sum_y_l  += y_left;  sum_xy_l  += x * y_left;  sum_xx_l  += x * x;
            sum_x_r  += x; sum_y_r  += y_right; sum_xy_r  += x * y_right; sum_xx_r  += x * x;
        }
        if (Nl > 1 && (Nl * sum_xx_l - sum_x_l * sum_x_l) != 0.0)
            LeftBoundarySlope = (Nl * sum_xy_l - sum_x_l * sum_y_l) / (Nl * sum_xx_l - sum_x_l * sum_x_l);
        else
            LeftBoundarySlope = 0.0;

        if (Nl > 1 && (Nl * sum_xx_r - sum_x_r * sum_x_r) != 0.0)
            RightBoundarySlope = (Nl * sum_xy_r - sum_x_r * sum_y_r) / (Nl * sum_xx_r - sum_x_r * sum_x_r);
        else
            RightBoundarySlope = 0.0;
    }
}

void CalcError()
{
	if (ImageStatus.OFFLineBoundary <= 50)
	{
		ImageStatus.Det_True = (ImageDeal[62].Center + ImageDeal[64].Center+ ImageDeal[66].Center) / 3 - LCDW / 2 + 1;
	}
	else
	{
		ImageStatus.Det_True = ImageDeal[ImageStatus.OFFLineBoundary + 10].Center - LCDW / 2 + 1;//5
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
	
	if(Block_Flag == 0 && Block_Done == 0)
	{
		Steer_P = 0.21;//0.21
		Steer_D = 0.15;//0.18
		SteerPID_Realize_Twice();
		return;
	}
	
	/*
	if(Block_Flag == 1 && !Blue_Block.empty())
	{
		Steer_P = 0.23;
		Steer_D = 0.14;
		SteerPID_Realize_Twice();
		return;
	}
	*/
	/*
	if(Block_Flag == 1 && Blue_Block.empty())
	{
		Steer_P = 0.12;
		Steer_D = 0.15;
		SteerPID_Realize_Twice();
		return;
	}
	*/
	
	if(Block_Flag == 1)
	{
		Steer_P = 0.21;
		Steer_D = 0.19;
		SteerPID_Realize_Twice();
		return;
	}
	if(Block_Done == 1 && Zebra_Flag == 0)
	{
		Steer_P = 0.25;
		Steer_D = 0.18;
		SteerPID_Realize_Twice();
		return;
	}
	// 3. 斑马线停车回正
	if (Zebra_Flag == 1)
	{
		Steer_P = 0.20;
		Steer_D = 0.22;
		SteerPID_Realize_Twice();
		return;
	}
	// 4. 强制变道固定打角
	//min 74 max 94 左大右小
	if (Lane_Change == 1)//0右变道
	{	
		if (Yaw_step == 0)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 74);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 94);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 104);//+10
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 64);//-10
			return;
		}
	}
	if(Lane_Change == 2)
	{
		Steer_P = 0.65;
		Steer_D = 0.20;
		SteerPID_Realize_Twice();
		return;
	}
	// 5. 锥桶引导阶段
	if (Lane_Change == 3)
	{
    	if (Yaw_step == 0)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 94);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 74);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 64);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 94);
			return;
		}
	}
	if (Lane_Change == 4 && Zebra_Flag != 4)
	{
		Steer_P = 0.25;
		Steer_D = 0.20;
		SteerPID_Realize_Twice();
		return;
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
		Steer_D = 0.21;
		SteerPID_Realize_Twice();
		return;
	}
	// 8. 正常巡线
	//Steer_P = 0.21;
	//Steer_D = 0.15;
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
		motorSet(1200); // 避障速度
		return;
	}
	// 2. 斑马线完全停车
	/*
	if (Zebra_Flag == 1) 
	{
		Zebra_time++;
		if(Zebra_time < 50)
        	motorSet(-2000);
		else
			motorSet(0);
		return;
    }
	*/
	if(SlowSpeed_Flag == 1 && Zebra_Flag == 0)
	{
		motorSet(1250);
		return;
	}
	if (Zebra_Flag == 1) 
	{
		motorSet(-2000);
		//motorSet(0);
		return;
	}
	//3. 强制变道
	if(Lane_Change == 1)
	{
		motorSet(1100);
		return;
	}
	//4. 锥桶引导变道
	if (Lane_Change == 2) 
	{
		motorSet(1100);
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
		motorSet(1500);//800
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
    //resize(Road_image, LineDisplay_image, Size(2 * LCDW, 2 * LCDH));
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

void SaveImageToVideo(const Mat& frame) 
{
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

void SaveSnapshotWithDate()
{
    using namespace std;
    using namespace cv;
    if (Original_image.empty()) return;
    Mat img_to_save = Original_image.clone();
    time_t t = time(nullptr);
    tm* tm_info = localtime(&t);
    char date_str[32];
    strftime(date_str, sizeof(date_str), "%Y-%m-%d %H:%M:%S", tm_info);
    putText(img_to_save, date_str, Point(10, 30), FONT_HERSHEY_SIMPLEX, 0.8, Scalar(0,255,0), 2);

    char filename[128];
    strftime(filename, sizeof(filename), "%Y%m%d_%H%M%S.jpg", tm_info);
    string full_path = "/home/5G/5G/picsave" + string(filename);
    imwrite(full_path, img_to_save);

    printf("Saved snapshot: %s\n", full_path.c_str());
}

void handle_sigint(int sig)
{
	printf("Ctrl+C detected, stopping motors and cleaning up...\n");
	ClearAllGPIO();
	gpioTerminate();
	destroyAllWindows();
	exit(0);
}

int a_low = 120, a_high = 180;
int b_low = 125, b_high = 180;
void createTrackbars() 
{
	namedWindow("LabRedParam", WINDOW_AUTOSIZE);
	createTrackbar("a Low", "LabRedParam", &a_low, 255);
	createTrackbar("a High", "LabRedParam", &a_high, 255);
	createTrackbar("b Low", "LabRedParam", &b_low, 255);
	createTrackbar("b High", "LabRedParam", &b_high, 255);
}


// CLAHE(L) + 非暗区统计 a/b 自适应 inRange
void AdaptiveRedMaskLab(const Mat& labImg, Mat& outBinary)
{
    int l_dark_thresh = 65; // 提高阈值，排除更多阴影
    double clahe_clip = 2.0;
    Size clahe_tile = Size(8,8);

    vector<Mat> ch;
    split(labImg, ch);
    Mat L = ch[0], A = ch[1], B = ch[2];

    Ptr<CLAHE> clahe = createCLAHE(clahe_clip, clahe_tile);
    Mat L_clahe;
    clahe->apply(L, L_clahe);

    Mat nonDarkMask = L_clahe > l_dark_thresh;
    //与原始L通道做AND，进一步排除阴影
    Mat nonDarkMask2 = L > l_dark_thresh;
    bitwise_and(nonDarkMask, nonDarkMask2, nonDarkMask);

    Scalar meanA, stdA, meanB, stdB;
    if (countNonZero(nonDarkMask) > 16) {
        meanStdDev(A, meanA, stdA, nonDarkMask);
        meanStdDev(B, meanB, stdB, nonDarkMask);
    } else {
        meanStdDev(A, meanA, stdA);
        meanStdDev(B, meanB, stdB);
    }
    double mA = meanA[0], sA = stdA[0];
    double mB = meanB[0], sB = stdB[0];

    // 下限自适应，上限固定180
    int a_low_dyn  = std::max(0, int(mA - 0.9 * sA));
    int a_high_dyn = 180;
    int b_low_dyn  = std::max(0, int(mB - 1.8 * sB));
    int b_high_dyn = 180;
    if (a_low_dyn >= a_high_dyn) a_low_dyn = std::max(0, a_high_dyn - 10);
    if (b_low_dyn >= b_high_dyn) b_low_dyn = std::max(0, b_high_dyn - 10);

    //printf("Adaptive Lab Red Thresholds: a[%d, %d], b[%d, %d]\n", a_low_dyn, a_high_dyn, b_low_dyn, b_high_dyn);

    inRange(labImg, Scalar(0, a_low_dyn, b_low_dyn), Scalar(255, a_high_dyn, b_high_dyn), outBinary);
}

void AdaptiveBlueMaskLab(const Mat& labImg, Mat& outBinary)
{
    int l_dark_thresh = 65;
    double clahe_clip = 2.0;
    Size clahe_tile = Size(8,8);

    vector<Mat> ch;
    split(labImg, ch);
    Mat L = ch[0], A = ch[1], B = ch[2];

    Ptr<CLAHE> clahe = createCLAHE(clahe_clip, clahe_tile);
    Mat L_clahe;
    clahe->apply(L, L_clahe);

    Mat nonDarkMask = L_clahe > l_dark_thresh;
    Mat nonDarkMask2 = L > l_dark_thresh;
    bitwise_and(nonDarkMask, nonDarkMask2, nonDarkMask);

    Scalar meanA, stdA, meanB, stdB;
    if (countNonZero(nonDarkMask) > 16) {
        meanStdDev(A, meanA, stdA, nonDarkMask);
        meanStdDev(B, meanB, stdB, nonDarkMask);
    } else {
        meanStdDev(A, meanA, stdA);
        meanStdDev(B, meanB, stdB);
    }
    double mA = meanA[0], sA = stdA[0];
    double mB = meanB[0], sB = stdB[0];

    // 蓝色 a 通道偏低，b 通道偏高
    int a_low_dyn  = std::max(80, int(mA - 1.2 * sA));
    int a_high_dyn = 180;
    int b_low_dyn  = std::max(80, int(mB + 1.4 * sB));
    int b_high_dyn = 180;
    if (a_low_dyn >= a_high_dyn) a_low_dyn = std::max(0, a_high_dyn - 10);
    if (b_low_dyn >= b_high_dyn) b_low_dyn = std::max(0, b_high_dyn - 10);

	printf("Adaptive Lab Blue Thresholds: a[%d, %d], b[%d, %d]\n", a_low_dyn, a_high_dyn, b_low_dyn, b_high_dyn);

    inRange(labImg, Scalar(0, a_low_dyn, b_low_dyn), Scalar(255, a_high_dyn, b_high_dyn), outBinary);
}

int main() 
{
	Start=3;
	Block_Flag=0;
	Zebra_Flag=0;		//0执行斑马线,1检测到斑马线，2斑马线停车结束,3执行变道
	Lane_Change=0;		//0从头测试,1强制变道,2锥桶引导,3开始引导，4变道结束
	Stop_Flag=0;
	uint8 warmup=0;		//相机预热计数
	Block_Done=0;		//避障完成标志
	Yaw_step = 0;		//避障阶段 0打死转向 1回正 2避障完成
	static int Time_num = 0;     		//时间计数器
	static int Slow_num = 0;

	signal(SIGINT, handle_sigint);//中断信号接收
	//createTrackbars();//滑动条

	//gpio引脚初始化
	if (gpioInitialise() < 0) 
	{
		std::cerr << "pigpio faild" << std::endl;
		return -1;
	}
	ClearAllGPIO();             // 清除全部引脚状态           
	PWMInit();                  //舵机 电机初始化

	//VideoCapture cap("/home/5G/video/5.avi");
	//VideoCapture cap("/home/5G/video/1.10.mp4");
	//VideoCapture cap("/home/5G/video/2.6.mp4");
	VideoCapture cap(0, CAP_V4L2); // 用V4L2后端打开摄像头
	if (!cap.isOpened()) 
	{
		std::cerr << "Can not open camera" << std::endl;
		return -1;
	}
	// 设置分辨率、帧率、格式（MJPG）
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT,480);
	cap.set(CAP_PROP_FPS, 120);
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

	while (cap.read(Original_image))
	{
		auto start = std::chrono::high_resolution_clock::now(); // 记录起始时间 
		if(warmup<10){warmup ++; continue;} // 预热10帧

		//上下镜像翻转
		//flip(Original_image, Original_image, 0);
		//左右镜像
		//flip(Original_image, Original_image, 1);

		//裁剪
		Rect roi_rect(0, Original_image.rows / 2, Original_image.cols, Original_image.rows / 2);
		Cropped_image = Original_image(roi_rect);

		//转Lab
		cvtColor(Cropped_image, Lab_image, COLOR_BGR2Lab);

		//resize(Lab_image, Compress_image , Size(LCDW, LCDH), 0, 0, INTER_NEAREST);

		Car_start();

		if(Start==3)
		{
			// 红色分割
			Mat Mask;
			//inRange(Lab_image, Scalar(0, a_low, b_low), Scalar(255, a_high, b_high), Mask);
			//AdaptiveRedMaskLab(Lab_image, Mask);
			AdaptiveBlueMaskLab(Lab_image, Mask);
			//AdaptiveBlueMaskLab(Compress_image, Mask);
			//AdaptiveBlueMaskLab(Compress_image, Binary_image);

			//压缩
			resize(Mask, Binary_image , Size(LCDW, LCDH), 0, 0, INTER_NEAREST);
			//反色
			//bitwise_not(Compress_image, Binary_image);
			//bitwise_not(Mask, Binary_image);

			Mat edges;
			Canny(Binary_image, edges, 28, 35, 3); // 边缘检测

			// 后续霍夫变换用膨胀后的边缘图
			vector<Vec4i> lines;
			HoughLinesP(edges, lines, 1, CV_PI/180, 28, 30, 10); 

			Road_image = Binary_image.clone();
			if (Lane_Change == 0 || Lane_Change == 4) // 非变道时用霍夫竖线
			{
				Road_image.setTo(0); // 清空road_image
				for (size_t i = 0; i < lines.size(); i++) {
					Vec4i l = lines[i];
					double angle = atan2(l[3] - l[1], l[2] - l[0]) * 180.0 / CV_PI;
					double len = sqrt(pow(l[2]-l[0],2) + pow(l[3]-l[1],2));
					// 排除横线，只显示非横线
					if (abs(angle) >= 20 && len > 35) {
						line(Road_image, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(255), 2);
					}
				}
			}
			else // 变道时直接用原始Binary_image
			{
				Binary_image.copyTo(Road_image);
			}

			// 后续巡线流程用 road_image
			//Search_Border_OTSU(Road_image, LCDH, LCDW, LCDH - 2);
			
			// 边界和中线检测流程
			Search_Border_OTSU(Binary_image, LCDH, LCDW, LCDH - 2);

			if(SlowSpeed_Flag == 0)
			{
				Slow_num++;
				//printf("Slow_num:%d\n",Slow_num);
				if(Slow_num == 300)
				{
					SlowSpeed_Flag = 1;
					Slow_num =0;
				}
			}

			if(Block_Done == 0)
			{
				Blue_Obstacles(); // 识别蓝色锥桶
				if(!Blue_Block.empty() && Block_Flag == 0)
				{
					int Blue_Block_Xsite = Blue_Block.x + Blue_Block.width / 2; // 蓝色锥桶中心X坐标
					int Blue_Block_Ysite = Blue_Block.y + Blue_Block.height/ 2; // 蓝色
					// 检查中心是否在指定的左右限制内以及Y坐标
					if (Blue_Block_Xsite > ImageDeal[Blue_Block_Ysite].LeftBoundary_First && 
						Blue_Block_Xsite <= ImageDeal[Blue_Block_Ysite].RightBoundary_First && 
						Blue_Block_Ysite > 5 && Blue_Block_Ysite< 110)
					{
						Block_Flag = 1; // 进入避障状态
						/*
						if (Blue_Block_Xsite < (ImageDeal[Blue_Block_Ysite].LeftBoundary_First + ImageDeal[Blue_Block_Ysite].RightBoundary_First) / 2)
							Left_Right_Flag = 0; // 右避障
						else 
							Left_Right_Flag = 1; // 左避障
						*/
						Left_Right_Flag = 0; // 右避障
						printf("Obstacle Detected! Left_Right_Flag:%d\n",Left_Right_Flag);
					}
				}
				if(Block_Flag == 1)
				{
					//printf("area:%d\n",Blue_Block.area());
					//printf("Blue_Block_Xsite:%d, Blue_Block_Ysite:%d\n",Blue_Block.x + Blue_Block.width / 2, Blue_Block.y + Blue_Block.height/ 2);
					if(!Blue_Block.empty())
						Time_num = 0; // 重置计数器
					else
						Time_num++;
					printf("Time_num:%d\n",Time_num);
					if(Time_num >= 2) //连续5帧未检测到蓝色
					{
						Block_Flag = 0; // 退出避障状态
						Block_Done = 1; // 避障完成
						Time_num = 0;
						//printf("Obstacle Avoidance Completed\n");
					}
				}
				//printf("Steer_P:%.2f, Steer_D:%.2f\n",Steer_P,Steer_D);
			}
			//斑马线检测
			if(Zebra_Flag== 0 && Block_Done == 1)
			{
				if(DetectZebraCrossing())
				//if(DetectZebraCrossing_Hough()) 
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
				//printf("Zebra Time_num:%d\n",Time_num);
				if(Time_num >= 100)
				{
					Time_num = 0;
					Zebra_Flag = 2;
				}
			}

			if(Zebra_Flag ==2)
			{
				int zebra_pixel_count = 0;
				for(int y = LCDH - 55; y < LCDH; ++y)
				{
					for(int x = ImageDeal[y].LeftBoundary_First + 3; x < ImageDeal[y].RightBoundary_First - 3; ++x)
					{
						if(Binary_image.at<uchar>(y, x) == 255)
						{
							zebra_pixel_count++;
						}
					}
				}
				printf("Zebra Pixel Count: %d\n", zebra_pixel_count);
				if(zebra_pixel_count <= 128)
				{
					Zebra_Flag =3;
					printf("Zebra line gone, start lane change!\n");
				}
			}

			// 斑马线后变道
			if(Zebra_Flag == 3 && Lane_Change < 4)
			{
				Blue_Obstacles(); // 继续检测蓝色锥桶
				YellowBlock();
				if(Lane_Change == 0)
				{
					Lane_Change_Flag = 1;//0右转变道 1左转变道
					Lane_Change = 1; // 标记变道中
				}
				else if (Lane_Change == 1)
				{
					if(Yaw_step == 0) // 打死转向阶段
					{
						Time_num++;
						if(Time_num >= 1)
						{
							if(CheckBoundarySlopeForYawStep())
							{
								Yaw_step = 1;
								Time_num = 0;
								printf("左右边界均已跳变，准备回正\n");
							}
						}
					}
					else if(Yaw_step == 1)
					{
						UpdateCenterLineSlope();
						printf("CenterLineSlope: %.3f, LeftBoundarySlope: %.3f, RightBoundarySlope: %.3f\n", CenterLineSlope, LeftBoundarySlope, RightBoundarySlope);
						//if (abs(CenterLineSlope) <= 0.20 && !(abs(LeftBoundarySlope) < 0.10 && abs(RightBoundarySlope) < 0.10))
						//if (abs(CenterLineSlope) <= 0.5 && !LeftBoundarySlope <= 0.1 && !RightBoundarySlope <= 0.1)
						if (abs(CenterLineSlope) <= 0.80)
						{
							Time_num++;
							int time_threshold = (Lane_Change_Flag == 0) ? 6 : 11;//右变道等于6，左变道等于10
							if(Time_num >= time_threshold)
							{
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
					int Blue_Block_Xsite = Blue_Block.x + Blue_Block.width / 2;
					int Blue_Block_Ysite = Blue_Block.y + Blue_Block.height/ 2;
					for (const auto& rect : Yellow_Block) 
					{
						int center_x = rect.x + rect.width / 2;
						int center_y = rect.y + rect.height / 2;
						
						if (center_x > ImageDeal[center_y].LeftBoundary_First + 10 &&
							center_x <= ImageDeal[center_y].RightBoundary_First - 10 &&
							center_y >= 40 && center_y < 100)
						{
						
						/*
						if (center_x >= 40 && center_x <= 120 &&
							center_y >= 75 && center_y <= 90)
						{
							*/
							printf("Find Yellow Block Center: (%d, %d)\n", center_x, center_y);
							if(Blue_Block.empty() || Blue_Block_Ysite > 60)
							{
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
					if(Yaw_step == 0)
					{
						Time_num++;
						if(Time_num >= 1)
						{
							if(CheckBoundarySlopeForYawStep())
							{
								Yaw_step = 1;
								Time_num = 0;
								printf("左右边界均已跳变，准备回正\n");
							}
						}
					}
					else if(Yaw_step == 1)
					{
						UpdateCenterLineSlope();
						printf("CenterLineSlope: %.3f, LeftBoundarySlope: %.3f, RightBoundarySlope: %.3f\n", CenterLineSlope, LeftBoundarySlope, RightBoundarySlope);
						if (abs(CenterLineSlope) <= 0.7 && !(abs(LeftBoundarySlope) <= 0.05 && abs(RightBoundarySlope) <= 0.05))
						{
							Time_num++;
							if(Time_num >= 12)
							{
								printf("Time_num:%d\n",Time_num);
								Yaw_step = 0;
								Lane_Change = 4;
								Time_num = 0;
								printf("锥桶引导：回正完成，进入停车\n");
							}
						}
					}
				}
			}
			if(Lane_Change == 4)
			{
				Time_num++;
				if(Time_num > 100)
				{
					Zebra_Flag = 4; // 标记停车状态
				}
			}
			//执行停车
			if(Zebra_Flag == 4)
			{
				Stop_left_right = 0;//1左
				DetectParkingArea();
				if (!Yellow_Parking.empty() && Stop_Flag == 0 && (LCDH - (Yellow_Parking.y + Yellow_Parking.height) < 90))
				{
					Stop_Flag = 1;
					printf("Parking Area Detected!\n");
				}
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
			}
			ShowBoundaryAndCenterLine(); //中线边线显示
			Center_Line();	//中心线修正
			CalcError();	//误差计算
			SteerControl();	//舵机调控
			//gpioPWM(12, Steer_Center);
			//MotorControl();	//电机调控

			//SaveImageToVideo(Original_image);
			//SaveImageToVideo(LineDisplay_image);
			//ZoomImage(Binary_image, 4); //放大指定图像
			//ZoomImage(Road_image, 1); //放大指定图像
			ZoomImage(LineDisplay_image, 1);//放大指定图像
			//printf("OFFLineBoundary: %d", ImageStatus.OFFLineBoundary);
			//printf(", Det_True: %.2f", ImageStatus.Det_True);
			//ZoomImage(Cropped_image, 2);//放大指定图像
			//ZoomImage(Compress_image, 4);//放大指定图像

			auto end = std::chrono::high_resolution_clock::now();   // 记录结束时间

			double frame_time = std::chrono::duration<double, std::milli>(end - start).count();
			//printf("Frame time: %.2f ms\n", frame_time);

			//每30秒保存一张图片
			static auto last_save_time = std::chrono::steady_clock::now();
			auto now = std::chrono::steady_clock::now();
			double seconds = std::chrono::duration<double>(now - last_save_time).count();
			if (seconds > 30.0) 
			{
				//SaveSnapshotWithDate();
				last_save_time = now;
			}
		}
		if (waitKey(1) == 'q') break; // 按q退出
	}
	ClearAllGPIO();
	gpioTerminate();
	cap.release();
	destroyAllWindows();
	return 0;
}