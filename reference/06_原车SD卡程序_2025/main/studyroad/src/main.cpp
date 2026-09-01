#include <opencv2/opencv.hpp>
#include <pigpio.h>
#include <signal.h>
#include <iostream>
#include <vector>
#include <cstdlib>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <deque>
#include <string>
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
#define YunTai_X 73       //云台X轴初始位置
#define YunTai_Y 75       //云台Y轴初始位置//73
#define Steer_Center 70   //舵机中值//84
#define Steer_min 60      //舵机限幅 右转 min 62
#define Steer_max 80      //舵机限幅 左转 max 102   
float Steer_P = 0.21;       //转向环参数赋值
float Steer_D = 0.8;   

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
Mat Gray_image;      	//Gray图像
Mat Compress_image; 	//压缩图像
Mat Binary_image;   	//二值图像
Mat Road_image;
Mat binary;
Mat LineDisplay_image;	//用于画边线和中线的显示图像

static int Slow_num = 0;
// 全局红色AB转向分割参数
Scalar g_lower_red1(0, 70, 170);
Scalar g_upper_red1(20, 255, 255);
Scalar g_lower_red2(160, 40, 150);
Scalar g_upper_red2(180, 255, 255);
// 全局蓝色AB转向分割参数
Scalar g_lower_blue(100, 10, 80);//60
Scalar g_upper_blue(140, 255, 255);//255 220

double CenterLineSlope = 0.0; // 全局中线斜率
double LeftBoundarySlope = 0.0;   // 左边界斜率
double RightBoundarySlope = 0.0;  // 右边界斜率

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
	
	if(SlowSpeed_Flag == 0 && Block_Done == 1)
	{
		for (int Ysite = LCDH - 1; Ysite > ImageStatus.OFFLineBoundary + 1; Ysite--)
        {
        	ImageDeal[Ysite].Center = (ImageDeal[Ysite].LeftBoundary_First + ImageDeal[Ysite].RightBoundary_First) / 2 + 10;   
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
	//else if(Stop_Flag == 1 || Stop_Flag == 2)
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

	printf("Error:%d\n",iError);
	printf("PWM:%d\n",PWM);
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

//蓝色锥桶
Rect Blue() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
	Mat lab, mask;
 	cvtColor(Cropped_image, lab, COLOR_BGR2Lab);
    Scalar lower_Blue(0, 130, 40); // 蓝色Lab下界
    Scalar upper_Blue(255, 255, 100); // 蓝色Lab上界
    inRange(lab, lower_Blue, upper_Blue, mask);
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

//红色锥桶
Rect Red() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Mat hsv, mask;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    
    // 红色的HSV范围（分为两部分）
    Scalar lower_R1(0, 100, 100);
    Scalar upper_R1(10, 255, 255);    
    Scalar lower_R2(160, 100, 100);
    Scalar upper_R2(180, 255, 255);    
    
    Mat mask1, mask2;
    inRange(hsv, lower_R1, upper_R1, mask1);
    inRange(hsv, lower_R2, upper_R2, mask2);
    
    // 合并两个mask
    mask = mask1 | mask2;
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

//红色转向AB标志
Rect Red_Flag() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Mat hsv, mask;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);

    Mat mask1, mask2;
    inRange(hsv, g_lower_red1, g_upper_red1, mask1);
    inRange(hsv, g_lower_red2, g_upper_red2, mask2);

    mask = mask1 | mask2;
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

std::string recognizeAB_and_vote()
{
    if (Cropped_image.empty()) return "Unknown";

    // 红色分割（HSV双区间）
    Mat hsv;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    Mat mask1, mask2, mask;
    inRange(hsv, g_lower_red1, g_upper_red1, mask1);
    inRange(hsv, g_lower_red2, g_upper_red2, mask2);
    mask = mask1 | mask2;

    // 找最大轮廓
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int max_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    if (max_idx == -1) return "Unknown";

    // 最小外接旋转矩形
    RotatedRect rect = minAreaRect(contours[max_idx]);
    Point2f box[4];
    rect.points(box);

    // 创建掩码：外层矩形填白，内层多边形填黑
    Mat fill_mask = Mat::zeros(mask.size(), CV_8UC1);
    std::vector<Point> box_pts(4);
    for (int i = 0; i < 4; ++i) box_pts[i] = box[i];
    std::vector<std::vector<Point>> box_vec{box_pts};
    drawContours(fill_mask, box_vec, 0, Scalar(255), FILLED);
    drawContours(fill_mask, contours, max_idx, Scalar(0), FILLED);

    // 叠加掩码到原图
    Mat mask_bgr;
    cvtColor(mask, mask_bgr, COLOR_GRAY2BGR);
    Mat show_img = mask_bgr.clone();
    show_img.setTo(Scalar(255,255,255), fill_mask);

    // 透视变换
    int w = int(rect.size.width);
    int h = int(rect.size.height);
    if (w <= 0 || h <= 0) return "Unknown";
    Point2f src_pts[4], dst_pts[4];
    for (int i = 0; i < 4; ++i) src_pts[i] = box[i];
    dst_pts[0] = Point2f(0, h-1);
    dst_pts[1] = Point2f(0, 0);
    dst_pts[2] = Point2f(w-1, 0);
    dst_pts[3] = Point2f(w-1, h-1);
    Mat M = getPerspectiveTransform(src_pts, dst_pts);
    Mat roi_bin;
    warpPerspective(show_img, roi_bin, M, Size(w, h));
    resize(roi_bin, roi_bin, Size(200, 200));

    // 反色并转为单通道
    Mat roi_gray, roi_img;
    cvtColor(roi_bin, roi_gray, COLOR_BGR2GRAY);
    bitwise_not(roi_gray, roi_img);
	rotate(roi_img, roi_img, ROTATE_90_COUNTERCLOCKWISE); // 逆时针旋转90度
	//imshow("Red Mask", show_img);
	//imshow("ROI Image", roi_img);
    // --- AB识别：横向扫线统计峰值 ---
    Mat roi_bin2;
    threshold(roi_img, roi_bin2, 127, 255, THRESH_BINARY);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(roi_bin2, roi_bin2, MORPH_OPEN, kernel);
    morphologyEx(roi_bin2, roi_bin2, MORPH_CLOSE, kernel);

    std::vector<int> row_sum(roi_bin2.rows, 0);
    for (int i = 0; i < roi_bin2.rows; ++i)
        row_sum[i] = countNonZero(roi_bin2.row(i));
    int threshold_row = int(0.3 * *std::max_element(row_sum.begin(), row_sum.end()));
    int min_peak_height = int(0.2 * roi_bin2.cols);
    std::vector<std::pair<int, int>> filtered_peaks;
    bool in_peak = false;
    int peak_start = 0;
    for (int i = 0; i < roi_bin2.rows; ++i) {
        if (row_sum[i] > threshold_row && !in_peak) {
            peak_start = i;
            in_peak = true;
        } else if ((row_sum[i] <= threshold_row || i == roi_bin2.rows - 1) && in_peak) {
            int peak_end = (row_sum[i] <= threshold_row) ? i - 1 : i;
            int peak_width = peak_end - peak_start + 1;
            int peak_max = *std::max_element(row_sum.begin() + peak_start, row_sum.begin() + peak_end + 1);
            if (peak_width > 3 && peak_max > min_peak_height)
                filtered_peaks.push_back({peak_start, peak_end});
            in_peak = false;
        }
    }
    int peak_count = filtered_peaks.size();

    // 结合空洞数和面积辅助判断
    std::vector<std::vector<Point>> holes;
    std::vector<Vec4i> hierarchy;
    findContours(roi_bin2, holes, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    int hole_count = 0;
    int min_hole_area = 100;
    if (!hierarchy.empty()) {
        for (size_t i = 0; i < hierarchy.size(); ++i) {
            if (hierarchy[i][3] != -1) {
                double area = contourArea(holes[i]);
                if (area > min_hole_area)
                    hole_count++;
            }
        }
    }
	printf("Peak count: %d, Hole count: %d\n", peak_count, hole_count);
    // 综合判决
    if (peak_count == 1 && hole_count == 1) {
        printf("识别为：A\n");
        return "A";
    } else if (peak_count == 2 && hole_count == 2) {
        printf("识别为：B\n");
        return "B";
    } else {
        printf("0\n");
        return "Unknown";
    }
}

std::string recognizeLeftRightByWhiteBar()
{
    if (Cropped_image.empty()) return "Unknown";

    // 红色分割（HSV双区间）
    Mat hsv;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    Mat mask1, mask2, mask;
    inRange(hsv, g_lower_red1, g_upper_red1, mask1);
    inRange(hsv, g_lower_red2, g_upper_red2, mask2);
    mask = mask1 | mask2;

    // 找最大轮廓
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int max_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        double area = contourArea(contours[i]);
        if (area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    if (max_idx == -1) return "Unknown";

    // 最小外接旋转矩形
    RotatedRect rect = minAreaRect(contours[max_idx]);
    Point2f box[4];
    rect.points(box);

    // 创建掩码：外层矩形填白，内层多边形填黑
    Mat fill_mask = Mat::zeros(mask.size(), CV_8UC1);
    std::vector<Point> box_pts(4);
    for (int i = 0; i < 4; ++i) box_pts[i] = box[i];
    std::vector<std::vector<Point>> box_vec{box_pts};
    drawContours(fill_mask, box_vec, 0, Scalar(255), FILLED);
    drawContours(fill_mask, contours, max_idx, Scalar(0), FILLED);

    // 叠加掩码到原图
    Mat mask_bgr;
    cvtColor(mask, mask_bgr, COLOR_GRAY2BGR);
    Mat show_img = mask_bgr.clone();
    show_img.setTo(Scalar(255,255,255), fill_mask);

    // 透视变换
    int w = int(rect.size.width);
    int h = int(rect.size.height);
    if (w <= 0 || h <= 0) return "Unknown";
    Point2f src_pts[4], dst_pts[4];
    for (int i = 0; i < 4; ++i) src_pts[i] = box[i];
    dst_pts[0] = Point2f(0, h-1);
    dst_pts[1] = Point2f(0, 0);
    dst_pts[2] = Point2f(w-1, 0);
    dst_pts[3] = Point2f(w-1, h-1);
    Mat M = getPerspectiveTransform(src_pts, dst_pts);
    Mat roi_bin;
    warpPerspective(show_img, roi_bin, M, Size(w, h));
    resize(roi_bin, roi_bin, Size(200, 200));

    // 反色并转为单通道
    Mat roi_gray, roi_img;
    cvtColor(roi_bin, roi_gray, COLOR_BGR2GRAY);
    bitwise_not(roi_gray, roi_img);
	//imshow("Red Mask", show_img);
	//imshow("roi_img", roi_img);
    // 以竖直白条为基准分割左右
    int h2 = roi_img.rows, w2 = roi_img.cols;
    cv::Mat col_sum;
    reduce(roi_img == 255, col_sum, 0, REDUCE_SUM, CV_32S);
    int max_col = 0;
    for (int i = 0; i < col_sum.cols; ++i)
        if (col_sum.at<int>(0, i) > max_col) max_col = col_sum.at<int>(0, i);
    int threshold = int(0.7 * max_col);
    std::vector<int> white_bar_indices;
    for (int i = 0; i < col_sum.cols; ++i)
        if (col_sum.at<int>(0, i) > threshold) white_bar_indices.push_back(i);
    int bar_center = w2 / 2;
    if (!white_bar_indices.empty())
        bar_center = (white_bar_indices.front() + white_bar_indices.back()) / 2;

    Mat left_top = roi_img(Range(0, h2/2), Range(0, bar_center));
    Mat right_top = roi_img(Range(0, h2/2), Range(bar_center, w2));
    double left_white_ratio = countNonZero(left_top) / double(left_top.total());
    double right_white_ratio = countNonZero(right_top) / double(right_top.total());

    if (left_white_ratio <= 0.05 && right_white_ratio <= 0.05) {
        return "Unknown";
    } else if (left_white_ratio > right_white_ratio) {
        printf("以竖条为界，左上白色比例高(%.2f > %.2f)，左转\n", left_white_ratio, right_white_ratio);
        return "L";
    } else if (right_white_ratio > left_white_ratio) {
        printf("以竖条为界，右上白色比例高(%.2f > %.2f)，右转\n", right_white_ratio, left_white_ratio);
        return "R";
    } else {
        return "Unknown";
    }
}

//蓝色转向AB标志
Rect Blue_Flag() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Mat hsv, mask;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    inRange(hsv, g_lower_blue, g_upper_blue, mask);

    threshold(mask, mask, 127, 255, THRESH_BINARY);
    //imshow("Blue Mask", mask);
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

std::string recognizeBlueAB_and_vote()
{
    if (Cropped_image.empty()) return "Unknown";

    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Mat hsv, mask;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    inRange(hsv, g_lower_blue, g_upper_blue, mask);

    // 找底行小于220且面积最大的轮廓
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int max_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect rect = boundingRect(contours[i]);
        int bottom = rect.y + rect.height;
        double area = contourArea(contours[i]);
        if (bottom < 220 && bottom > 120 && area > max_area) {
            max_area = area;
            max_idx = i;
        }
    }
    if (max_idx == -1) return "Unknown";

    // 最小外接旋转矩形
    RotatedRect rect = minAreaRect(contours[max_idx]);
    Point2f box[4];
    rect.points(box);

    // 创建掩码：外层矩形填白，内层多边形填黑
    Mat fill_mask = Mat::zeros(mask.size(), CV_8UC1);
    std::vector<Point> box_pts(4);
    for (int i = 0; i < 4; ++i) box_pts[i] = box[i];
    std::vector<std::vector<Point>> box_vec{box_pts};
    drawContours(fill_mask, box_vec, 0, Scalar(255), FILLED);
    drawContours(fill_mask, contours, max_idx, Scalar(0), FILLED);

    // 叠加掩码到原图
    Mat mask_bgr;
    cvtColor(mask, mask_bgr, COLOR_GRAY2BGR);
    Mat show_img = mask_bgr.clone();
    show_img.setTo(Scalar(255,255,255), fill_mask);

    // 透视变换
    int w = int(rect.size.width);
    int h = int(rect.size.height);
    if (w <= 0 || h <= 0) return "Unknown";
    Point2f src_pts[4], dst_pts[4];
    for (int i = 0; i < 4; ++i) src_pts[i] = box[i];
    dst_pts[0] = Point2f(0, h-1);
    dst_pts[1] = Point2f(0, 0);
    dst_pts[2] = Point2f(w-1, 0);
    dst_pts[3] = Point2f(w-1, h-1);
    Mat M = getPerspectiveTransform(src_pts, dst_pts);
    Mat roi_bin;
    warpPerspective(show_img, roi_bin, M, Size(w, h));
    resize(roi_bin, roi_bin, Size(200, 200));

    // 反色并转为单通道
    Mat roi_gray, roi_img;
    cvtColor(roi_bin, roi_gray, COLOR_BGR2GRAY);
    bitwise_not(roi_gray, roi_img);
    rotate(roi_img, roi_img, ROTATE_90_COUNTERCLOCKWISE);

    // --- AB识别：横向扫线统计峰值 ---
    Mat roi_bin2;
    threshold(roi_img, roi_bin2, 127, 255, THRESH_BINARY);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
    morphologyEx(roi_bin2, roi_bin2, MORPH_OPEN, kernel);
    morphologyEx(roi_bin2, roi_bin2, MORPH_CLOSE, kernel);

    std::vector<int> row_sum(roi_bin2.rows, 0);
    for (int i = 0; i < roi_bin2.rows; ++i)
        row_sum[i] = countNonZero(roi_bin2.row(i));
    int threshold_row = int(0.3 * *std::max_element(row_sum.begin(), row_sum.end()));
    int min_peak_height = int(0.2 * roi_bin2.cols);
    std::vector<std::pair<int, int>> filtered_peaks;
    bool in_peak = false;
    int peak_start = 0;
    for (int i = 0; i < roi_bin2.rows; ++i) {
        if (row_sum[i] > threshold_row && !in_peak) {
            peak_start = i;
            in_peak = true;
        } else if ((row_sum[i] <= threshold_row || i == roi_bin2.rows - 1) && in_peak) {
            int peak_end = (row_sum[i] <= threshold_row) ? i - 1 : i;
            int peak_width = peak_end - peak_start + 1;
            int peak_max = *std::max_element(row_sum.begin() + peak_start, row_sum.begin() + peak_end + 1);
            if (peak_width > 3 && peak_max > min_peak_height)
                filtered_peaks.push_back({peak_start, peak_end});
            in_peak = false;
        }
    }
    int peak_count = filtered_peaks.size();

    // 结合空洞数和面积辅助判断
    std::vector<std::vector<Point>> holes;
    std::vector<Vec4i> hierarchy;
    findContours(roi_bin2, holes, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);
    int hole_count = 0;
    int min_hole_area = 100;
    if (!hierarchy.empty()) {
        for (size_t i = 0; i < hierarchy.size(); ++i) {
            if (hierarchy[i][3] != -1) {
                double area = contourArea(holes[i]);
                if (area > min_hole_area)
                    hole_count++;
            }
        }
    }
    printf("Blue AB Peak count: %d, Hole count: %d\n", peak_count, hole_count);
    // 综合判决
    if (peak_count == 1 && hole_count == 1) {
        printf("蓝色识别为：A\n");
        return "A";
    } else if (peak_count == 1 && hole_count == 2) {//22
        printf("蓝色识别为：B\n");
        return "B";
    } else {
        printf("蓝色识别为：Unknown\n");
        return "Unknown";
    }
}

std::string recognizeLeftRightByBlueBar()
{
    if (Cropped_image.empty()) return "Unknown";

    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
    Mat hsv, mask;
    cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);
    inRange(hsv, g_lower_blue, g_upper_blue, mask);

    // 找底行小于220且面积最大的轮廓
    std::vector<std::vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int max_idx = -1;
    double max_area = 0;
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect rect = boundingRect(contours[i]);
        int bottom = rect.y + rect.height;
        double area = contourArea(contours[i]);
        if (bottom < 220 && bottom > 180 && area > max_area && area > 5000) {
            max_area = area;
            max_idx = i;
        }
    }
    if (max_idx == -1) return "Unknown";

    // 最小外接旋转矩形
    RotatedRect rect = minAreaRect(contours[max_idx]);
    Point2f box[4];
    rect.points(box);

    // 创建掩码：外层矩形填白，内层多边形填黑
    Mat fill_mask = Mat::zeros(mask.size(), CV_8UC1);
    std::vector<Point> box_pts(4);
    for (int i = 0; i < 4; ++i) box_pts[i] = box[i];
    std::vector<std::vector<Point>> box_vec{box_pts};
    drawContours(fill_mask, box_vec, 0, Scalar(255), FILLED);
    drawContours(fill_mask, contours, max_idx, Scalar(0), FILLED);

    // 叠加掩码到原图
    Mat mask_bgr;
    cvtColor(mask, mask_bgr, COLOR_GRAY2BGR);
    Mat show_img = mask_bgr.clone();
    show_img.setTo(Scalar(255,255,255), fill_mask);

    // 透视变换
    int w = int(rect.size.width);
    int h = int(rect.size.height);
    if (w <= 0 || h <= 0) return "Unknown";
    Point2f src_pts[4], dst_pts[4];
    for (int i = 0; i < 4; ++i) src_pts[i] = box[i];
    dst_pts[0] = Point2f(0, h-1);
    dst_pts[1] = Point2f(0, 0);
    dst_pts[2] = Point2f(w-1, 0);
    dst_pts[3] = Point2f(w-1, h-1);
    Mat M = getPerspectiveTransform(src_pts, dst_pts);
    Mat roi_bin;
    warpPerspective(show_img, roi_bin, M, Size(w, h));
    resize(roi_bin, roi_bin, Size(200, 200));

    // 反色并转为单通道
    Mat roi_gray, roi_img;
    cvtColor(roi_bin, roi_gray, COLOR_BGR2GRAY);
    bitwise_not(roi_gray, roi_img);

    // 以竖直白条为基准分割左右
    int h2 = roi_img.rows, w2 = roi_img.cols;
    cv::Mat col_sum;
    reduce(roi_img == 255, col_sum, 0, REDUCE_SUM, CV_32S);
    int max_col = 0;
    for (int i = 0; i < col_sum.cols; ++i)
        if (col_sum.at<int>(0, i) > max_col) max_col = col_sum.at<int>(0, i);
    int threshold = int(0.7 * max_col);
    std::vector<int> white_bar_indices;
    for (int i = 0; i < col_sum.cols; ++i)
        if (col_sum.at<int>(0, i) > threshold) white_bar_indices.push_back(i);
    int bar_center = w2 / 2;
    if (!white_bar_indices.empty())
        bar_center = (white_bar_indices.front() + white_bar_indices.back()) / 2;

    Mat left_top = roi_img(Range(0, h2/2), Range(0, bar_center));
    Mat right_top = roi_img(Range(0, h2/2), Range(bar_center, w2));
    double left_white_ratio = countNonZero(left_top) / double(left_top.total());
    double right_white_ratio = countNonZero(right_top) / double(right_top.total());

    if (left_white_ratio <= 0.05 && right_white_ratio <= 0.05) {
        return "Unknown";
    } else if (left_white_ratio > right_white_ratio && left_white_ratio < 0.3) {
        printf("蓝色竖条为界，左上白色比例高(%.2f > %.2f)，左转\n", left_white_ratio, right_white_ratio);
        return "L";
    } else if (right_white_ratio > left_white_ratio && right_white_ratio < 0.3) {
        printf("蓝色竖条为界，右上白色比例高(%.2f > %.2f)，右转\n", right_white_ratio, left_white_ratio);
        return "R";
    } else {
        return "Unknown";
    }
}

vector<Rect> YellowBlock() 
{
    Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
	Mat hsv, mask;
 	cvtColor(Cropped_image, hsv, COLOR_BGR2HSV);

    // 常用黄色 HSV 范围（可根据实际场景调节）
    //Scalar lower_Y(30, 10, 100);   // H:20 S:100 V:100
    //Scalar upper_Y(55, 255, 255);   // H:35 S:255 V:255

	Scalar lower_Y(15, 60, 100);   // H:20 S:100 V:100
    Scalar upper_Y(35, 255, 255);   

    inRange(hsv, lower_Y, upper_Y, mask);
    threshold(mask, mask, 127, 255, THRESH_BINARY);
    vector<vector<Point>> contours;
    findContours(mask, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    Yellow_Block.clear();
    for (size_t i = 0; i < contours.size(); ++i) {
        Rect r = boundingRect(contours[i]);
        if (r.area() > 20) {
            int x = (int)(r.x / (float)Cropped_image.cols * LCDW + 0.5);
            int y = (int)(r.y / (float)Cropped_image.rows * LCDH + 0.5);
            int w = (int)(r.width / (float)Cropped_image.cols * LCDW + 0.5);
            int h = (int)(r.height / (float)Cropped_image.rows * LCDH + 0.5);
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
    Scalar lower_Y(10, 70, 100);   // H:20 S:100 V:100
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
			int x = (int)(r.x / (float)Cropped_image.cols * LCDW + 0.5);
			int y = (int)(r.y / (float)Cropped_image.rows * LCDH + 0.5);
			int w = (int)(r.width / (float)Cropped_image.cols * LCDW + 0.5);
			int h = (int)(r.height / (float)Cropped_image.rows * LCDH + 0.5);
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
    //Rect obstacle_object = Red(); // 识别到的蓝色物体
	Rect obstacle_object = Blue(); // 识别到的蓝色物体
    //printf("width: %d\n", obstacle_object.width);
    //printf("height: %d\n", obstacle_object.height);
    if (obstacle_object.area()>10) // 对识别到的蓝色物体限幅
    {
        // 压缩到 LCDW × LCDH 坐标系
        int x = (int)(obstacle_object.x / (float)Cropped_image.cols * LCDW + 0.5);
        int y = (int)(obstacle_object.y / (float)Cropped_image.rows * LCDH + 0.5);
        int w = (int)(obstacle_object.width / (float)Cropped_image.cols * LCDW + 0.5);
        int h = (int)(obstacle_object.height / (float)Cropped_image.rows * LCDH + 0.5);
        Blue_Block = Rect(x, y, w, h);
    }
}

bool DetectZebraCrossing()
{
    // 1. 图像预处理（与Hough方法一致）
    Mat zebra_img;
    resize(Cropped_image, zebra_img, Size(800, 400));
    Mat gray_zebra;
    cvtColor(zebra_img, gray_zebra, COLOR_BGR2GRAY);
    Mat blur_zebra;
    GaussianBlur(gray_zebra, blur_zebra, Size(5, 5), 5);
    Mat thresh_zebra;
    threshold(blur_zebra, thresh_zebra, 200, 255, THRESH_BINARY);
    Mat kernel_ero = getStructuringElement(MORPH_RECT, Size(3, 1));
    Mat img_ero;
    erode(thresh_zebra, img_ero, kernel_ero, Point(-1, -1), 3);
    Mat kernel_dia = getStructuringElement(MORPH_RECT, Size(5, 1));
    Mat img_dia;
    dilate(img_ero, img_dia, kernel_dia, Point(-1, -1), 1);

    // 2. 跳变计数逻辑（在处理后的图像上）
    int net = 0;
    int NUM = 0;
    // 只在图像上部区域统计（可根据实际情况调整行数）
    for (int Ysite = 50; Ysite < 85; Ysite++) // 20 55
    {
		int left = (ImageDeal[Ysite].LeftBoundary_First + 1)*5;
    	int right = (ImageDeal[Ysite].RightBoundary_First - 1)*5;
        for (int Xsite = left; Xsite < right; Xsite++) // 800宽，左右各缩进80
        {
            if (img_dia.at<uchar>(Ysite, Xsite) == 0 && img_dia.at<uchar>(Ysite, Xsite + 1) == 255)
            {
                int Xstart = Xsite;
                net = 0;
                while (img_dia.at<uchar>(Ysite, Xstart + 1) == 255)
                {
                    if (img_dia.at<uchar>(Ysite, Xstart) == 255)
                    {
                        net++;
                    }
                    Xstart++;
                }
                //printf("net:%d\n", net);
                if (net >= 10) // 由于分辨率变大，阈值适当提高
                {
                    NUM++;
                    Xsite = Xstart;
                }
            }
        }
    }
    printf("Zebra NUM: %d\n", NUM);
    if (NUM >= 100) // 阈值也适当提高80 70
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

	//imshow("Cropped Image", Cropped_image);
	//imshow("Gray Zebra", gray_zebra);
	//imshow("Thresholded Zebra", thresh_zebra);
	//imshow("Blurred Zebra", blur_zebra);
    //imshow("Zebra Contours", img_dia);

    // 统计宽度和高度符合要求且下边界小于200的轮廓数量
	int count = 0;
	bool all_in_range = true;
	int max_bottom = 0;
	for (size_t i = 0; i < contours.size(); ++i)
	{
		Rect rect = boundingRect(contours[i]);
		if (rect.width > 10 && rect.height > 20)
		{
			count++;
			int bottom = rect.y + rect.height;
			if (bottom > max_bottom)
				max_bottom = bottom;
			if (!(bottom >= 30 && bottom <= 300))
				all_in_range = false; // 只要有一个不在范围内就失败
		}
	}
	printf("Zebra stripe count: %d, Max bottom line: %d\n", count, max_bottom);
	// 所有都在范围内才返回true
	return (count >= 6) && all_in_range;
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
	if(frame_count >= 8)
	{
		if (Lane_Change_Flag == 0)
		{
			if (Lane_Change == 1 && frame_count >=20)
			{
				if (!Right_Change && fabs(right_slope - last_right_slope) >= 0.15)
				{
					Right_Change = true;
					
				}
				if (Right_Change && !Left_Change && fabs(left_slope - last_left_slope) >= 0.15)
					Left_Change = true;
			}
			else if (Lane_Change == 3 && frame_count >= 16)
			{
				if (!Left_Change && fabs(left_slope - last_left_slope) >= 0.15)
				{
					Left_Change = true;
					
				}
				if (Left_Change && !Right_Change && fabs(right_slope - last_right_slope) >= 0.15)
					Right_Change = true;
			}
		}
		// 左变道
		else if (Lane_Change_Flag == 1)
		{
			if (Lane_Change == 1 && frame_count >= 22)
			{
				if (!Left_Change && fabs(left_slope - last_left_slope) >= 0.15)
				{
					Left_Change = true;
					
				}
				if (Left_Change && !Right_Change && fabs(right_slope - last_right_slope) >= 0.15)
					Right_Change = true;
			}
			else if (Lane_Change == 3 && frame_count >= 12)
			{
				if (!Right_Change && fabs(right_slope - last_right_slope) >= 0.15)
				{
					Right_Change = true;
					
				}
				if (Right_Change && !Left_Change && fabs(left_slope - last_left_slope) >= 0.15)
					Left_Change = true;
			}
		}
	}
    printf("LeftSlope: %.3f, RightSlope: %.3f, LeftChange: %d, RightChange: %d\n",
        left_slope, right_slope, Left_Change, Right_Change);

    last_left_slope = left_slope;
    last_right_slope = right_slope;

	if ((Lane_Change == 1 && Left_Change && Right_Change) ||
		(Lane_Change == 3 && (Left_Change || Right_Change)))
	{
		slope_count++;
		int slope_delay = 1; // 默认延时
		if (Lane_Change == 1 && Lane_Change_Flag == 0)
			slope_delay = 1; // 右变道第一阶段
		else if (Lane_Change == 1 && Lane_Change_Flag == 1)
			slope_delay = 1; // 左变道第一阶段
		else if (Lane_Change == 3 && Lane_Change_Flag == 0)
			slope_delay = 3; // 右变道锥桶阶段
		else if (Lane_Change == 3 && Lane_Change_Flag == 1)
			slope_delay = 3; // 左变道锥桶阶段

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
		Steer_D = 0.8;//0.18
		SteerPID_Realize_Twice();
		return;
	}
	if(Block_Flag == 1)
	{
		Steer_P = 0.21;
		Steer_D = 0.80;
		SteerPID_Realize_Twice();
		return;
	}
	if(Block_Done == 1 && Zebra_Flag == 0 && SlowSpeed_Flag == 0)
	{
		Steer_P = 0.21;
		Steer_D = 0.8;//0.8
		SteerPID_Realize_Twice();
		return;
	}
	if(Zebra_Flag == 0 && SlowSpeed_Flag == 1)
	{
			Steer_P = 0.21;
			Steer_D = 0.8;//0.8
			SteerPID_Realize_Twice();
			return;
	}
	// 3. 斑马线停车回正
	if (Zebra_Flag == 1)
	{
		Steer_P = 0.21;
		Steer_D = 0.80;
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
				gpioPWM(12, 60);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 78);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 90);//+10
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 50);//-10
			return;
		}
	}
	if(Lane_Change == 2)
	{
		Steer_P = 0.21;
		Steer_D = 0.8;
		SteerPID_Realize_Twice();
		return;
	}
	// 5. 锥桶引导阶段//100 0右1左
	if (Lane_Change == 3)
	{
    	if (Yaw_step == 0)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 84);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 52);
			return;
		}
		else if (Yaw_step == 1)
		{
			if(Lane_Change_Flag == 0)
				gpioPWM(12, 50);
			else if(Lane_Change_Flag == 1)
				gpioPWM(12, 80);
			return;
		}
	}
	if (Lane_Change == 4 && Zebra_Flag != 4)
	{
		Steer_P = 0.21;
		Steer_D = 0.80;
		SteerPID_Realize_Twice();
		return;
	}
	if(Zebra_Flag == 4)//直道
	{
		Steer_P = 0.21;
		Steer_D = 0.80;
		SteerPID_Realize_Twice();
		return;
	}
	//7. 停车状态
	if(Stop_Flag == 1 && Stop_left_right == 0) 
	{
		Steer_P = 0.30;
		Steer_D = 0.14;
		SteerPID_Realize_Twice();
		return;
	}

	if(Stop_Flag == 1 && Stop_left_right == 1) 
	{
		Steer_P = 0.30;
		Steer_D = 0.14;
		SteerPID_Realize_Twice();
		return;
	}

	if(Stop_Flag == 2)
	{
		/*
		Steer_P = 0.80;
		Steer_D = 0.20;
		SteerPID_Realize_Twice();
		*/
		gpioPWM(12, Steer_Center);
		return;
	}
	if(Stop_Flag == 3)
	{
		
		Steer_P = 0.60;
		Steer_D = 0.21;
		SteerPID_Realize_Twice();
		
		//gpioPWM(12, Steer_Center);
		return;
	}
	// 8. 正常巡线
	Steer_P = 0.21;
	Steer_D = 0.80;
	SteerPID_Realize_Twice();
}

//750勉强起步
void MotorControl()//电机调控
{
	// 1. 避障
	if (Block_Flag == 1) 
	{
		motorSet(1800); // 避障速度
		return;
	}
	if(SlowSpeed_Flag == 1 && Zebra_Flag == 0)
	{
		motorSet(1600);
		return;
	}
		// 2. 斑马线完全停车
	if (Zebra_Flag == 1) 
	{
		motorSet(-3500);
		//usleep(500);
		//motorSet(0);
		//usleep(500);
		//motorSet(-1000);
		return;
	}
	if(Zebra_Flag == 2)
	{
		motorSet(1100);
		return;
	}
	//3. 强制变道
	if(Lane_Change == 1)
	{
		motorSet(1600);
		return;
	}
	//4. 锥桶引导变道
	if (Lane_Change == 2) 
	{
		motorSet(1600);
		return;
	}
	if	(Lane_Change == 3)
	{
		motorSet(1600);
		return;
	}
	if(Zebra_Flag == 4 && SlowSpeed_Flag == 1)
	{
		motorSet(1800);
		return;
	}
	if(Zebra_Flag == 4 && SlowSpeed_Flag == 2)
	{
		motorSet(1400);
		return;
	}
	//5. 停车区域引导
	if(Stop_Flag == 1 || Stop_Flag == 2 || Stop_Flag == 3)
	{
		motorSet(1150);
		return;
	}
	if(Stop_Flag ==	4)
	{
		motorSet(-2000);
        return;
	}
	// 6. 正常巡线
	if (Start == 3) 
	{
		motorSet(1850);//800
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
    string full_path = "/home/5G/5G/picsave/picsave" + string(filename);
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

int main() 
{
	// 启动前输入变道标志和停车方向
    std::cout << "请输入变道和停车标志(两位数,如01,前一位为变道,后一位为停车,0右1左):";
    int input_flag;
    std::cin >> input_flag;
    Lane_Change_Flag = input_flag / 10;    // 十位为变道标志
    Stop_left_right  = input_flag % 10;    // 个位为停车标志

	Start=0;
	Block_Flag=0;
	Zebra_Flag=0;		//0执行斑马线,1检测到斑马线，2斑马线停车结束,3执行变道,4执行停车
	Lane_Change=0;		//0从头测试,1强制变道,2锥桶引导,3开始引导，4变道结束,5开始停车
	Stop_Flag=0;
	SlowSpeed_Flag=0;	//
	uint8 warmup=0;		//相机预热计数
	Block_Done=1;		//避障完成标志
	Yaw_step = 0;		//避障阶段 0打死转向 1回正 2避障完成
	bool allow_zebra_detect = false;
	bool allow_stop_detect = false;
	static int Time_num = 0;     		//时间计数器
	static int limit_time = 0;

	int YunTai_X_val = YunTai_X;
	int YunTai_Y_val = YunTai_Y;
	bool trackbar_created = false;

	signal(SIGINT, handle_sigint);//中断信号接收

	//gpio引脚初始化
	if (gpioInitialise() < 0) 
	{
		std::cerr << "pigpio faild" << std::endl;
		return -1;
	}
	ClearAllGPIO();             // 清除全部引脚状态           
	PWMInit();                  //舵机 电机初始化

	//蓝色
	//string tflite_path = "/home/5G/5G/main/studyroad/src/STRONG blue-ground.v2(10184).tflite";
	//string tflite_path = "/home/5G/5G/main/studyroad/src/STRONG blue-ground.v1(6784).tflite";
	//string tflite_path = "/home/5G/5G/main/studyroad/src/STRONG blue-ground.v3(1944).tflite";
	//红色
	string tflite_path = "/home/5G/5G/main/studyroad/src/strong-1704.tflite";
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
	//VideoCapture cap("/home/5G/video/stronglight1.mp4");
	//VideoCapture cap("/home/5G/video/stronglight1.mp4");
	VideoCapture cap(0, CAP_V4L2); // 用V4L2后端打开摄像头
	if (!cap.isOpened()) 
	{
		std::cerr << "Can not open camera" << std::endl;
		return -1;
	}
	// 设置分辨率、帧率、格式（MJPG）
	cap.set(CAP_PROP_FRAME_WIDTH, 640);
	cap.set(CAP_PROP_FRAME_HEIGHT,480);
	cap.set(CAP_PROP_FPS, 30);
	cap.set(CAP_PROP_FOURCC, VideoWriter::fourcc('M','J','P','G'));

	while (cap.read(Original_image))
	{ 
		if(warmup<20){warmup ++; continue;}//预热10帧
		auto start = std::chrono::high_resolution_clock::now(); // 记录起始时间
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
			Binary_image.create(out_h, out_w, CV_8UC1);
			for (int i = 0; i < out_h; ++i) {
				for (int j = 0; j < out_w; ++j) {
					int idx = i * out_w * out_c + j * out_c;
					int val = static_cast<int>(y[idx]);
					for (int k = 1; k < out_c; ++k) {
    				val = std::max(val, static_cast<int>(y[idx + k]));
					}
					Binary_image.at<uchar>(i, j) = val >= seg_model.OUT_THR ? 255 : 0;
				}
			}
			threshold(Binary_image, Binary_image, 127, 255, THRESH_BINARY);
			Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
			//erode(Binary_image, Binary_image, kernel, Point(-1, -1), 1);
			dilate(Binary_image, Binary_image, kernel, Point(-1, -1), 2);

			// 边界和中线检测流程
			Search_Border_OTSU(Binary_image, LCDH, LCDW, LCDH - 2);

			if(SlowSpeed_Flag == 0)
			{
				Slow_num++;
				printf("Slow_num:%d\n",Slow_num);
				if(Slow_num >= 1400)//770 700 660 630 580  开始减速的时间
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
						Blue_Block_Ysite > 20 && Blue_Block_Ysite< 110)
					{
						Block_Flag = 1; // 进入避障状态
						/*
						if (Blue_Block_Xsite < (ImageDeal[Blue_Block_Ysite].LeftBoundary_First + ImageDeal[Blue_Block_Ysite].RightBoundary_First) / 2-20)
							Left_Right_Flag = 0; // 右避障
						else 
							Left_Right_Flag = 1; // 左避障
						*/
						Left_Right_Flag = 0; // 右避障
						//Left_Right_Flag = 1; // 左避障
						//printf("Xsite:%d, Ysite:%d\n",Blue_Block_Xsite,Blue_Block_Ysite);
						//printf("ImageDeal[Blue_Block_Ysite].LeftBoundary_First:%d, ImageDeal[Blue_Block_Ysite].RightBoundary_First:%d\n",ImageDeal[Blue_Block_Ysite].LeftBoundary_First,ImageDeal[Blue_Block_Ysite].RightBoundary_First);
						printf("Obstacle Detected! Left_Right_Flag:%d\n",Left_Right_Flag);
					}
				}
				if(Block_Flag == 1)
				{
					printf("area:%d\n",Blue_Block.area());
					printf("Blue_Block_Xsite:%d, Blue_Block_Ysite:%d\n",Blue_Block.x + Blue_Block.width / 2, Blue_Block.y + Blue_Block.height/ 2);
					if(!Blue_Block.empty())
						Time_num = 0; // 重置计数器
					else
						Time_num++;
					printf("Time_num:%d\n",Time_num);
					if(Time_num >= 20) //连续5帧未检测到蓝色
					{
						Block_Flag = 0; // 退出避障状态
						Block_Done = 1; // 避障完成
						Time_num = 0;
						//printf("Obstacle Avoidance Completed\n");
					}
				}
				printf("Steer_P:%.2f, Steer_D:%.2f\n",Steer_P,Steer_D);
			}
			if(SlowSpeed_Flag == 1 && Zebra_Flag == 0 && !allow_zebra_detect)
			{
				Time_num++;
				if(Time_num >= 10)//100   减速后多少帧允许识别
				{
					allow_zebra_detect = true;
					Time_num = 0;
				}
			}
			//斑马线检测
			if(Zebra_Flag== 0 && Block_Done == 1 && allow_zebra_detect)
			{
				if(DetectZebraCrossing())
				//if(DetectZebraCrossing_Hough()) 
				{
					printf("Zebra Crossing Detected!\n");
					Zebra_Flag = 1;
				}
			}

			// 斑马线后变道
			if(Zebra_Flag == 1)
			{
				Time_num ++;
				printf("Zebra Waiting Time_num:%d\n",Time_num);
				if(Time_num == 10)
				{
					printf("准备播放语音\n");
					//system("amixer -c 2 sset 'PCM' 100%");
					system("aplay -D plughw:2,0 /home/5G/5G/voice/2.wav &"); // 加&后台播放
				}
				//printf("Zebra Time_num:%d\n",Time_num);
				if(Time_num >= 90)
				{
					Time_num = 0;
					Zebra_Flag = 2;
				}
			}

			/*
			if (Zebra_Flag == 2)
			{
				static std::deque<std::string> lr_results; // 用于统计L/R
				Rect red_flag_rect = Red_Flag();

				// 实时识别并统计
				std::string lr = recognizeLeftRightByWhiteBar();
				printf("本帧左右转识别为: %s\n", lr.c_str());
				if (lr == "L" || lr == "R") lr_results.push_back(lr);

				// 判断红色标志底部是否到达指定行数
				int bottom = red_flag_rect.y + red_flag_rect.height;
				printf("Red Flag Bottom:%d\n", bottom);

				// 到达指定行数后，统计L/R数量
				if (bottom > 200)
				{
					int l_count = std::count(lr_results.begin(), lr_results.end(), "L");
					int r_count = std::count(lr_results.begin(), lr_results.end(), "R");
					printf("L_count:%d, R_count:%d\n", l_count, r_count);

					std::string final_lr = "Unknown";
					if (l_count > r_count)
						final_lr = "L";
					else if (r_count > l_count)
						final_lr = "R";
					else if (l_count == 0 && r_count == 0)
						final_lr = (Lane_Change_Flag == 1) ? "L" : "R"; // 没有识别到，按输入值

					printf("最终左右转识别为: %s\n", final_lr.c_str());
					// 根据识别结果设置变道方向
					if(final_lr == "L")
						Lane_Change_Flag = 1; // 左变道
					else if(final_lr == "R")
						Lane_Change_Flag = 0; // 右变道

					lr_results.clear();
					Zebra_Flag = 3;
				}
			}
			*/

			if (Zebra_Flag == 2)
			{
				static std::deque<std::string> lr_results_blue; // 用于统计L/R
				static std::string final_lr = "Unknown";
				static bool locked = false;
				Rect blue_flag_rect = Blue_Flag();

				// 实时识别并统计
				std::string lr = recognizeLeftRightByBlueBar();
				printf("本帧蓝色左右转识别为: %s\n", lr.c_str());

				// 只有还没锁定时才统计
				int l_count = std::count(lr_results_blue.begin(), lr_results_blue.end(), "L");
				int r_count = std::count(lr_results_blue.begin(), lr_results_blue.end(), "R");
				if (!locked) {
					if (lr == "L" || lr == "R") lr_results_blue.push_back(lr);
					l_count = std::count(lr_results_blue.begin(), lr_results_blue.end(), "L");
					r_count = std::count(lr_results_blue.begin(), lr_results_blue.end(), "R");
					if (l_count >= 10) {
						final_lr = "L";
						locked = true;
						printf("L识别到3次，锁定为L\n");
					} else if (r_count >= 10) {
						final_lr = "R";
						locked = true;
						printf("R识别到3次，锁定为R\n");
					}
				}

				// 判断蓝色标志底部是否到达指定行数
				int bottom = blue_flag_rect.y + blue_flag_rect.height;
				printf("Blue Flag Bottom:%d\n", bottom);

				Time_num++;
				printf("Time_num:%d\n", Time_num);

				// 只有到达指定行数后才允许切换状态
				if ((bottom > 235  && Time_num >70)|| Time_num >= 90)
				{ 
					printf("L_count:%d, R_count:%d\n", l_count, r_count);
					printf("最终蓝色左右转识别为: %s\n", final_lr.c_str());
					// 根据识别结果设置变道方向
					if(final_lr == "L")
						Lane_Change_Flag = 1; // 左变道
					else if(final_lr == "R")
						Lane_Change_Flag = 0; // 右变道
					else if(final_lr == "Unknown")
						Lane_Change_Flag = (Lane_Change_Flag == 1) ? 1 : 0; // 没有识别到，按输入值
					printf("Lane_Change_Flag:%d\n",Lane_Change_Flag);

					// 状态切换和变量重置
					lr_results_blue.clear();
					final_lr = "Unknown";
					locked = false;
					Zebra_Flag = 3;
					Time_num = 0;
				}
			}

			/*
			if(Zebra_Flag == 2)
			{
				Time_num++;
				printf("Time:%d\n", Time_num);
				if(Time_num >= 20)
				{
					Zebra_Flag = 3;
					Time_num = 0;
				}
			}
			*/

			// 斑马线后变道
			if(Zebra_Flag == 3 && Lane_Change < 4)
			{
				Blue_Obstacles(); // 继续检测蓝色锥桶
				YellowBlock();
				if(Lane_Change == 0)
				{
					//Lane_Change_Flag = 0;//0右转变道 1左转变道
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
						limit_time++;
						if (abs(CenterLineSlope) <= 0.80)
						{
							Time_num++;
							int time_low = (Lane_Change_Flag == 0) ? 13 : 14;//右变道前，左变道后
							int time_high  = (Lane_Change_Flag == 0) ? 14 : 16;//右变道前，左变道后
							if(Time_num >= time_low || limit_time >= time_high)
							{
								Yaw_step = 0;        // 变道完成
								Lane_Change = 2;     // 进入下一阶段
								Time_num = 0;
								limit_time = 0;
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
							center_y >= 20 && center_y < 100)//6 6 20 100
						{
							printf("Find Yellow Block Center: (%d, %d)\n", center_x, center_y);
							printf("Blue Block Ysite:%d\n",Blue_Block_Ysite);
							//if(1)
							if(Blue_Block.empty() || Blue_Block_Ysite >= 20)
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
						limit_time++;
						if (abs(CenterLineSlope) <= 0.7)
						{
							Time_num++;
							int time_low = (Lane_Change_Flag == 0) ? 11 : 16;//右变道前，左变道后// 8 12
							int time_high  = (Lane_Change_Flag == 0) ? 16 : 25;//右变道前，左变道后//12 15
							if(Time_num >= time_low || limit_time >= time_high)
							{
								printf("Time_num:%d\n",Time_num);
								Yaw_step = 0;
								Lane_Change = 4;
								Time_num = 0;
								limit_time = 0;
								printf("锥桶引导：回正完成，进入停车\n");
							}
						}
					}
				}
			}
			if(Lane_Change == 4)
			{
				Time_num++;
				if(Time_num > 10)
				{
					Zebra_Flag = 4; // 标记停车状态
					Lane_Change = 5; // 变道完成
					Time_num = 0;
				}
			}

			if(SlowSpeed_Flag == 1 && Zebra_Flag == 4)
			{
				Slow_num++;
				printf("Slow_num:%d\n",Slow_num);
				if(Slow_num >= 200)//160,120，70,200
				{
					SlowSpeed_Flag = 2;
					Slow_num =0;
				}
			}

			if(SlowSpeed_Flag == 2 && Zebra_Flag == 4 && !allow_stop_detect)
			{
				Time_num++;
				printf("Time_num:%d\n",Time_num);
				if(Time_num >= 100)//100
				{
					allow_stop_detect = true;
					Time_num = 0;
				}
			}

			/*			
			if(Zebra_Flag == 4 && SlowSpeed_Flag == 2)
			{
				Rect red_flag_rect = Red_Flag();
				if(red_flag_rect.area() > 50)
				{
					int bottom = red_flag_rect.y + red_flag_rect.height;//640*240
					printf("Red Flag Bottom:%d\n",bottom);
					if(bottom >= 50)
					{
						Time_num++;
					}
					if(Time_num >= 3)
					{
						Zebra_Flag = 5;
						Stop_Flag = 1;
						Time_num = 0;
						printf("Red Flag Detected, Prepare to Stop!\n");
					}
				}
			}
			*/
		
			/*
			//执行停车
			if (Zebra_Flag == 4 && SlowSpeed_Flag == 2)
			{
				static std::deque<std::string> ab_results; // 用于统计A/B
				Rect red_flag_rect = Red_Flag();

				// 实时识别并统计
				std::string ab = recognizeAB_and_vote();
				printf("本帧AB识别为: %s\n", ab.c_str());
				if (ab == "A" || ab == "B") ab_results.push_back(ab);

				// 判断红色标志底部是否到达指定行数
				int bottom = red_flag_rect.y + red_flag_rect.height;
				printf("Red Flag Bottom:%d\n", bottom);

				// 到达指定行数后，统计A/B数量
				if (bottom > 200)
				{
					int a_count = std::count(ab_results.begin(), ab_results.end(), "A");
					int b_count = std::count(ab_results.begin(), ab_results.end(), "B");
					printf("A_count:%d, B_count:%d\n", a_count, b_count);

					std::string final_ab = "Unknown";
					if (a_count > b_count)
						final_ab = "A";
					else if (b_count > a_count)
						final_ab = "B";
					else if (a_count == 0 && b_count == 0)
						final_ab = (Stop_left_right == 1) ? "A" : "B"; // 没有识别到，按输入值

					printf("最终AB识别为: %s\n", final_ab.c_str());
					// 根据识别结果设置停车方向
					if (final_ab == "A")
						Stop_left_right = 1;
					else if (final_ab == "B")
						Stop_left_right = 0;

					ab_results.clear();
					Zebra_Flag = 5;
					Stop_Flag = 1;
				}
			}
			*/

			if (Zebra_Flag == 4 && SlowSpeed_Flag == 2 && allow_stop_detect)
			{
				static std::deque<std::string> ab_results_blue; // 用于统计A/B
				static std::string final_ab = "Unknown";
				static bool locked_ab = false;
				Rect blue_flag_rect = Blue_Flag();

				// 实时识别并统计
				std::string ab = recognizeBlueAB_and_vote();
				printf("本帧蓝色AB识别为: %s\n", ab.c_str());

				// 只有还没锁定时才统计
				int a_count = std::count(ab_results_blue.begin(), ab_results_blue.end(), "A");
				int b_count = std::count(ab_results_blue.begin(), ab_results_blue.end(), "B");
				if (!locked_ab) {
					if (ab == "A" || ab == "B") ab_results_blue.push_back(ab);
					a_count = std::count(ab_results_blue.begin(), ab_results_blue.end(), "A");
					b_count = std::count(ab_results_blue.begin(), ab_results_blue.end(), "B");
					if (a_count >= 1) {
						final_ab = "A";
						locked_ab = true;
						printf("A识别到3次，锁定为A\n");
					} else if (b_count >= 1) {
						final_ab = "B";
						locked_ab = true;
						printf("B识别到3次，锁定为B\n");
					}
				}

				// 判断蓝色标志底部是否到达指定行数
				int bottom = blue_flag_rect.y + blue_flag_rect.height;
				printf("Blue Flag Bottom:%d\n", bottom);

				// 只有到达指定行数后才允许切换状态
				if (bottom > 200 && bottom <= 235)
				{
					printf("A_count:%d, B_count:%d\n", a_count, b_count);
					printf("最终蓝色AB识别为: %s\n", final_ab.c_str());
					// 根据识别结果设置停车方向
					if (final_ab == "A")
						Stop_left_right = 1;
					else if (final_ab == "B")
						Stop_left_right = 0;
					else if (final_ab == "Unknown")
						Stop_left_right = (Stop_left_right == 1) ? 1 : 0; // 没有识别到，按输入值

					ab_results_blue.clear();
					final_ab = "Unknown";
					locked_ab = false;
					Zebra_Flag = 5;
					Stop_Flag = 1;
				}
			}

			if(Zebra_Flag == 5)
			{
				//Stop_left_right = 0;//1左
				DetectParkingArea();
				printf("(Yellow_Parking.y + Yellow_Parking.height):%d\n",(Yellow_Parking.y + Yellow_Parking.height));
				if(Stop_Flag == 1 && ((Yellow_Parking.y + Yellow_Parking.height) >= 40))
				{
					Time_num++;
					int stop_threshold = (Stop_left_right == 1) ? 6 : 6; // 左停车阈值前，右停车阈值9
					if(Time_num > stop_threshold)
					{
						Stop_Flag = 2;
						Time_num =0;
						printf("Entering Parking Area!\n");
					}
				}
				if(Stop_Flag == 2)
				{
					
					Time_num++;
					int stop_threshold = (Stop_left_right == 1) ? 1 : 2; // 左停车阈值前，右停车阈值30
					if(Time_num > stop_threshold)
					{
						Stop_Flag = 3;
						Time_num = 0;
						printf("Strat Parking\n");
					}
					
					/*
					if(((Yellow_Parking.y + Yellow_Parking.height) >= 40)>=110)
					{
						Time_num++;
						if(Time_num > 10)
						{
							Stop_Flag = 3;
							Time_num = 0;
							printf("Start Parking\n");
						}
					}
					*/
				}
				if(Stop_Flag == 3)
				{
					Time_num++;
					int stop_threshold = (Stop_left_right == 1) ? 15 : 14;//3 4
					if(Time_num > stop_threshold)
					{
						Stop_Flag = 4;
					}
				}
				if(Stop_Flag == 4)
				{
					Time_num++;
					if(Time_num > 5)
					{
						Start = 4;
						printf("Parking Completed\n");
					}
				}
			}
			ShowBoundaryAndCenterLine(); //中线边线显示
			Center_Line();	//中心线修正
			CalcError();	//误差计算
			SteerControl();	//舵机调控
			//gpioPWM(12, Steer_Center);
			MotorControl();	//电机调控
			//printf("LeftBoundary_First:%d,RightBoundary_First:%d\n",ImageDeal[62].LeftBoundary_First,ImageDeal[62].RightBoundary_First);

			//SaveImageToVideo(Original_image);
			//SaveImageToVideo(LineDisplay_image);
			//ZoomImage(Original_image,1);
			//ZoomImage(Binary_image, 4); //放大指定图像
			//ZoomImage(Road_image, 1); //放大指定图像
			//ZoomImage(LineDisplay_image, 1);//放大指定图像
			//printf("OFFLineBoundary: %d\n", ImageStatus.OFFLineBoundary);
			//printf(", Det_True: %.2f", ImageStatus.Det_True);
			//ZoomImage(Cropped_image, 2);//放大指定图像
			//ZoomImage(Compress_image, 4);//放大指定图像
		}
		if (Start==4)
		{
			if (!trackbar_created)	
			{
				namedWindow("Original_image", WINDOW_NORMAL);
				createTrackbar("YunTai_X", "Original_image", NULL, 103, NULL); // 上限103
				createTrackbar("YunTai_Y", "Original_image", NULL, 103, NULL); // 上限103
				setTrackbarPos("YunTai_X", "Original_image", YunTai_X_val);
				setTrackbarPos("YunTai_Y", "Original_image", YunTai_Y_val);
				trackbar_created = true;
			}
			// 每帧获取滑动条值并控制云台
			YunTai_X_val = getTrackbarPos("YunTai_X", "Original_image");
			YunTai_Y_val = getTrackbarPos("YunTai_Y", "Original_image");
			// 限定下限为43
			if(YunTai_X_val < 43) YunTai_X_val = 43;
			if(YunTai_Y_val < 43) YunTai_Y_val = 43;
			gpioPWM(22, YunTai_X_val);
			gpioPWM(23, YunTai_Y_val);

			imshow("Original_image", Original_image);
		}
		auto end = std::chrono::high_resolution_clock::now();   // 记录结束时间

		double frame_time = std::chrono::duration<double, std::milli>(end - start).count();
		printf("Frame time: %.2f ms\n", frame_time);

		//每30秒保存一张图片
		static auto last_save_time = std::chrono::steady_clock::now();
		auto now = std::chrono::steady_clock::now();
		double seconds = std::chrono::duration<double>(now - last_save_time).count();
		if (seconds > 30.0) 
		{
			SaveSnapshotWithDate();
			last_save_time = now;
		}

		if (waitKey(1) == 'q') break; // 按q退出
	}
	ClearAllGPIO();
	gpioTerminate();
	cap.release();
	destroyAllWindows();
	return 0;
}