#include<iostream>
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/imgproc/types_c.h>
#include<stdlib.h>
#include <string>
using namespace std;
using namespace cv;
#define PI 3.1415926
#define Usage()\
{std::cerr<<"usage: ./showpic FILE"<<std::endl;}

Rect Obstacles(Mat img);
Rect blue(Mat img);

Rect blue(Mat img)//识别到蓝色 
{
	Mat kernel_3 = Mat::ones(Size(3, 3), CV_8U);
	Mat HSV, roi;
	GetROI(img, roi);
	cvtColor(roi, HSV, COLOR_BGR2HSV);
	Scalar Lower(78, 43, 46);
	Scalar Upper(110, 255, 255);
	Mat mask;
	inRange(HSV, Lower, Upper, mask);
	Mat erosion;
	erode(mask, erosion, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
	erode(mask, erosion, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
	Mat dilation;
	dilate(erosion, dilation, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
	dilate(erosion, dilation, kernel_3, Point(-1, -1), 1, BORDER_CONSTANT, morphologyDefaultBorderValue());
	Mat target;
	bitwise_and(roi, roi, target, dilation);
	Mat binary;
	threshold(dilation, binary, 127, 255, THRESH_BINARY);
	vector<vector<Point>> contours;
	findContours(binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
	if (contours.size() == 0) {
		return Rect();
	}
	else {
		return boundingRect(contours[0]);
	}
}


Rect Obstacles(Mat img)//识别到锥桶
{
	Rect obstacle_object = blue(img);//识别到的蓝色物体
	if (obstacle_object.height > 5 && obstacle_object.height < 100)//对识别到的蓝色物体限幅,
	{
		rectangle(img, Point(obstacle_object.x, obstacle_object.y + frame.rows / 2), Point(obstacle_object.x + obstacle_object.width, obstacle_object.y + obstacle_object.height + frame.rows / 2), Scalar(0, 255, 0), 3);
		return obstacle_object;
	}
	return Rect();
}


int main()
{
    Rect list;
    list = Obstacles(frame);
	int angle_bi;
    if (!list.empty())//遇到障碍物舵机打角
    {
        len = list.x + list.width - frame.rows / 2;
		angle_bi= PID(len);
        Set_duo(angle_bi);
        continue;
    }
    else//否则正常循迹
    {
        picture();
    }
}


