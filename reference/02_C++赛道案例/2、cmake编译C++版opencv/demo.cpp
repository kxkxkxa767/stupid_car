#include <iostream>
#include<stdlib.h>
#include<opencv4/opencv2/core/core.hpp>
#include<opencv4/opencv2/highgui.hpp>
#include<opencv4/opencv2/opencv.hpp>
#include<opencv4/opencv2/imgproc/imgproc_c.h>
#define Usage()\
{std::cerr<<"usage: ./showpic FILE"<<std::endl;}
using namespace std;
using namespace cv;
Mat frame, image;

int main()
{

	VideoCapture capture;
  capture.open(0);
	if (!capture.isOpened())
	{
		cout << "Can not open video file!" << endl;
		system("pause");
		return -1;
	}

	while (capture.read(frame))
	{
		imshow("1",frame);
  if (waitKey(1) == 27)
   break;
	}
 
  capture.release();
	destroyAllWindows();
	system("pause");
	return 0;
}