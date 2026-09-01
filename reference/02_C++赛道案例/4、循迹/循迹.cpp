void picture()
{
	//得到赛道图像
	Mat roi, hui, gao, ca;
	cvtColor(frame, hui, COLOR_BGR2GRAY);
	GaussianBlur(hui, gao, Size(5, 5), 0.5, 0.5);
	Canny(gao, ca, MIN_YU, MAX_YU, 3);
	imshow("canny", ca);
	
	int co = ca.cols, ro = ca.rows;
	std::vector<int> left(co, -1);  // 左边线数组
	std::vector<int> right(co, -1);  // 右边线数组
	std::vector<int> mid(co, -1);  // 中线数组
	int mid_sum = 0;  // 中点坐标的总和
	int l_lost = 0;  // 左边线丢失标志位
	int r_lost = 0;  // 右边线丢失标志位
	int t = 0;  // 计数器

    
//扫线
	for (int i = 350; i >= 250 ; i--)
   {
		l_lost = 0;
		r_lost = 0;
		for (int j = co/2; j > 10; j--) //寻找左边界
        {
			if (ca.at<uchar>(i, j) == 255) {
				left[t] = j;
				l_lost = 1;
				break;
			}
		}

		if (l_lost == 0) //左边线丢失
  		{
			left[t] = 0;
		}

		for (int j = co/2; j <= co ; j++)//寻找右边线
        {
			if (ca.at<uchar>(i, j) == 255 ) 
            {
				right[t] = j;
				r_lost = 1;
				break;
			}
		}

		if (r_lost == 0) //右边线丢失
        {
			right[t] = ro - 1;
		}
   
		//描点画线
    	Point pa(right[t],  i);
		Point pb(left[t],  i );
   
		circle(frame, pa, 2, Scalar(255, 25, 0));

		circle(frame, pb, 2, Scalar(255, 25, 0));

		Point p((left[t] + right[t]) / 2, i + co / 2);
		circle(frame, p, 2, Scalar(255, 25, 0));
   
		mid[t] = (left[t] + right[t]) / 2;
		mid_sum += mid[t];
		t++;
	}

	imshow("closemat", frame);
	mid_final = static_cast<double>(mid_sum) / 100;//中线求平均
	cout << mid_final << endl;
	double error = mid_final - co/2; 
	//偏差控制
	PID(error);
}
