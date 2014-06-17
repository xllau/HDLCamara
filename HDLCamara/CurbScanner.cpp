
#include "stdafx.h"
#include "CurbScanner.h"

ofstream roadpoint_f("roadpoint.txt");

double A_m[3][3]={
1034.78651912241,	-0.699546663476867,	385.382553640537,
0,					1033.92695954424,	278.481492936457,
0,					0,					1};
double Rc_m[3][3]={
0.999112142868413,	-0.0374103001563469,	-0.0193751236151105,
-0.0260333433804964,	-0.186639528040299,	-0.982083474866229,
0.0331238736460871,	0.981715924295367,	-0.187447733994230};

double Tc_m[3][1]={
20.3714570235835,
-7.88997327582634,
18.9850937351037};

double R_m[4][4] = {
0.999112142868413,		-0.0374103001563469,	-0.0193751236151105,	20.3714570235835,
-0.0260333433804964,	-0.186639528040299,		-0.982083474866229,		-7.88997327582634,
0.0331238736460871,		0.981715924295367,		-0.187447733994230,		18.9850937351037,
0, 0, 0, 1};

void tentacle(IplImage * img_gray,IplImage * img);

CurbScanner::CurbScanner()
{
	m_cloud = NULL;
	m_point = NULL;
}

CurbScanner::~CurbScanner()
{
	if (m_cloud)
	{
		delete[] m_cloud;
		m_cloud = NULL;
	}
	if (m_point)
	{
		delete[] m_point;
		m_point = NULL;
	}
}

void CurbScanner::Initialize()
{
	m_cloud = new LPoint_t[HDL_MAX_POINT_NUMBER];
	m_cloud_count = 0;
	m_point = new Point_laser_t[HDL_LASER_NUMBER];
	
	int temp[] = {39, 40, 43, 44, 33, 34, 37, 38, 41, 42,
		47, 48, 51, 52, 55, 56, 45, 46, 49, 50,
		53, 54, 59, 60, 63, 64, 35.0, 36, 57, 58,
		61, 62, 7, 8, 11, 12, 1, 2, 5, 6,
		9, 10, 15, 16, 19, 20, 23, 24, 13, 14,
		17, 18, 21, 22, 27, 28, 31, 32, 3, 4,
		25, 26, 29, 30
	};

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_laser_index[i] = temp[i];
	}

	
	srand((unsigned)time(NULL));

	m_birdeye_size = cvSize(400, 600);
	m_x_min = -10;		//meter
	m_x_max = 10;		//meter
	m_y_min = 5;		//meter
	m_y_max = 35.0;		//meter
	m_x_scale = m_birdeye_size.width / (m_x_max - m_x_min);
	m_y_scale = m_birdeye_size.height / (m_y_max - m_y_min);
//cvInitFont(&font,CV_FONT_HERSHEY_DUPLEX ,1.0f,1.0f,0,1,CV_AA);
	img = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 3);
	gray = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 1);
}

void CurbScanner::ProcessFrame(IplImage * imgOrigin)
{
	int c, mcount[HDL_LASER_NUMBER]={0};
	//int nWindowHeight[100];
	//Point_laser *m_point=new Point_laser[HDL_LASER_NUMBER];
	
	for(int i = 0;i < HDL_LASER_NUMBER;i++)
	{
		for(int j = 0;j < HDL_MAX_POINT_LASER;j++){
			m_point[i].pt[j].rot = 0;
			m_point[i].pt[j].x = 0;
			m_point[i].pt[j].y = 0;
			m_point[i].pt[j].z = 0;
			m_point[i].pt[j].gray = 0;
			m_point[i].pt[j].height = 0;
			m_point[i].pt[j].road = false;
		}
		m_point[i].pt_count = HDL_MAX_POINT_LASER;
	}
	for (int i = 0; i < m_cloud_count; i++)
	{
		//if(abs(pts[i].x) > 20000 || pts[i].y < 4000
		//	|| (pts[i].rot < 30000 && pts[i].rot > 6000))
			//	continue;

		c = m_cloud[i].c;
		//pts_road[i]=false;
		int index = m_cloud[i].rot/9;
		m_point[c].pt[index].x = m_cloud[i].x;
		m_point[c].pt[index].y = m_cloud[i].y;
		m_point[c].pt[index].z = m_cloud[i].z;
		//int temp = 0.1 * (pts[i].z + 3560);
		int temp = 0.2 * (m_cloud[i].z + 2780);
		temp = min(temp,255);
		temp = max(temp,0);
		m_point[c].pt[index].height = temp;
		m_point[c].pt[index].gray = 0;
		m_point[c].pt[index].i = m_cloud[i].i;
		m_point[c].pt[index].rot = m_cloud[i].rot;
		m_point[c].rec[index] = i;
		m_point[c].is_road[index] = false;
		m_point[c].pt_count = mcount[c];
		m_point[c].pt[index].c = c;
		mcount[c]++;
	}


	//Sort the 64 lasers from near to far
	Point_laser* pl = new Point_laser[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		pl[i] = m_point[m_laser_index[i] - 1];
	}
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point[i] = pl[i];
	}
	delete[] pl;

	Mat img(imgOrigin);

	detection_road();
	//Construction_3D(img);
}

void CurbScanner::detection_road()
{
	Mat l_map(64,4000,CV_8UC1);

	//store the data to the map
	uchar* p;
	int value_z = 0;
	for(int i = 0;i < l_map.rows;i++){
		p = l_map.ptr<uchar>(i);
		for(int j = 0;j < l_map.cols;j++)
			p[j] = m_point[HDL_LASER_NUMBER -1 - i].pt[j].height;
	}
	//rearrange the data
	Mat sub_map2 = l_map(Range::all(),Range(0,500));
	Mat sub_map1 = l_map(Range::all(),Range(l_map.cols - 500,l_map.cols));
	//Mat sub_map1 = l_map(Range::all(),Range(1000,1500));

	//imshow("sub_map1",sub_map1);
	//imshow("sub_map2",sub_map2);

	//sub_map = sub_map.t();
	Mat sub_map(sub_map1.rows , sub_map1.cols + sub_map2.cols , l_map.type());
	sub_map1.copyTo(sub_map(Range::all(),Range(0,500)));
	sub_map2.copyTo(sub_map(Range::all(),Range(500,500+sub_map2.cols)));
	//imshow("original",l_map);
	//imwrite("original.jpg",l_map);
	//waitKey(0);
	//imshow("sub_map",sub_map);//imwrite("2.jpg",sub_map);
	Mat element = getStructuringElement(0,Size(3,3),Point(0,0));
	//morphologyEx(sub_map,sub_map,MORPH_DILATE,element);
	morphologyEx(sub_map,sub_map,MORPH_CLOSE,element);
//imshow("temp",sub_map);//imwrite("3.jpg",sub_map);

	int kernel_size = 3;
	Mat Sx;
	Sobel(sub_map, Sx, CV_32F, 1, 0, kernel_size);
	Mat Sy;
	Sobel(sub_map, Sy, CV_32F, 0, 1, kernel_size);
	Mat mag, ori;
	magnitude(Sx, Sy, mag);
	Mat abs_mag = abs(mag);
	Mat binary(64,1000,CV_8UC1);
	//phase(Sx, Sy, ori, true);
	threshold(abs_mag,binary,80,255,CV_THRESH_BINARY);
	//imshow("magnitude",mag);
	//imshow("binary",binary);
	Mat close(binary);

	element = getStructuringElement(0,Size(5,5),Point(0,0));
	morphologyEx(binary,close,MORPH_CLOSE,element);
	imshow("binary_close",close);
	//Mat_<UINT8>markers(64,1000);
	//markers(Range::all(),Range::all()) = 0;
	//markers(63,500) = 255;
	Mat mfloodfill(close.size(),close.type());
	cvtColor(close,mfloodfill,CV_GRAY2BGR);
	//mask.create(close.rows + 2, close.cols + 2, CV_8UC1);
	Mat mask = Mat::zeros(mfloodfill.rows + 2, mfloodfill.cols + 2, CV_8U);
	floodFill(mfloodfill, mask, Point(500,63), 255, 0, cv::Scalar(), cv::Scalar(), 4 + (255 << 8) +FLOODFILL_MASK_ONLY);
	imshow("mask",mask);
	//morphologyEx(binary,close,MORPH_ERODE,element);


	//draw the points
	for(int i = 0;i< mfloodfill.rows;i++){
		const uchar * ptr = mask.ptr<uchar>(i+1);
		for(int j = 0; j < mfloodfill.cols;j++){
			if(ptr[j+1] > 128)
			{
				if(j < 500)
					m_point[63 - i].pt[3499 + j + 1].road = true;
				if(j >= 500)
					m_point[63 - i].pt[j - 500 + 1].road = true;
			}
		}
	}


	//ofstream data("map.txt");
	//
	//int d = 0;
	////data<<d<<endl;
	//for(int i = 0;i < sub_map.rows;i++){
	//	p = sub_map.ptr<uchar>(i);
	//	for(int j = 0;j < sub_map.cols;j++)
	//		{//cout<<(int)p[j]<<endl;
	//		d = (int)p[j];
	//			data<<d<<"  ";
	//	}
	//		data<<endl;
	//}
	//data.close();
	imshow("l_map",sub_map);
	cvWaitKey(10);
}

void CurbScanner::Construction_3D(Mat img_original)
{
	Mat A = Mat(3,3,CV_64F,A_m);
	Mat Rc = Mat(3,3,CV_64F,Rc_m);
	Mat Tc = Mat(3,1,CV_64F,Tc_m);
	Mat R = Mat(4,4,CV_64F,R_m);

	//cout<<A<<endl;
	//for(int i = 0;i < HDL_LASER_NUMBER;i++)
	//{
	//	for(int j = 0;j < HDL_MAX_POINT_LASER;j++){
	//	Mat P = (Mat_<double>(4,1)<<m_point[i].pt[j].x,	m_point[i].pt[j].y,	m_point[i].pt[j].z,1);
	//	Mat P_camara = R*P;
	//	//cout<<P_camara<<endl;
	//	Mat P_camara2 = P_camara(Range(0,3),Range::all());
	//	//cout<<P_camara2<<endl;
	//	Mat uvz = A* P_camara2;
	//	//cout<<uvz<<endl;
	//	int u = uvz.at<double>(0,0)/uvz.at<double>(2,0);
	//	int v = uvz.at<double>(1,0)/uvz.at<double>(2,0);
	//	if(u >= 0 && u < 780 && v>= 0 && v < 582)
	//		img.at<double>(v,u) = P.at<double>(2,0);
	//	}
	//}
	//Mat P(1,4,CV_64FC1,Scalar::all(1));//HDL_LASER_NUMBER
	Mat P,index;
	for(int i = 0;i < m_cloud_count;i++)
	{
		m_cloud[i].gray = 0;
		if(//abs(m_cloud[i].x) < 2000  &&  
			m_cloud[i].y > 0  && 
			m_cloud[i].z > -3000){
			Mat a =(Mat_<double>(1,4)<< m_cloud[i].x, m_cloud[i].y,	m_cloud[i].z, 1);
			P.push_back(a);
			Mat b = (Mat_<int>(1,1)<<i);
			index.push_back(b);
			//cout<<index<<endl;
			//cout<<P<<endl;
		}
	}
	
	Mat P_camara = R*P.t();
	Mat P_camara2 = P_camara(Range(0,3),Range::all());//cout<<P_camara2<<endl;
	Mat uvz = A* P_camara2;//cout<<uvz<<endl;//cout<<uvz<<endl;
	Mat_<double>u = uvz.row(0)/uvz.row(2);//cout<<u<<endl;
	Mat_<double>v = uvz.row(1)/uvz.row(2);//cout<<v<<endl;
	
	//Mat img(582,780,CV_16SC1,Scalar::all(0));
	int x,y;
	for(int i = 0; i < u.cols;i++){
		x = u(0,i);y = v(0,i);
		if(x >= 0 && x < 780 && y >= 0 && y < 582)
			m_cloud[index.at<int>(i,0)].gray = img_original.at<uchar>((int)y,(int)x);
			//img.at<ushort>((int)y,(int)x) = P.at<double>(i,2) + 3000;//P.at<float>(2,i);
	}

	//Store all the points according to their c

	int i;
	int c, count[HDL_LASER_NUMBER];
	
	for (i = 0; i < HDL_LASER_NUMBER; i++)
		count[i] = 0;

	for (i = 0; i < m_cloud_count; i++)
	{
		c = m_cloud[i].c;
		m_point[c].pt[count[c]].x = m_cloud[i].x;
		m_point[c].pt[count[c]].y = m_cloud[i].y;
		m_point[c].pt[count[c]].z = m_cloud[i].z;
		m_point[c].pt[count[c]].gray = m_cloud[i].gray;
		m_point[c].pt[count[c]].c = c;
		count[c]++;
	}
	for (i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point[i].pt_count = count[i];
	}
		
	//Sort the 64 lasers from near to far
	Point_laser* pl = new Point_laser[HDL_LASER_NUMBER];
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		pl[i] = m_point[m_laser_index[i] - 1];
	}
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_point[i] = pl[i];
	}
	delete[] pl;
	//Mat close(img.size(),img.type());
	//Mat element = getStructuringElement(0,Size(15,15),Point(0,0));
	//morphologyEx(img,close,MORPH_CLOSE,element);

	/*Mat img_show;
	img.convertTo(img_show,CV_8UC1);
	imshow("height",img_show);*/
	//waitKey(0);



}

void CurbScanner::pointconvert(CvPoint2D32f *point)
{//convert the real point into the image
	CvPoint2D32f temp;
	temp = * point;
	point->x = (temp.x - m_x_min) * m_x_scale;
	point->y = m_birdeye_size.height - (temp.y - m_y_min) * m_y_scale;
}

void CurbScanner::pointreconvert(CvPoint2D32f *point)
{//convert the image point to real coordinate
	CvPoint2D32f temp;
	temp = * point;
	point->x = temp.x / m_x_scale + m_x_min;
	point->y = (m_birdeye_size.height - temp.y) / m_y_scale + m_y_min;
}

bool CurbScanner::GetBirdEyeView(IplImage * imgOrigin)//output the m_imgGray and the m_imgResult;
{
	double x, y, u, v;
	int ix, iy;
	CvPoint pos[4];
	uchar pixel[4];
	double p[3], Pl[3];
	uchar *imagebase;
	IplImage * temp = cvCreateImage(cvSize(imgOrigin->width,imgOrigin->height) ,8,1);
	if(imgOrigin->nChannels == 3)
		cvCvtColor(imgOrigin,temp , CV_BGR2GRAY);

	if (fabs(m_homography[8]) < 1e-6)
		return false;

	cvZero(gray);

	imagebase = (uchar*)gray->imageData;

	for (int i = 0; i < m_birdeye_size.height; i++)
	{
		for (int j = 0; j < m_birdeye_size.width; j++)
		{
			x = j / m_x_scale + m_x_min;
			y = (m_birdeye_size.height - i) / m_y_scale + m_y_min;
			

			Pl[0] = x;
			Pl[1] = y;
			Pl[2] = 1;
			for (int k = 0; k < 3; k++)
			{
				p[k] = 0;
				for (int l = 0; l < 3; l++)
				{
					p[k] += m_homography[k*3+l] * Pl[l];
				}
			}
			p[0] /= p[2];
			p[1] /= p[2];

			ix = (int)p[0];
			iy = (int)p[1];
			u = p[0] - ix;
			v = p[1] - iy;
			pos[0] = cvPoint(ix, iy);
			pos[1] = cvPoint(ix + 1, iy);
			pos[2] = cvPoint(ix, iy + 1);
			pos[3] = cvPoint(ix + 1, iy + 1);

			for (int k = 0; k < 4; k++)
			{
				if (pos[k].x < 0 || pos[k].x >= temp->width || pos[k].y < 0 || pos[k].y >= temp->height)
					pixel[k] = 0;
				else
					pixel[k] = ((uchar*)(temp->imageData + temp->widthStep * pos[k].y))[pos[k].x];
			}
			imagebase[j] = (uchar)((1-u)*(1-v)*pixel[0] + u*(1-v)*pixel[1] + (1-u)*v*pixel[2] + u*v*pixel[3]);
		}
		imagebase += gray->widthStep;
	}
	cvCvtColor(gray, img, CV_GRAY2BGR);
	cvZero(gray);
	return true;
}

void CurbScanner::detect_obstacle(void)
{
	
	for (int i = 5; i < 64; i++)
	{
		//Detect right road surface
		for (int j = 0; j < m_point[i].pt_count - 1; j++)
		{
			m_point[i].is_road[j] = false;
		}
	}

	for (int i = 5; i < 64; i++)
	{
		//Detect right road surface
		for (int j = 0; j < m_point[i].pt_count - 1; j++)
		{
			//cout<<(m_point[i].pt[j].y - (m_point[i].pt[j].z- z0) * k + y0)<<endl;
			if(abs(m_point[i].pt[j].z - ((m_point[i].pt[j].y- y0) * k + z0)) < 200 )
			{	
				//cout<<abs(m_point[i].pt[j].z - (m_point[i].pt[j].y- y0) * k + z0)<<endl;
				m_point[i].is_road[j] = true;
			}
		}
	}

}






bool flag_tentacle_center,flag_tentacle_left[30],flag_tentacle_right[30];
double width_safe = 3.25/2.0; //half of the lane width
int num_tcl = 20;
double dist_det = 30.0;

void tentacle(IplImage * img_gray,IplImage * img)
{
	//21 lines
	//center one
	//cvShowImage("test",img_gray);
//cvWaitKey(5);
//	cout<<"come on"<<endl;
//	unsigned char * data = (unsigned char *)img_gray->imageData;
//for(int i = 0;i< 400;i++)
//	cout<<(int)data[i]<<endl;
	double x = 0,x_center = 0;
	double y0=0;
	flag_tentacle_center = true;
	double pi = 3.1415926;
	bool left_succeed = true,right_succeed = true;
	for(double y = 10;y < dist_det;y += 0.05)
	{
		x_center = 0;
		x = x_center + width_safe; //right side
		x = (x + 10)* 20;
		y0 =(35.0 - y)* 20;
		//cvCircle(img,cvPoint(x,y0),1,CV_RGB(0,0,255),1);

		if(cvGet2D(img_gray,y0,x).val[0] == 0)
		{
			right_succeed = false;
			flag_tentacle_center = false;
			break;
		}
		x = x_center - width_safe;
		x = (x + 10)* 20;
		y0 =(35.0 - y)* 20;
		//cvCircle(img,cvPoint(x,y0),1,CV_RGB(0,0,255),1);
		if(cvGet2D(img_gray,y0,x).val[0] == 0)
		{
			left_succeed = false;
			flag_tentacle_center = false;
			break;
		}
	}

	//else if(right_succeed == false)//scan left
	{
		for(int i = 1; i < num_tcl;i++)
		{
			double theta = i*2/180.0 * pi; //theta
			double r = 35.0/theta; //radius
			flag_tentacle_left[i] = true;
			double x_t=0;
			for(double y = 10;y < dist_det; y += 0.05)
			{
				x = sqrt(r*r - y*y) - r;
				
				x_t = x + width_safe;
				x_t = (x_t + 10)* 20;
				y0 = (35.0 - y)* 20;
				if(cvGet2D(img_gray,y0,x_t).val[0] == 0)
				{
					flag_tentacle_left[i] = false;
					break;
				}
				x_t = x - width_safe;
				x_t = (int)(x_t + 10)* 20;
				y0 = (int)(35.0 - y)* 20;
				if(cvGet2D(img_gray,y0,x_t).val[0] == 0)
				{
					flag_tentacle_left[i] = false;
					break;
				}
			}
		}
	}
	//else if(left_succeed == false)//scan right
	{
		for(int i = 1; i < num_tcl;i++)
		{
			double theta = i*2/180.0 * pi; //theta
			double r = 35.0/theta; //radius
			flag_tentacle_right[i] = true;
			double x_t=0;
			for(double y = 10;y < dist_det;y += 0.05)
			{
				x = r - sqrt(r*r - y*y);
				
				x_t = x + width_safe;
				x_t = (int)(x_t + 10)* 20;
				y0 = (int)(35.0 - y)* 20;
				if(cvGet2D(img_gray,y0,x_t).val[0] == 0)
				{
					flag_tentacle_right[i] = false;
					break;
				}
				x_t = x - width_safe;
				x_t = (int)(x_t + 10)* 20;
				y0 = (int)(35.0 - y)* 20;
				if(cvGet2D(img_gray,y0,x_t).val[0] == 0)
				{
					flag_tentacle_right[i] = false;
					break;
				}
			}
		}
	}
	//draw line
	for(double y = 10;y < 30.0;y += 0.05)
	{
		x_center = 0;
		x = x_center ; //right side
		x = (x + 10)* 20;
		y0 = (35.0 - y)* 20;
		if(flag_tentacle_center)cvCircle(img,cvPoint(x,y0),0,CV_RGB(0,255,0),0);
		else					cvCircle(img,cvPoint(x,y0),0,CV_RGB(255,0,0),0);
	}
	//left
	for(int i = 1; i < num_tcl;i++)
	{
		double theta = i*2/180.0 * pi; //theta
		double r = 35.0/theta; //radius
		for(double y = 10;y < 27.0;y += 0.05)
		{
			x = sqrt(r*r - y*y) - r;
			x = (x + 10)* 20;
			y0 = (35.0 - y)* 20;

			if(flag_tentacle_left[i])cvCircle(img,cvPoint(x,y0),0,CV_RGB(0,255,0),0);
			else					cvCircle(img,cvPoint(x,y0),0,CV_RGB(255,0,0),0);
		}
	}
	//right
	for(int i = 1; i < num_tcl;i++)
	{
		double theta = i*2/180.0 * pi; //theta
		double r = 35.0/theta; //radius
		for(double y = 10;y < 27.0;y += 0.05)
		{
			x = r - sqrt(r*r - y*y) ;
			x = (x + 10)* 20;
			y0 = (35.0 - y)* 20;

			if(flag_tentacle_right[i])cvCircle(img,cvPoint(x,y0),0,CV_RGB(0,255,0),0);
			else					cvCircle(img,cvPoint(x,y0),0,CV_RGB(255,0,0),0);
		}
	}
	{
		//give the preview
		double y = 20;// m_vehicle_dist;
		double x = 0,eulr = 0;
		if(flag_tentacle_center)
		{
			x = 0;eulr = 0;y = 20;
			x = (x + 10)* 20;
			y = (35.0 - y)* 20;
			cvCircle(img,cvPoint(x,y),3,CV_RGB(0,0,255),2);
		}
		else
		{
			for(int i =1 ; i < num_tcl;i++)
				if(flag_tentacle_right[i])
				{
					double theta = i*2/180.0 * pi; //theta
					double r = 35.0/theta; //radius
					y = 20;
					x = r - sqrt(r*r - y*y) ;
					eulr = theta;
					x = (x + 10)* 20;
					y = (35.0 - y)* 20;
					cvCircle(img,cvPoint(x,y),3,CV_RGB(0,0,255),2);
					break;
				}
			for(int i = 1 ; i < num_tcl;i++)
				if(flag_tentacle_left[i])
				{
					double theta = i*2/180.0 * pi; //theta
					double r = 35.0/theta; //radius
					y = 20;
					x = sqrt(r*r - y*y) - r;
					eulr = theta;
					x = (x + 10)* 20;
					y = (35.0 - y)* 20;
					cvCircle(img,cvPoint(x,y),3,CV_RGB(0,0,255),2);
					break;
				}
		}
	}

}

