#pragma once


#include "LineParamEstimator.h"


#define filter_length 129
#define MAX_POINTS 3000
#define CLOUD_NUM 256000
#define ANGLE_NUM 36000

#define HDL_LASER_NUMBER       64
#define HDL_MAX_POINT_LASER    4000
#define HDL_MAX_POINT_NUMBER HDL_MAX_POINT_LASER * HDL_LASER_NUMBER



typedef struct LPoint_t
{
	int x;
	int y;
	int z;
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
	unsigned char gray;
	int height;
	bool road;
};

typedef struct HPoint
{
	double x;
	double y;
	double z;
}HPoint_t;

typedef struct Point_laser
{
	int pt_count;
	LPoint_t pt[HDL_MAX_POINT_LASER];
	int rec[HDL_MAX_POINT_LASER];
	int is_road[HDL_MAX_POINT_LASER];
}Point_laser_t;

class CurbScanner
{
public:
	CurbScanner();
	~CurbScanner();
	void Initialize();
	void ProcessFrame(IplImage * imgOrigin);
	void detection_road();
	void Construction_3D(Mat img_original);

public:
	LPoint_t *m_cloud;
	Point_laser_t *m_point;
	int m_cloud_count;
	
	IplImage * img;
	IplImage * gray;
	CvSize m_birdeye_size ;
	int m_x_min ;		//meter
	int m_x_max ;		//meter
	int m_y_min ;		//meter
	int m_y_max ;		//meter
	double m_x_scale ;
	double m_y_scale ;
	double k ,y0,z0; //road surface

	double m_homography[9];
	double m_homography_inv[9];
private:
	int m_laser_index[HDL_LASER_NUMBER];
	void pointconvert(CvPoint2D32f *point);
	void pointreconvert(CvPoint2D32f *point);
	bool GetBirdEyeView(IplImage * imgOrigin);
	void ransac_compute(vector<Point2D>,double &k,double &y0,double &z0);
	void detect_obstacle(void);
};

