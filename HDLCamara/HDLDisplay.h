#pragma once

#include "GLFunc.h"

#define ANGLE_NUM 36000

typedef struct PointSave
{
	unsigned short dist;
	unsigned short rot;
	unsigned char i;
	unsigned char c;
}PointSave_t;

class HDLDisplay
{
public:
	HDLDisplay(void);
	~HDLDisplay(void);
	bool Initialize(const char* hdlfilepath);
	bool Initialize(const char* hdlfilepath, const char* timestamp);
	bool ProcessFrame();

private:
	bool load_laser_color(const char* colorpath);
	bool load_laser_info(const char* data_path);

public:
	MyColor_t m_lasercolor[HDL_LASER_NUMBER];
	LPoint_t *m_cloud;
	int m_save_cloud_count; //number of points of a scan

private:
	//fixed system info for the specific lidar, read from xml
	float m_rot[HDL_LASER_NUMBER];  //for each laser: rot angle
	float m_vert[HDL_LASER_NUMBER];  //vertical angle correction
	float m_dist[HDL_LASER_NUMBER];  //distance system error
	float m_z_off[HDL_LASER_NUMBER]; //vertical offset
	float m_x_off[HDL_LASER_NUMBER]; //horizantal offset

	//S2 new features:
	float m_min_i[HDL_LASER_NUMBER]; //minIntensity
	float m_max_i[HDL_LASER_NUMBER]; //maxIntensity
	float m_distX[HDL_LASER_NUMBER]; //distCorrectionX
	float m_distY[HDL_LASER_NUMBER]; //distCorrectionY
	float m_f_d[HDL_LASER_NUMBER];  //focalDistance
	float m_f_s[HDL_LASER_NUMBER]; //focalSlope

	float m_cos_rot[HDL_LASER_NUMBER];
	float m_sin_rot[HDL_LASER_NUMBER];
	float m_cos_vert[HDL_LASER_NUMBER];
	float m_sin_vert[HDL_LASER_NUMBER];
	float *m_cos_raw;
	float *m_sin_raw;

	PointSave_t *m_save_cloud;
	FILE* m_hdldata;
};

