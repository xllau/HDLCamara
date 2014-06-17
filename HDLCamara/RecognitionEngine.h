#pragma once

#include "LaneMarkingScanner.h"
#include "BirdEyeView.h"
#include "HDLDisplay.h"
#include "CurbScanner.h"
#include "GLFunc.h"

#define TYPE_BIRDEYE 0
#define TYPE_PERSPECTIVE 1

#define IPM_ART 0
#define IPM_HOMO 1



class CRecognitionEngine
{
public:
	CRecognitionEngine();
	~CRecognitionEngine();

	void Initialize(int nType, int nIPM);
	bool OpenVideoFile(const char* filename);
	bool OpenHDLFile(const char* filename);
	bool OpenHomographyFile(const char* filename);
	bool OpenARTFile(const char* filename);
	void RunVideoHDL();
	void RunVideo();
	void RunHDL();

public:
	HDLDisplay *m_hdl;
	CurbScanner *m_curb;
	//BirdEyeView *m_bev;
	//LaneMarkingScanner *m_lms;
private:		
	int m_nType, m_nIPM;
	CvCapture *m_videoreader[NUM_VIDEO];
	CvVideoWriter *m_videowriter[NUM_VIDEO];
	char m_device_name[3][20];
	CvMat *m_homography[NUM_VIDEO], *m_homography_inv[NUM_VIDEO];
	double m_intrinsicA[NUM_VIDEO][9], m_laser64Rc[NUM_VIDEO][9], m_laser64Tc[NUM_VIDEO][3];
	CvSize m_size, m_birdeye_size;
	ifstream m_timestampfile_video, m_timestampfile_hdl;
	ofstream m_output;
	unsigned long m_timestamp_video, m_timestamp_video_next, m_timestamp_hdl;
	double m_vehicle_speed, m_preview_dist, m_eulr;
	double m_x_min, m_x_max, m_y_min, m_y_max;
	double m_x_scale, m_y_scale;
	
	//Output Parameters
	CvScalar m_lane_left, m_lane_right, m_lane_mid, m_output_param;
	CvScalar m_road_left, m_road_right, m_road_mid;
	int m_lane_left_type, m_lane_right_type;
	double m_stopline_dist, m_lane_width, m_road_width, m_ratio, m_curvature;
	bool m_pass_left, m_pass_right, m_use_lane;
	

	void DrawOpenGL();
	void SaveHDL();
	bool InitVideo();
	void PointConvert(CvPoint2D32f * point);
	void PointReverseConvert(CvPoint2D32f * point);
};
