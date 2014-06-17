#pragma once

#include "DoubleSidesModelFitting.h"

#define LMS_WIDTH_MIN_THRESHOLD 0.9
#define LMS_WIDTH_MAX_THRESHOLD 1.1

#define LMS_TRACKER_LINE_PARAM_A 0.1
#define LMS_TRACKER_LINE_PARAM_B 40
#define LMS_TRACKER_TIMES_TO_CONFIRM1 20
#define LMS_TRACKER_TIMES_TO_CONFIRM2 5

#define LMS_TYPE_SOLID 0
#define LMS_TYPE_DASHED 1
#define NUM_VIDEO 1

class LaneMarkingScanner
{
public:
	IplImage *m_img[NUM_VIDEO];
	IplImage *m_imgGray[NUM_VIDEO], *m_imgEdge[NUM_VIDEO], *m_imgEdgeColor[NUM_VIDEO];
	IplImage *m_imgIntensity[NUM_VIDEO], *m_imgIntensityColor[NUM_VIDEO], *m_imgResult[NUM_VIDEO];
	CvSize m_size;
	double m_homography[NUM_VIDEO][9], m_homography_inv[NUM_VIDEO][9];
	CvScalar m_vehicle_left, m_vehicle_right, m_vehicle_mid;
	double m_lanewidth_vehicle, m_stopline_dist;
	int m_left_type, m_right_type;

public:
	LaneMarkingScanner();
	~LaneMarkingScanner();

	void Initialize();
	void ProcessFrame();
	
private:
	CvScalar m_curve_left, m_curve_right, m_curve_mid;
	CvScalar m_line_left[NUM_VIDEO], m_line_right[NUM_VIDEO];
	vector<CvScalar> m_vec_line_left[NUM_VIDEO], m_vec_line_right[NUM_VIDEO];
	vector<CvPoint> m_vec_stopline;
	double m_a_max[3], m_a_min[3]; 
	int m_horizon[NUM_VIDEO], m_lane_width[NUM_VIDEO], m_mid_pos[NUM_VIDEO], m_height_skip;
	double m_dir[NUM_VIDEO];
	CvRect m_rect_intensity[3], m_rect_line[3];
	vector<CvPoint> m_list_left, m_list_right, m_list_mid;
	CvScalar m_stopline;
	CvFont m_font;
	DoubleSidesModelFitting m_model_single, m_model_double;
	
private:
	void DoubleThresh(IplImage* src_gray, IplImage* src_edge, IplImage* dst, double thresh);
	void GetEdgeMap(int num);
	void GetIntensityMap(int num);
	void ProcessEdge(int num);
	void ProcessIntensity(int num);
	void GroupLineLeft(int num);
	void GroupLineRight(int num);
	void TrackLineLeft(int num);
	void TrackLineRight(int num);
	void DetectStopLine();
	void EstimateLines(int num);
	void EstimateCurves();
	void MarkImage();
	CvScalar GetLaneMarkVehicleCoord(const CvScalar& curve);
	void GetCurveVehicle();
	double GetStopLineDist();
};

