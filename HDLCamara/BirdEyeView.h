#pragma once

#define BEV_WIDTH_MIN_THRESHOLD 0.8
#define BEV_WIDTH_MAX_THRESHOLD 1.2

#define BEV_TRACKER_LINE_PARAM_A 0.05
#define BEV_TRACKER_LINE_PARAM_B 10
#define BEV_TRACKER_TIMES_TO_CONFIRM1 20
#define BEV_TRACKER_TIMES_TO_CONFIRM2 5

#define BEV_TYPE_SOLID 0
#define BEV_TYPE_DASHED 1

class BirdEyeView
{
public:
	IplImage *m_img[NUM_VIDEO];
	IplImage *m_imgGray[NUM_VIDEO], *m_imgEdge[NUM_VIDEO], *m_imgEdgeColor[NUM_VIDEO];
	IplImage *m_imgIntensity[NUM_VIDEO], *m_imgIntensityColor[NUM_VIDEO];
	IplImage *m_imgResult[NUM_VIDEO], *m_imgOriginResult[NUM_VIDEO], *m_imgOriginGray[NUM_VIDEO];
	CvSize m_size, m_birdeye_size;
	double m_x_min, m_x_max, m_y_min, m_y_max;
	double m_x_scale, m_y_scale;
	double m_homography[NUM_VIDEO][9], m_homography_inv[NUM_VIDEO][9];
	double m_lanewidth_vehicle, m_stopline_dist;
	CvScalar m_vehicle_left, m_vehicle_right, m_vehicle_mid;
	int m_left_type, m_right_type;

public:
	BirdEyeView();
	~BirdEyeView(void);
	
	void Initialize();
	void ProcessFrame();
	
private:
	CvRect m_rect_line[3], m_rect_intensity[3];
	double m_limit_a;
	vector<CvScalar> m_vec_line_left[NUM_VIDEO], m_vec_line_right[NUM_VIDEO];
	CvScalar m_line_left[NUM_VIDEO], m_line_right[NUM_VIDEO];
	CvScalar m_curve_left[NUM_VIDEO], m_curve_right[NUM_VIDEO], m_curve_mid[NUM_VIDEO];
	vector<CvPoint2D64f> m_list_left[NUM_VIDEO], m_list_right[NUM_VIDEO], m_list_mid[NUM_VIDEO];
	double m_dir[NUM_VIDEO], m_lane_width[NUM_VIDEO];
	int m_mid_pos[NUM_VIDEO];
	
private:
	void DoubleThresh(IplImage* src_gray, IplImage* src_edge, IplImage* dst, double thresh);
	bool GetBirdEyeView(int num);
	void GetEdgeMap(int num);
	void GetIntensityMap(int num);
	void ProcessEdge(int num);
	void ProcessIntensity(int num);
	void GroupLineLeft(int num);
	void GroupLineRight(int num);
	void TrackLineLeft(int num);
	void TrackLineRight(int num);
	void EstimateLines(int num);
	void EstimateCurves(int num);
	CvScalar FitCurve(const vector<CvPoint2D64f>& list);
	void MarkImage(int num);
	CvScalar GetLaneMarkVehicleCoord(const CvScalar& curve);
	void GetCurveVehicle();
	void DetectStopLine();
};

