#include "stdafx.h"
#include "LaneMarkingScanner.h"

int lms_is_equal(const void* _a, const void* _b, void* userdata)
{
	CvScalar a = *(const CvScalar*)_a;
	CvScalar b = *(const CvScalar*)_b;
	double y = *(double*)userdata;

	double x_a = a.val[0] * y + a.val[1];
	double x_b = b.val[0] * y + b.val[1];

	double d_angle = fabs(a.val[0] - b.val[0]);
	double d_dist = fabs(x_a - x_b);

	return (d_angle < 0.1 && d_dist < 40);
}

LaneMarkingScanner::LaneMarkingScanner()
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_imgGray[i] = NULL;
		m_imgEdge[i] = NULL;
		m_imgEdgeColor[i] = NULL;
		m_imgIntensity[i] = NULL;
		m_imgIntensityColor[i] = NULL;
		m_imgResult[i] = NULL;
		m_homography[i][8] = 0;
		m_homography_inv[i][8] = 0;
	}
}

LaneMarkingScanner::~LaneMarkingScanner(void)
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if (m_imgGray[i])
		{
			cvReleaseImage(&m_imgGray[i]);
			m_imgGray[i] = NULL;
		}
		if (m_imgEdge[i])
		{
			cvReleaseImage(&m_imgEdge[i]);
			m_imgEdge[i] = NULL;
		}
		if (m_imgEdgeColor[i])
		{
			cvReleaseImage(&m_imgEdgeColor[i]);
			m_imgEdgeColor[i] = NULL;
		}
		if (m_imgIntensity[i])
		{
			cvReleaseImage(&m_imgIntensity[i]);
			m_imgIntensity[i] = NULL;
		}
		if (m_imgIntensityColor[i])
		{
			cvReleaseImage(&m_imgIntensityColor[i]);
			m_imgIntensityColor[i] = NULL;
		}
		if (m_imgResult[i])
		{
			cvReleaseImage(&m_imgResult[i]);
			m_imgResult[i] = NULL;
		}
	}
}

void LaneMarkingScanner::Initialize()
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_imgGray[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
		m_imgEdge[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
		//m_imgEdgeColor[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 3);
		m_imgIntensity[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
		//m_imgIntensityColor[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 3);
		m_imgResult[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 3);

		m_horizon[i] = 0;
		m_lane_width[i] = 0;
		m_mid_pos[i] = 0;
		m_dir[i] = 0.0;
		m_line_left[i] = cvScalar(0);
		m_line_right[i] = cvScalar(0);
	}

	m_height_skip = 30;
	m_lanewidth_vehicle = 0.0;
	m_curve_left = cvScalar(0);
	m_curve_right = cvScalar(0);
	m_curve_mid = cvScalar(0);
	m_vehicle_left = cvScalar(0);
	m_vehicle_right = cvScalar(0);
	m_vehicle_mid = cvScalar(0);
	m_stopline = cvScalar(0);

	m_rect_intensity[0].x = (int)(m_size.width * 0.3);
	m_rect_intensity[0].y = (int)(m_size.height * 0.6);
	m_rect_intensity[0].width = (int)(m_size.width * 0.4);
	m_rect_intensity[0].height = (int)(m_size.height * 0.05);
	
	m_rect_intensity[1].x = (int)(m_size.width * 0.3);
	m_rect_intensity[1].y = (int)(m_size.height * 0.6);
	m_rect_intensity[1].width = (int)(m_size.width * 0.4);
	m_rect_intensity[1].height = (int)(m_size.height * 0.05);
	
	m_rect_intensity[2].x = (int)(m_size.width * 0.3);
	m_rect_intensity[2].y = (int)(m_size.height * 0.6);
	m_rect_intensity[2].width = (int)(m_size.width * 0.4);
	m_rect_intensity[2].height = (int)(m_size.height * 0.05);

	m_rect_line[0].x = 5;
	m_rect_line[0].y = (int)(m_size.height * 0.4);
	m_rect_line[0].width = m_size.width - 10;
	m_rect_line[0].height = (int)(m_size.height * 0.4);

	m_rect_line[1].x = 5;
	m_rect_line[1].y = (int)(m_size.height * 0.4);
	m_rect_line[1].width = m_size.width - 10;
	m_rect_line[1].height = (int)(m_size.height * 0.4);
	
	m_rect_line[2].x = 5;
	m_rect_line[2].y = (int)(m_size.height * 0.4);
	m_rect_line[2].width = m_size.width - 10;
	m_rect_line[2].height = (int)(m_size.height * 0.4);

	m_a_min[0] = 0;
	m_a_max[0] = 1.5;
	m_a_min[1] = -1.5;
	m_a_max[1] = -0.5;
	m_a_min[2] = -1.5;
	m_a_max[2] = -0.5;
	
	m_model_single.Initialization(DSMF_TYPE_SINGLE);
	m_model_double.Initialization(DSMF_TYPE_DOUBLE);

	cvInitFont(&m_font, CV_FONT_HERSHEY_SIMPLEX, 0.5f, 0.5f);
}

void LaneMarkingScanner::ProcessFrame()
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if (m_img[i])
		{
			if (m_img[i]->nChannels == 3)
			{
				cvCvtColor(m_img[i], m_imgGray[i], CV_BGR2GRAY);
				cvCopy(m_img[i], m_imgResult[i]);
			}
			else
			{
				cvCvtColor(m_img[i], m_imgResult[i], CV_GRAY2BGR);
				cvCopy(m_img[i], m_imgGray[i]);
			}
			EstimateLines(i);
		}
	}
	EstimateCurves();
	GetCurveVehicle();
	DetectStopLine();
	//MarkImage();
}

void LaneMarkingScanner::DoubleThresh(IplImage* src_gray, IplImage* src_edge, IplImage* dst, double thresh)
{
	int row, col;
	unsigned char *ptr_gray, *ptr_edge, *ptr_dst;

	ptr_gray = (unsigned char*)src_gray->imageData;
	ptr_edge = (unsigned char*)src_edge->imageData;
	ptr_dst = (unsigned char*)dst->imageData;

	for (row = 0; row < m_size.height; row++)
	{
		for (col = 0; col < m_size.width; col++)
		{
			ptr_dst[col] = (ptr_edge[col] == 255 && ptr_gray[col] < thresh) ? 0 : ptr_edge[col];
		}
		ptr_gray += src_gray->widthStep;
		ptr_edge += src_edge->widthStep;
		ptr_dst += dst->widthStep;
	}
}

void LaneMarkingScanner::EstimateLines(int num)
{
	int lane_width;

	m_stopline = cvScalar(0);
	m_vec_stopline.clear();

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_line_left[i] = cvScalar(0);
		m_line_right[i] = cvScalar(0);
		m_vec_line_left[i].clear();
		m_vec_line_right[i].clear();

		GetEdgeMap(i);
		GetIntensityMap(i);
		ProcessEdge(i);
		ProcessIntensity(i);
		GroupLineLeft(i);
		GroupLineRight(i);
		TrackLineLeft(i);
		TrackLineRight(i);

		lane_width = (int)((m_line_right[i].val[0] - m_line_left[i].val[0]) * m_size.height + (m_line_right[i].val[1] - m_line_left[i].val[1]));
		if (m_line_left[i].val[3] >= 0.8 && m_line_right[i].val[3] >= 0.8)
		{
			//Calculate and update the parameters necessary, if both left and right line can be detected instead of predicted.
			m_horizon[i] = (int)((m_line_left[i].val[1] - m_line_right[i].val[1]) / (m_line_right[i].val[0] - m_line_left[i].val[0]));
			m_mid_pos[i] = (int)(((m_line_left[i].val[0] + m_line_right[i].val[0]) * m_size.height + (m_line_left[i].val[1] + m_line_right[i].val[1])) / 2);
			m_lane_width[i] = lane_width;
			m_dir[i] = (m_line_left[i].val[0] + m_line_right[i].val[0]) / 2;
		}
	}
	m_left_type = (m_line_left[0].val[2] >= 0.8 * m_rect_line[0].height) ? LMS_TYPE_SOLID : LMS_TYPE_DASHED;
	m_right_type = (m_line_right[0].val[2] >= 0.8 * m_rect_line[0].height) ? LMS_TYPE_SOLID : LMS_TYPE_DASHED;
}

void LaneMarkingScanner::DetectStopLine()
{
	CvPoint pt[2], pt_temp;
	int x_limit_left, x_limit_right;

	if ((int)m_vec_stopline.size() >= 2 && m_curve_left.val[3] >= 0.4 && m_curve_right.val[3] >= 0.4)
	{
		for (int i = 0; i < (int)m_vec_stopline.size(); i += 2)
		{
			pt[0] = m_vec_stopline[i];
			pt[1] = m_vec_stopline[i+1];
			
			if (pt[0].x > pt[1].x)
			{
				pt_temp = pt[1];
				pt[1] = pt[0];
				pt[0] = pt_temp;
			}
			x_limit_left = (int)(m_curve_left.val[0] * pt[0].y * pt[0].y + m_curve_left.val[1] * pt[0].y + m_curve_left.val[2]);
			x_limit_right = (int)(m_curve_right.val[0] * pt[0].y * pt[0].y + m_curve_right.val[1] * pt[0].y + m_curve_right.val[2]);
			
			if ((pt[0].x + pt[1].x) / 2 > x_limit_left && (pt[0].x + pt[1].x) / 2 < x_limit_right)
			{
				m_stopline.val[1] = (double)(pt[0].y - pt[1].y) / (pt[0].x - pt[1].x);
				m_stopline.val[1] = (double)(pt[0].y * pt[1].x - pt[1].y * pt[0].x) / (pt[1].x - pt[0].x);
				m_stopline.val[2] = 0;
				m_stopline.val[3] = 1;
				break;
			}
		}
	}
	m_stopline_dist = GetStopLineDist();
}

void LaneMarkingScanner::GetEdgeMap(int num)
{
	CvScalar avg;
	
	cvSetImageROI(m_imgGray[num], m_rect_intensity[num]);
	avg = cvAvg(m_imgGray[num]);
	cvResetImageROI(m_imgGray[num]);

	cvCanny(m_imgGray[num], m_imgEdge[num], 60, 150);
	DoubleThresh(m_imgGray[num], m_imgEdge[num], m_imgEdge[num], avg.val[0] + 10);
	//cvCvtColor(m_imgEdge[num], m_imgEdgeColor[num], CV_GRAY2BGR);
}

void LaneMarkingScanner::ProcessEdge(int num)
{
	double a, b, x;
	int minlength = m_size.height / 20;
	int maxgap = minlength;
	
	cvSetImageROI(m_imgEdge[num], m_rect_line[num]);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
		
	lines = cvHoughLines2(m_imgEdge[num], storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 50, minlength, maxgap);
	cvResetImageROI(m_imgEdge[num]);
	
	for(int i = 0; i < lines->total; i++)
	{
		CvPoint* pt = (CvPoint*)cvGetSeqElem(lines, i);
		pt[0].x += m_rect_line[num].x;
		pt[0].y += m_rect_line[num].y;
		pt[1].x += m_rect_line[num].x;
		pt[1].y += m_rect_line[num].y;

		
		//Filter out lines that are strictly vertical or close to be horizontal
		if (fabs((double)(pt[1].y - pt[0].y) / (pt[1].x - pt[0].x)) < 0.02 && abs(pt[0].x - pt[1].x) > 0.2 * m_size.width)
		{
			if (num == 0)
			{
				m_vec_stopline.push_back(pt[0]);
				m_vec_stopline.push_back(pt[1]);
				//cvLine(m_imgEdgeColor[num], pt[0], pt[1], CV_RGB(255, 255, 0), 2);
			}
		}
		else		
		{
			a = (double)(pt[0].x - pt[1].x) / (pt[0].y - pt[1].y);
			b = (double)(pt[0].x * pt[1].y - pt[1].x * pt[0].y) / (pt[1].y - pt[0].y);
			x = a * m_rect_line[num].y + b;
						
			if (num != 1)
			{
				if (x < m_rect_line[num].x + m_rect_line[num].width / 2 && a >= -m_a_max[num] && a <= -m_a_min[num])
				{
					//cvLine(m_imgEdgeColor[num], pt[0], pt[1], CV_RGB(128, 128, 0), 2);
					m_vec_line_left[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
				}
			}
			if (num != 2)
			{
				if (x >= m_rect_line[num].x + m_rect_line[num].width / 2 && a >= m_a_min[num] && a <= m_a_max[num])
				{
					//cvLine(m_imgEdgeColor[num], pt[0], pt[1], CV_RGB(128, 0, 128), 2);
					m_vec_line_right[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
				}
			}
		}
	}
	cvReleaseMemStorage(&storage);
}

void LaneMarkingScanner::GetIntensityMap(int num)
{
	int nTopWidth = 4, nBottomWidth = 26, nCurWidth, col, row;
	unsigned char *lanebase_gray, *lanebase_intensity;
	double sum;
	bool* bNS = new bool[m_rect_line[num].width];
	
	lanebase_gray = (unsigned char*)(&m_imgGray[num]->imageData[(m_rect_line[num].y + m_rect_line[num].height) * m_imgGray[num]->widthStep]);
	lanebase_intensity = (unsigned char*)(&m_imgIntensity[num]->imageData[(m_rect_line[num].y + m_rect_line[num].height) * m_imgIntensity[num]->widthStep]);
	cvZero(m_imgIntensity[num]);

	for (row = m_rect_line[num].y + m_rect_line[num].height; row > m_rect_line[num].y; row--)
	{
		nCurWidth = (int)((double)(nBottomWidth - nTopWidth) / m_rect_line[num].height * (row - m_rect_line[num].y)) + nTopWidth;
		for (col = nCurWidth + m_rect_line[num].x; col <= m_rect_line[num].x + m_rect_line[num].width - nCurWidth; col++)
		{
			sum = 0;
			for (int i = -nCurWidth; i < -nCurWidth / 2; i++)
			{
				sum -= lanebase_gray[col + i];
			}
			for (int i = -nCurWidth / 2; i < nCurWidth / 2; i++)
			{
				sum += lanebase_gray[col + i];
			}
			for (int i = nCurWidth / 2; i < nCurWidth; i++)
			{
				sum -= lanebase_gray[col + i];
			}
			sum /= nCurWidth;
			if (sum < 0) sum = 0;
			lanebase_intensity[col - m_rect_line[num].x] = (unsigned char)sum;
		}

		for (col = 0; col < m_rect_line[num].width; col++)
		{
			bNS[col] = false;
		}

		for (col = nCurWidth + m_rect_line[num].x; col <= m_rect_line[num].x + m_rect_line[num].width - nCurWidth; col++)
		{
			if (lanebase_intensity[col] > lanebase_intensity[col - 1]
			&& lanebase_intensity[col] > lanebase_intensity[col + 1]
			&& lanebase_intensity[col] > 10)
			{
				bNS[col] = true;
			}
			else
			{
				bNS[col] = false;
			}
		}

		for (col = 0; col < m_rect_line[num].width; col++)
		{
			if (bNS[col])
			{
				lanebase_intensity[col + m_rect_line[num].x] = 255;
			}
			else
			{
				lanebase_intensity[col + m_rect_line[num].x] = 0;
			}
		}
		lanebase_gray -= m_imgGray[num]->widthStep;
		lanebase_intensity -= m_imgIntensity[num]->widthStep;
	}
	//cvCvtColor(m_imgIntensity[num], m_imgIntensityColor[num], CV_GRAY2BGR);

	delete[] bNS;
}

void LaneMarkingScanner::ProcessIntensity(int num)
{
	int minlength = m_size.height / 20;
	int maxgap = minlength;
	double a, b, x;
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;
	
	lines = cvHoughLines2(m_imgIntensity[num], storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 20, minlength, maxgap);
	for (int i = 0; i < lines->total; i++)
	{
		CvPoint* pt = (CvPoint*)cvGetSeqElem(lines, i);

		if (pt[1].x == pt[0].x || pt[1].y == pt[0].y)
			continue;

		a = (double)(pt[0].x - pt[1].x) / (pt[0].y - pt[1].y);
		b = (double)(pt[0].x * pt[1].y - pt[1].x * pt[0].y) / (pt[1].y - pt[0].y);
		x = a * m_rect_line[num].y + b;

		if (num != 1)
		{
			if (x < m_rect_line[num].x + m_rect_line[num].width / 2 && a >= -m_a_max[num] && a <= -m_a_min[num])
			{
				//cvLine(m_imgIntensityColor[num], pt[0], pt[1], CV_RGB(128, 128, 0), 2);
				m_vec_line_left[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
			}
		}
		if (num != 2)
		{
			if (x >= m_rect_line[num].x + m_rect_line[num].width / 2 && a >= m_a_min[num] && a <= m_a_max[num])
			{
				//cvLine(m_imgIntensityColor[num], pt[0], pt[1], CV_RGB(128, 0, 128), 2);
				m_vec_line_right[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
			}
		}
	}
	cvReleaseMemStorage(&storage);
}

void LaneMarkingScanner::GroupLineLeft(int num)
{
	double a, b, x_lane = 0;
	double nHeight = m_rect_line[num].y + m_rect_line[num].height;
	bool bFound = false;
	int nSize = (int)m_vec_line_left[num].size();
	CvScalar line_current = cvScalar(0);

	if (nSize >= 2)
	{
		CvMemStorage* mem = cvCreateMemStorage(0);
		CvSeq* seq_line = cvCreateSeq(CV_64FC4, sizeof(CvSeq), sizeof(CvScalar), mem);
		CvSeq* labels = 0;

		for (int i = 0; i < nSize; i++)
		{
			cvSeqPush(seq_line, &m_vec_line_left[num][i]);
		}

		int class_count = cvSeqPartition(seq_line, 0, &labels, lms_is_equal, &nHeight);

		CvScalar** line = new CvScalar*[class_count];
		int* count = new int[class_count];
		CvScalar* average = new CvScalar[class_count];

		for (int i = 0; i < class_count; i++)
		{
			line[i] = new CvScalar[nSize];
			count[i] = 0;
			for (int j = 0; j < 4; j++)
				average[i].val[j] = 0;
		}

		for (int i = 0; i < labels->total; i++)
		{
			CvScalar scalar = *(CvScalar*)cvGetSeqElem(seq_line, i);
			int label = *(int*)cvGetSeqElem(labels, i);
			line[label][count[label]] = scalar;
			count[label]++;
		}

		for (int i = 0; i < class_count; i++)
		{
			if (count[i] >= 2)
			{
				for (int j = 0; j < count[i]; j++)
				{
					average[i].val[0] += line[i][j].val[0];
					average[i].val[1] += line[i][j].val[1];
					if (line[i][j].val[2] > average[i].val[2])
						average[i].val[2] = line[i][j].val[2];
					average[i].val[3] += line[i][j].val[3];
				}
				average[i].val[0] /= count[i];
				average[i].val[1] /= count[i];
				average[i].val[3] /= count[i];
			}
		}

		for (int i = 0; i < class_count; i++)
		{
			if (count[i] >= 2)
			{
				a = average[i].val[0];
				b = average[i].val[1];
				
				if (m_line_left[num].val[3] >= 0.95)
				{
					double a0 = m_line_left[num].val[0];
					double b0 = m_line_left[num].val[1];
					if (i == 0 || abs(a - a0) * 50 + abs(a * nHeight + b - a0 * nHeight - b0) < x_lane)
					{
						x_lane = abs(a - a0) * 50 + abs(a * nHeight + b - a0 * nHeight - b0);
						line_current = average[i];
					}
				}
				else if (i == 0 || (a * nHeight + b) > x_lane)
				{
					x_lane = a * nHeight + b;
					line_current = average[i];
				}
				bFound = true;
			}
		}
		if (!bFound)
			m_line_left[num] = cvScalar(0);
		else
			m_line_left[num] = line_current;

		cvReleaseMemStorage(&mem);
		for (int i = 0; i < class_count; i++)
			delete[] line[i];
		delete[] line;
		delete[] average;
		delete[] count;
	}
	else
	{
		m_line_left[num] = cvScalar(0);
	}
}

void LaneMarkingScanner::GroupLineRight(int num)
{
	double a, b, x_lane = 0;
	double nHeight = m_rect_line[num].y + m_rect_line[num].height;
	bool bFound = false;
	int nSize = (int)m_vec_line_right[num].size();
	CvScalar line_current = cvScalar(0);

	if (nSize >= 2)
	{
		CvMemStorage* mem = cvCreateMemStorage(0);
		CvSeq* seq_line = cvCreateSeq(CV_64FC4, sizeof(CvSeq), sizeof(CvScalar), mem);
		CvSeq* labels = 0;

		for (int i = 0; i < nSize; i++)
		{
			cvSeqPush(seq_line, &m_vec_line_right[num][i]);
		}

		int class_count = cvSeqPartition(seq_line, 0, &labels, lms_is_equal, &nHeight);

		CvScalar** line = new CvScalar*[class_count];
		int* count = new int[class_count];
		CvScalar* average = new CvScalar[class_count];

		for (int i = 0; i < class_count; i++)
		{
			line[i] = new CvScalar[nSize];
			count[i] = 0;
			for (int j = 0; j < 4; j++)
				average[i].val[j] = 0;
		}

		for (int i = 0; i < labels->total; i++)
		{
			CvScalar scalar = *(CvScalar*)cvGetSeqElem(seq_line, i);
			int label = *(int*)cvGetSeqElem(labels, i);
			line[label][count[label]] = scalar;
			count[label]++;
		}

		for (int i = 0; i < class_count; i++)
		{
			if (count[i] >= 2)
			{
				for (int j = 0; j < count[i]; j++)
				{
					average[i].val[0] += line[i][j].val[0];
					average[i].val[1] += line[i][j].val[1];
					if (line[i][j].val[2] > average[i].val[2])
						average[i].val[2] = line[i][j].val[2];
					average[i].val[3] += line[i][j].val[3];
				}
				average[i].val[0] /= count[i];
				average[i].val[1] /= count[i];
				average[i].val[3] /= count[i];
			}
		}

		for (int i = 0; i < class_count; i++)
		{
			if (count[i] >= 2)
			{
				a = average[i].val[0];
				b = average[i].val[1];

				if (m_line_right[num].val[3] >= 0.95)
				{
					double a0 = m_line_right[num].val[0];
					double b0 = m_line_right[num].val[1];
					if (i == 0 || abs(a - a0) * 50 + abs(a * nHeight + b - a0 * nHeight - b0) < x_lane)
					{
						x_lane = abs(a - a0) * 50 + abs(a * nHeight + b - a0 * nHeight - b0);
						line_current = average[i];
					}
				}
				else if (i == 0 || (a * nHeight + b) < x_lane)
				{
					x_lane = a * nHeight + b;
					line_current = average[i];
				}
				bFound = true;
			}
		}
		if (!bFound)
			m_line_right[num] = cvScalar(0);
		else
			m_line_right[num] = line_current;

		cvReleaseMemStorage(&mem);
		for (int i = 0; i < class_count; i++)
			delete[] line[i];
		delete[] line;
		delete[] average;
		delete[] count;
	}
	else
	{
		m_line_right[num] = cvScalar(0);
	}
}

void LaneMarkingScanner::TrackLineLeft(int num)
{
	static CvScalar line[NUM_VIDEO];
	static CvScalar changed_line[NUM_VIDEO];
	static bool bTrack[NUM_VIDEO];
	//indicate how many consecutive frames that the lane can be detected
	static int counter_track[NUM_VIDEO];
	//indicate how many consecutive frames that the lane can not be detected
	static int counter_notrack[NUM_VIDEO];

	if (bTrack[num])
	{
		if (m_line_left[num].val[3] == 0.0 ||
			fabs(m_line_left[num].val[0] - line[num].val[0]) >= LMS_TRACKER_LINE_PARAM_A ||
			fabs(m_line_left[num].val[1] - line[num].val[1]) >= LMS_TRACKER_LINE_PARAM_B)
		{
			//no lines in the tracking area
			counter_notrack[num]++;
			line[num].val[3] = 0.5;
			if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM1)
			{
				bTrack[num] = false;
				counter_notrack[num] = 0;
			}
		}
		else
		{
			//there are lines in the tracking area, perform a smooth filtering
			for (int i = 0; i < 4; i++)
				line[num].val[i] = (line[num].val[i] + m_line_left[num].val[i]) / 2;
			counter_notrack[num] = 0;
		}
	}
	else
	{
		if (changed_line[num].val[3] == 0.0 && m_line_left[num].val[3] == 0)
		{
			line[num] = cvScalar(0);
		}

		if (changed_line[num].val[3] == 0.0 && m_line_left[num].val[3] > 0.5)
		{
			changed_line[num] = m_line_left[num];
			counter_track[num]++;
		}
		if (changed_line[num].val[3] > 0.5 && m_line_left[num].val[3] == 0.0)
		{
			counter_track[num] = 0;
			counter_notrack[num]++;
			if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
			{
				counter_notrack[num] = 0;
				line[num] = cvScalar(0);
				changed_line[num] = cvScalar(0);
			}
		}
		if (changed_line[num].val[3] > 0.5 && m_line_left[num].val[3] > 0.5)
		{
			if(fabs(m_line_left[num].val[0] - changed_line[num].val[0]) >= LMS_TRACKER_LINE_PARAM_A ||
				fabs(m_line_left[num].val[1] - changed_line[num].val[1]) >= LMS_TRACKER_LINE_PARAM_B)
			{
				//the current line is far from the previous line
				changed_line[num] = m_line_left[num];
				counter_track[num] = 0;
				counter_notrack[num]++;
				if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
				{
					counter_notrack[num] = 0;
					line[num] = cvScalar(0);
					changed_line[num] = cvScalar(0);
				}
			}
			else
			{
				//the current line is near the previous line
				for (int i = 0; i < 4; i++)
					changed_line[num].val[i] = (changed_line[num].val[i] + m_line_left[num].val[i]) / 2;

				counter_track[num]++;
				counter_notrack[num] = 0;
				if (counter_track[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
				{
					bTrack[num] = true;
					counter_track[num] = 0;
					line[num] = changed_line[num];
					changed_line[num] = cvScalar(0);
				}
			}
		}
	}
	m_line_left[num] = line[num];
}

void LaneMarkingScanner::TrackLineRight(int num)
{
	static CvScalar line[NUM_VIDEO];
	static CvScalar changed_line[NUM_VIDEO];
	static bool bTrack[NUM_VIDEO];
	//indicate how many consecutive frames that the lane can be detected
	static int counter_track[NUM_VIDEO];
	//indicate how many consecutive frames that the lane can not be detected
	static int counter_notrack[NUM_VIDEO];


	if (bTrack[num])
	{
		if (m_line_right[num].val[3] == 0.0 ||
			fabs(m_line_right[num].val[0] - line[num].val[0]) >= LMS_TRACKER_LINE_PARAM_A ||
			fabs(m_line_right[num].val[1] - line[num].val[1]) >= LMS_TRACKER_LINE_PARAM_B)
		{
			//no lines in the tracking area
			counter_notrack[num]++;
			line[num].val[3] = 0.5;
			if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM1)
			{
				bTrack[num] = false;
				counter_notrack[num] = 0;
			}
		}
		else
		{
			//there are lines in the tracking area, perform a smooth filtering
			for (int i = 0; i < 4; i++)
				line[num].val[i] = (line[num].val[i] + m_line_right[num].val[i]) / 2;
			counter_notrack[num] = 0;
		}
	}
	else
	{
		if (changed_line[num].val[3] == 0.0 && m_line_right[num].val[3] == 0)
		{
			line[num] = cvScalar(0);
		}

		if (changed_line[num].val[3] == 0.0 && m_line_right[num].val[3] > 0.5)
		{
			changed_line[num] = m_line_right[num];
			counter_track[num]++;
		}
		if (changed_line[num].val[3] > 0.5 && m_line_right[num].val[3] == 0.0)
		{
			counter_track[num] = 0;
			counter_notrack[num]++;
			if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
			{
				counter_notrack[num] = 0;
				line[num] = cvScalar(0);
				changed_line[num] = cvScalar(0);
			}
		}
		if (changed_line[num].val[3] > 0.5 && m_line_right[num].val[3] > 0.5)
		{
			if(fabs(m_line_right[num].val[0] - changed_line[num].val[0]) >= LMS_TRACKER_LINE_PARAM_A ||
				fabs(m_line_right[num].val[1] - changed_line[num].val[1]) >= LMS_TRACKER_LINE_PARAM_B)
			{
				//the current line is far from the previous line
				changed_line[num] = m_line_right[num];
				counter_track[num] = 0;
				counter_notrack[num]++;
				if (counter_notrack[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
				{
					counter_notrack[num] = 0;
					line[num] = cvScalar(0);
					changed_line[num] = cvScalar(0);
				}
			}
			else
			{
				//the current line is near the previous line
				for (int i = 0; i < 4; i++)
					changed_line[num].val[i] = (changed_line[num].val[i] + m_line_right[num].val[i]) / 2;

				counter_track[num]++;
				counter_notrack[num] = 0;
				if (counter_track[num] == LMS_TRACKER_TIMES_TO_CONFIRM2)
				{
					bTrack[num] = true;
					counter_track[num] = 0;
					line[num] = changed_line[num];
					changed_line[num] = cvScalar(0);
				}
			}
		}
	}
	m_line_right[num] = line[num];
}

void LaneMarkingScanner::EstimateCurves()
{
	unsigned char * lanebase;
	double lane_width_step;
	double current_lane_width;
	bool left_found, right_found;
	//Flags used to remember on which side a boundary point is found.
	int x_left = 0, x_right = 0, x, begin_x;

	m_curve_left = cvScalar(0);
	m_curve_right = cvScalar(0);
	m_curve_mid = cvScalar(0);
	m_vehicle_left = cvScalar(0);
	m_vehicle_right = cvScalar(0);
	m_vehicle_mid = cvScalar(0);
	m_list_left.clear();
	m_list_right.clear();
	m_list_mid.clear();

	if (m_line_left[0].val[3] >= 0.5)
	{
		for (int y = m_rect_line[0].y + m_rect_line[0].height; y > m_rect_line[0].y; y--)
		{
			x = (int)(m_line_left[0].val[0] * y + m_line_left[0].val[1]);
			m_list_left.push_back(cvPoint(x, y));
		}
	}
	if (m_line_right[0].val[3] >= 0.5)
	{
		for (int y = m_rect_line[0].y + m_rect_line[0].height; y > m_rect_line[0].y; y--)
		{
			x = (int)(m_line_right[0].val[0] * y + m_line_right[0].val[1]);
			m_list_right.push_back(cvPoint(x, y));
		}
	}

	//Initialize the variables used in the scan.
	lanebase = (unsigned char*)(&m_imgEdge[0]->imageData[m_rect_line[0].y * m_imgEdge[0]->widthStep]);
	begin_x = m_mid_pos[0] - m_dir[0] * (m_size.height - m_rect_line[0].y);
	lane_width_step = (double)m_lane_width[0] / (2 * (m_size.height - m_horizon[0]));
	current_lane_width = m_lane_width[0] / 2 - lane_width_step * (m_size.height - m_rect_line[0].y);

	//Scan from bottom upwards.
	for(int y = m_rect_line[0].y; y > max(0, m_horizon[0] + m_height_skip); y--, current_lane_width -= lane_width_step, lanebase -= m_imgEdge[0]->widthStep)
	{
		left_found = false;
		right_found = false;
		m_list_mid.push_back(cvPoint((int)begin_x, y));

		//From the middle to each side, find the appropriate points in the predicted position.
		if (m_line_left[0].val[3] >= 0.5)
		{
			for(x_left = min((int)(begin_x - current_lane_width * LMS_WIDTH_MIN_THRESHOLD), m_imgEdge[0]->width - 1);
				x_left >= 5 && x_left > (int)(begin_x - current_lane_width * LMS_WIDTH_MAX_THRESHOLD);
				x_left--)
			{
				if(lanebase[x_left] != 0)
				{
					left_found = true;
					break;
				}
			}
		}
		if (m_line_right[0].val[3] >= 0.5)
		{
			for(x_right = max((int)(begin_x + current_lane_width * LMS_WIDTH_MIN_THRESHOLD), 0);
				x_right < m_imgEdge[0]->width - 5 && x_right < (int)(begin_x + current_lane_width * LMS_WIDTH_MAX_THRESHOLD);
				x_right++)
			{
				if(lanebase[x_right] != 0)
				{
					right_found = true;
					break;
				}
			}
		}

		//Update the variables, according to the points detected from both side.
		if(left_found)
		{
			if(right_found)
			{
				begin_x = (int)(x_left + x_right - m_dir[0]) / 2;
				m_list_left.push_back(cvPoint(x_left - 2, y));
				m_list_right.push_back(cvPoint(x_right + 2, y));
			}
			else
			{
				begin_x = (int)(x_left + current_lane_width - m_dir[0]);
				m_list_left.push_back(cvPoint(x_left - 2, y));
			}
		}
		else
		{
			if(right_found)
			{
				begin_x = (int)(x_right - current_lane_width - m_dir[0]);
				m_list_right.push_back(cvPoint(x_right + 2, y));
			}
			else
			{
				begin_x -= m_dir[0];
			}
		}
	}

	if (m_line_left[0].val[3] >= 0.5 && m_line_right[0].val[3] >= 0.5)
	{
		if (m_model_double.FitList(m_list_left, m_list_right, m_horizon[0]) == DSMF_SUCCESS)
		{
			m_curve_left = m_model_double.GetLeftResult();
			m_curve_right = m_model_double.GetRightResult();
		}
	}
	else if (m_line_left[0].val[3] >= 0.5)
	{
		if (m_model_single.SingleFitList(m_list_left, m_horizon[0]) == DSMF_SUCCESS)
		{
			m_curve_left = m_model_single.GetSingleResult();
		}
	}
	else if (m_line_right[0].val[3] >= 0.5)
	{
		if (m_model_single.SingleFitList(m_list_right, m_horizon[0]) == DSMF_SUCCESS)
		{
			m_curve_right = m_model_single.GetSingleResult();
		}
	}
}

void LaneMarkingScanner::MarkImage()
{
	CvPoint p1, p2;
	CvPoint p_last, p_current;
	CvScalar color;
	int y;
	char text[120];

	//Draw the area where the hough transform is performed
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		cvRectangle(m_imgResult[i], cvPoint(m_rect_line[i].x, m_rect_line[i].y),
			cvPoint(m_rect_line[i].x + m_rect_line[i].width, m_rect_line[i].y + m_rect_line[i].height), CV_RGB(0, 255, 0), 1);
		cvRectangle(m_imgResult[i], cvPoint(m_rect_intensity[i].x, m_rect_intensity[i].y),
			cvPoint(m_rect_intensity[i].x + m_rect_intensity[i].width, m_rect_intensity[i].y + m_rect_intensity[i].height), CV_RGB(0, 255, 0), 1);

		//Draw the horizon
		if (m_horizon[i] >= 0)
			cvLine(m_imgResult[i], cvPoint(0, m_horizon[i]), cvPoint(m_size.width, m_horizon[i]), CV_RGB(255, 255, 0), 2);
		
		sprintf(text, "m_line_left=%.2f, %.2f, %.2f, %.2f", m_line_left[i].val[0], m_line_left[i].val[1], m_line_left[i].val[2], m_line_left[i].val[3]);
		cvPutText(m_imgResult[i], text, cvPoint(0, 15), &m_font, CV_RGB(0, 0, 255));
		sprintf(text, "m_line_right=%.2f, %.2f, %.2f, %.2f", m_line_right[i].val[0], m_line_right[i].val[1], m_line_right[i].val[2], m_line_right[i].val[3]);
		cvPutText(m_imgResult[i], text, cvPoint(0, 30), &m_font, CV_RGB(0, 0, 255));
	}
	if (m_left_type == LMS_TYPE_SOLID)
		strcpy(text, "Solid");
	else
		strcpy(text, "Dashed");
	cvPutText(m_imgResult[0], text, cvPoint(m_size.width / 2, 15), &m_font, CV_RGB(0, 0, 255));

	if (m_right_type == LMS_TYPE_SOLID)
		strcpy(text, "Solid");
	else
		strcpy(text, "Dashed");
	cvPutText(m_imgResult[0], text, cvPoint(m_size.width / 2, 30), &m_font, CV_RGB(0, 0, 255));

	if (m_stopline.val[3] > 0.5)
	{
		p1.x = 0;
		p1.y = (int)m_stopline.val[1];
		p2.x = m_size.width - 1;
		p2.y = (int)(m_stopline.val[0] * p2.x + m_stopline.val[1]);
		cvLine(m_imgResult[0], p1, p2, CV_RGB(0, 255,255), 2);
	}
			
	//Mark the points on the boundaries.
	for(int i = 0; i < (int)m_list_left.size(); i++)
	{
		cvCircle(m_imgResult[0], cvPoint(m_list_left[i].x, m_list_left[i].y), 2, CV_RGB(255, 255, 0), 2);
	}
	for(int i = 0; i < (int)m_list_right.size(); i++)
	{
		cvCircle(m_imgResult[0], cvPoint(m_list_right[i].x, m_list_right[i].y), 2, CV_RGB(255, 0, 255), 2);
	}

	if(m_curve_left.val[3] >= 0.4)
	{
		if (m_curve_left.val[3] >= 0.5)
			color = CV_RGB(255, 0, 0);
		else
			color = CV_RGB(255, 128, 0);

		y = m_size.height - 1;
		p_last = cvPoint((int)(m_curve_left.val[0] / (y - m_horizon[0]) + m_curve_left.val[1] * (y - m_horizon[0]) + m_curve_left.val[2]), y);
		for(y = m_size.height - 2; y > max(0, m_horizon[0] + m_height_skip); y--)
		{
			p_current = cvPoint((int)(m_curve_left.val[0] / (y - m_horizon[0]) + m_curve_left.val[1] * (y - m_horizon[0]) + m_curve_left.val[2]), y);
			cvLine(m_imgResult[0], p_last, p_current, color, 2, CV_AA);
			p_last = p_current;
		}
	}
	if(m_curve_right.val[3] >= 0.4)
	{
		if (m_curve_right.val[3] >= 0.5)
			color = CV_RGB(0, 0, 255);
		else
			color = CV_RGB(0, 128, 255);
		
		y = m_size.height - 1;
		p_last = cvPoint((int)(m_curve_right.val[0] / (y - m_horizon[0]) + m_curve_right.val[1] * (y - m_horizon[0]) + m_curve_right.val[2]), y);
		for(y = m_size.height - 2; y > max(0, m_horizon[0] + m_height_skip); y--)
		{
			p_current = cvPoint((int)(m_curve_right.val[0] / (y - m_horizon[0]) + m_curve_right.val[1] * (y - m_horizon[0]) + m_curve_right.val[2]), y);
			cvLine(m_imgResult[0], p_last, p_current, color, 2, CV_AA);
			p_last = p_current;
		}
	}

	if (m_curve_mid.val[3] > 0.5)
	{
		color = CV_RGB(0, 255, 0);

		y = m_size.height - 1;
		p_last = cvPoint((int)(m_curve_mid.val[0] / (y - m_horizon[0]) + m_curve_mid.val[1] * (y - m_horizon[0]) + m_curve_mid.val[2]), y);
		for(y = m_size.height - 2; y > max(0, m_horizon[0] + m_height_skip); y--)
		{
			p_current = cvPoint((int)(m_curve_mid.val[0] / (y - m_horizon[0]) + m_curve_mid.val[1] * (y - m_horizon[0]) + m_curve_mid.val[2]), y);
			cvLine(m_imgResult[0], p_last, p_current, color, 2, CV_AA);
			p_last = p_current;
		}
	}
	sprintf(text, "m_curve_left=%.2f, %.2f, %.2f, %.2f", m_curve_left.val[0], m_curve_left.val[1], m_curve_left.val[2], m_curve_left.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(0, 45), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_curve_right=%.2f, %.2f, %.2f, %.2f", m_curve_right.val[0], m_curve_right.val[1], m_curve_right.val[2], m_curve_right.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(0, 60), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_curve_mid=%.2f, %.2f, %.2f, %.2f", m_curve_mid.val[0], m_curve_mid.val[1], m_curve_mid.val[2], m_curve_mid.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(0, 75), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_vehicle_left=%.2f, %.3f, %.3f, %.3f", m_vehicle_left.val[0], m_vehicle_left.val[1], m_vehicle_left.val[2], m_vehicle_left.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(m_size.width / 2, 45), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_vehicle_right=%.2f, %.3f, %.3f, %.3f", m_vehicle_right.val[0], m_vehicle_right.val[1], m_vehicle_right.val[2], m_vehicle_right.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(m_size.width / 2, 60), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_vehicle_mid=%.2f, %.3f, %.3f, %.3f", m_vehicle_mid.val[0], m_vehicle_mid.val[1], m_vehicle_mid.val[2], m_vehicle_mid.val[3]);
	cvPutText(m_imgResult[0], text, cvPoint(m_size.width / 2, 75), &m_font, CV_RGB(0, 0, 255));
	sprintf(text, "m_lanewidth_vehicle=%f", m_lanewidth_vehicle);
	cvPutText(m_imgResult[0], text, cvPoint(0, 90), &m_font, CV_RGB(0, 0, 255));
}

double LaneMarkingScanner::GetStopLineDist()
{
	double pt[3], PT[3];

	if (m_stopline.val[3] < 0.5)
		return 0;

	pt[0] = (double)m_size.width / 2;
	pt[1] = m_stopline.val[0] * pt[0] + m_stopline.val[1];
	pt[2] = 1;

	for (int i = 0; i < 3; i++)
	{
		PT[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			PT[i] += m_homography_inv[0][i*3+j] * pt[j];
		}
	}
	PT[0] = PT[0] / PT[2];
	PT[1] = PT[1] / PT[2];

	return PT[1];
}

CvScalar LaneMarkingScanner::GetLaneMarkVehicleCoord(const CvScalar& curve)
{
	double pt[3], PT[3];
	double a, b;
	CvMat *A, *B, *X;

	if (fabs(m_homography_inv[0][8]) < 1e-6 || curve.val[3] < 0.5)
		return cvScalar(0);

	A = cvCreateMat(2, 2, CV_64FC1);
	B = cvCreateMat(2, 1, CV_64FC1);
	X = cvCreateMat(2, 1, CV_64FC1);

	pt[1] = (double)m_size.height;
	pt[0] = (double)(curve.val[0] * pt[1] + curve.val[1]);
	pt[2] = 1;
	for (int i = 0; i < 3; i++)
	{
		PT[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			PT[i] += m_homography_inv[0][i*3+j] * pt[j];
		}
	}
	PT[0] = PT[0] / PT[2];
	PT[1] = PT[1] / PT[2];
	cvmSet(A, 0, 0, PT[1]);
	cvmSet(A, 0, 1, 1);
	cvmSet(B, 0, 0, PT[0]);

	pt[1] = (double)(m_rect_line[0].y);
	pt[0] = (double)(curve.val[0] * pt[1] + curve.val[1]);
	pt[2] = 1;
	for (int i = 0; i < 3; i++)
	{
		PT[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			PT[i] += m_homography_inv[0][i*3+j] * pt[j];
		}
	}
	PT[0] = PT[0] / PT[2];
	PT[1] = PT[1] / PT[2];
	cvmSet(A, 1, 0, PT[1]);
	cvmSet(A, 1, 1, 1);
	cvmSet(B, 1, 0, PT[0]);

	cvSolve(A, B, X, CV_LU);

	a = cvmGet(X, 0, 0);
	b = cvmGet(X, 1, 0);
	
	cvReleaseMat(&A);
	cvReleaseMat(&B);
	cvReleaseMat(&X);

	return cvScalar(0, a, b, curve.val[3]);
}

void LaneMarkingScanner::GetCurveVehicle()
{
	m_vehicle_left = GetLaneMarkVehicleCoord(m_line_left[0]);
	m_vehicle_right = GetLaneMarkVehicleCoord(m_line_right[0]);

	if (m_vehicle_left.val[3] >= 0.8 && m_vehicle_right.val[3] >= 0.8)
	{
		m_lanewidth_vehicle = m_vehicle_right.val[2] - m_vehicle_left.val[2];
	}

	if (m_curve_left.val[3] > 0.8 && m_curve_right.val[3] == 0 && m_lane_width[0] > 500 && m_lane_width[0] < 1000)
	{
		m_curve_right.val[0] = m_curve_left.val[0];
		m_curve_right.val[1] = m_curve_left.val[1] + (double)m_lane_width[0] / (m_size.height - m_horizon[0]);
		m_curve_right.val[2] = m_curve_left.val[2];
		m_curve_right.val[3] = 0.4;
	}
	if (m_vehicle_left.val[3] > 0.8 && m_vehicle_right.val[3] == 0 && m_lanewidth_vehicle > 3 && m_lanewidth_vehicle < 4.5)
	{
		m_vehicle_right.val[0] = m_vehicle_left.val[0];
		m_vehicle_right.val[1] = m_vehicle_left.val[1];
		m_vehicle_right.val[2] = m_vehicle_left.val[2] + m_lanewidth_vehicle;
		m_vehicle_right.val[3] = 0.4;
	}
	if (m_curve_right.val[3] > 0.8 && m_curve_left.val[3] == 0 && m_lane_width[0] > 500 && m_lane_width[0] < 1000)
	{
		m_curve_left.val[0] = m_curve_right.val[0];
		m_curve_left.val[1] = m_curve_right.val[1] - (double)m_lane_width[0] / (m_size.height - m_horizon[0]);
		m_curve_left.val[2] = m_curve_right.val[2];
		m_curve_left.val[3] = 0.4;
	}
	if (m_vehicle_right.val[3] > 0.8 && m_vehicle_left.val[3] == 0 && m_lanewidth_vehicle > 3 && m_lanewidth_vehicle < 4.5)
	{
		m_vehicle_left.val[0] = m_vehicle_right.val[0];
		m_vehicle_left.val[1] = m_vehicle_right.val[1];
		m_vehicle_left.val[2] = m_vehicle_right.val[2] - m_lanewidth_vehicle;
		m_vehicle_left.val[3] = 0.4;
	}

	for (int i = 0; i < 4; i++)
	{
		m_curve_mid.val[i] = (m_curve_left.val[i] + m_curve_right.val[i]) / 2;
		m_vehicle_mid.val[i] = (m_vehicle_left.val[i] + m_vehicle_right.val[i]) / 2;
	}
}