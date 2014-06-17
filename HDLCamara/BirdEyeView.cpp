#include "stdafx.h"
#include "BirdEyeView.h"

int bev_is_equal(const void* _a, const void* _b, void* userdata)
{
	CvScalar a = *(const CvScalar*)_a;
	CvScalar b = *(const CvScalar*)_b;
	double y = *(double*)userdata;

	double x_a = a.val[0] * y + a.val[1];
	double x_b = b.val[0] * y + b.val[1];

	double d_angle = fabs(a.val[0] - b.val[0]);
	double d_dist = fabs(x_a - x_b);

	return (d_angle < 0.05 && d_dist < 5);
}

BirdEyeView::BirdEyeView(void)
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_imgGray[i] = NULL;
		m_imgEdge[i] = NULL;
		m_imgEdgeColor[i] = NULL;
		m_imgIntensity[i] = NULL;
		m_imgIntensityColor[i] = NULL;
		m_imgResult[i] = NULL;
		m_imgOriginResult[i] = NULL;
		m_imgOriginGray[i] = NULL;
		m_homography[i][8] = 0;
		m_homography_inv[i][8] = 0;
	}
}

BirdEyeView::~BirdEyeView(void)
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
			m_imgIntensityColor[i] = NULL;
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
		if (m_imgOriginResult[i])
		{
			cvReleaseImage(&m_imgOriginResult[i]);
			m_imgOriginResult[i] = NULL;
		}
		if (m_imgOriginGray[i])
		{
			cvReleaseImage(&m_imgOriginGray[i]);
			m_imgOriginGray[i] = NULL;
		}
	}
}

void BirdEyeView::Initialize()
{
	m_limit_a = 0.1;

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_lane_width[i] = 0;
		m_mid_pos[i] = 0;
		m_dir[i] = 0.0;
		m_line_left[i] = cvScalar(0);
		m_line_right[i] = cvScalar(0);
		m_curve_left[i] = cvScalar(0);
		m_curve_right[i] = cvScalar(0);
		m_curve_mid[i] = cvScalar(0);

		m_imgGray[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 1);
		m_imgEdge[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 1);
		m_imgEdgeColor[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 3);
		m_imgIntensity[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 1);
		m_imgIntensityColor[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 3);
		m_imgResult[i] = cvCreateImage(m_birdeye_size, IPL_DEPTH_8U, 3);
		m_imgOriginResult[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 3);
		m_imgOriginGray[i] = cvCreateImage(m_size, IPL_DEPTH_8U, 1);
	}
	
	m_rect_line[0].x = (int)(m_birdeye_size.width * 0.31);
	m_rect_line[0].y = (int)(m_birdeye_size.height * 0.35);
	m_rect_line[0].width = (int)(m_birdeye_size.width * 0.4);
	m_rect_line[0].height = (int)(m_birdeye_size.height * 0.65);

	m_rect_line[1].x = (int)(m_birdeye_size.width * 0.13);
	m_rect_line[1].y = (int)(m_birdeye_size.height * 0.35);
	m_rect_line[1].width = (int)(m_birdeye_size.width * 0.4);
	m_rect_line[1].height = (int)(m_birdeye_size.height * 0.65);

	m_rect_line[2].x = (int)(m_birdeye_size.width * 0.49);
	m_rect_line[2].y = (int)(m_birdeye_size.height * 0.35);
	m_rect_line[2].width = (int)(m_birdeye_size.width * 0.4);
	m_rect_line[2].height = (int)(m_birdeye_size.height * 0.65);

	m_rect_intensity[0].x = 175;
	m_rect_intensity[0].y = 450;
	m_rect_intensity[0].width = 50;
	m_rect_intensity[0].height = 10;

	m_rect_intensity[1].x = 175;
	m_rect_intensity[1].y = 450;
	m_rect_intensity[1].width = 50;
	m_rect_intensity[1].height = 10;

	m_rect_intensity[2].x = 175;
	m_rect_intensity[2].y = 450;
	m_rect_intensity[2].width = 50;
	m_rect_intensity[2].height = 10;
}

void BirdEyeView::ProcessFrame()
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if (m_img[i]->nChannels == 3)
		{
			cvCopy(m_img[i], m_imgOriginResult[i]);
			cvCvtColor(m_img[i], m_imgOriginGray[i], CV_BGR2GRAY);
		}
		else
		{
			cvCopy(m_img[i], m_imgOriginGray[i]);
			cvCvtColor(m_img[i], m_imgOriginResult[i], CV_GRAY2BGR);
		}

		if (!GetBirdEyeView(i))
			return;

		//EstimateLines(i);
		//EstimateCurves(i);
	}
	
	//GetCurveVehicle();

	//for (int i = 0; i < NUM_VIDEO; i++)
	//{
	//	MarkImage(i);
	//}
}

bool BirdEyeView::GetBirdEyeView(int num)
{
	double x, y, u, v;
	int ix, iy;
	CvPoint pos[4];
	uchar pixel[4];
	double p[3], Pl[3];
	uchar *imagebase;

	if (fabs(m_homography[num][8]) < 1e-6)
		return false;

	cvZero(m_imgGray[num]);

	imagebase = (uchar*)m_imgGray[num]->imageData;

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
					p[k] += m_homography[num][k*3+l] * Pl[l];
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
				if (pos[k].x < 0 || pos[k].x >= m_size.width || pos[k].y < 0 || pos[k].y >= m_size.height)
					pixel[k] = 0;
				else
					pixel[k] = ((uchar*)(m_imgOriginGray[num]->imageData + m_imgOriginGray[num]->widthStep * pos[k].y))[pos[k].x];
			}
			imagebase[j] = (uchar)((1-u)*(1-v)*pixel[0] + u*(1-v)*pixel[1] + (1-u)*v*pixel[2] + u*v*pixel[3]);
		}
		imagebase += m_imgGray[num]->widthStep;
	}
	cvCvtColor(m_imgGray[num], m_imgResult[num], CV_GRAY2BGR);
	
	return true;
}

void BirdEyeView::DoubleThresh(IplImage* src_gray, IplImage* src_edge, IplImage* dst, double thresh)
{
	int row, col;
	unsigned char *ptr_gray, *ptr_edge, *ptr_dst;

	ptr_gray = (unsigned char*)src_gray->imageData;
	ptr_edge = (unsigned char*)src_edge->imageData;
	ptr_dst = (unsigned char*)dst->imageData;

	for (row = 0; row < m_birdeye_size.height; row++)
	{
		for (col = 0; col < m_birdeye_size.width; col++)
		{
			ptr_dst[col] = (ptr_edge[col] == 255 && ptr_gray[col] < thresh) ? 0 : ptr_edge[col];
		}
		ptr_gray += src_gray->widthStep;
		ptr_edge += src_edge->widthStep;
		ptr_dst += dst->widthStep;
	}
}

void BirdEyeView::EstimateLines(int num)
{
	int lane_width;
	
	m_line_left[num] = cvScalar(0);
	m_line_right[num] = cvScalar(0);

	m_vec_line_left[num].clear();
	m_vec_line_right[num].clear();

	GetEdgeMap(num);
	GetIntensityMap(num);
	ProcessEdge(num);
	ProcessIntensity(num);
	GroupLineLeft(num);
	GroupLineRight(num);
	TrackLineLeft(num);
	TrackLineRight(num);
	
	m_mid_pos[num] = 0;
	m_lane_width[num] = 0;
	m_dir[num] = 0;
	lane_width = (int)((m_line_right[num].val[0] - m_line_left[num].val[0]) * m_birdeye_size.height + (m_line_right[num].val[1] - m_line_left[num].val[1]));
	if (m_line_left[num].val[3] >= 0.8 && m_line_right[num].val[3] >= 0.8 && lane_width >= 50 && lane_width <= 100)
	{
		//Calculate and update the parameters necessary, if both left and right line can be detected instead of predicted.
		m_mid_pos[num] = (int)(((m_line_left[num].val[0] + m_line_right[num].val[0])* m_birdeye_size.height + (m_line_left[num].val[1] + m_line_right[num].val[1])) / 2);
		m_lane_width[num] = lane_width;
		m_dir[num] = (m_line_left[num].val[0] + m_line_right[num].val[0]) / 2;
	}
}

void BirdEyeView::GetEdgeMap(int num)
{
	CvScalar avg;

	cvSetImageROI(m_imgGray[num], m_rect_intensity[num]);
	avg = cvAvg(m_imgGray[num]);
	cvResetImageROI(m_imgGray[num]);

	cvCanny(m_imgGray[num], m_imgEdge[num], 60, 150);
	DoubleThresh(m_imgGray[num], m_imgEdge[num], m_imgEdge[num], avg.val[0] + 20);
	cvCvtColor(m_imgEdge[num], m_imgEdgeColor[num], CV_GRAY2BGR);
}

void BirdEyeView::ProcessEdge(int num)
{
	double a, b, x;
	int minlength =  m_birdeye_size.height / 20;
	int maxgap = minlength / 2;
	CvPoint *pt;
		
	cvSetImageROI(m_imgEdge[num], m_rect_line[num]);
	cvRectangle(m_imgEdgeColor[num], cvPoint(m_rect_line[num].x, m_rect_line[num].y),
		cvPoint(m_rect_line[num].x + m_rect_line[num].width, m_rect_line[num].y + m_rect_line[num].height), CV_RGB(0, 255, 0), 1);
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq* lines = 0;

	lines = cvHoughLines2(m_imgEdge[num], storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 30, minlength, maxgap);
	cvResetImageROI(m_imgEdge[num]);

	for(int i = 0; i < lines->total; i++)
	{
		pt = (CvPoint*)cvGetSeqElem(lines, i);
		pt[0].x += m_rect_line[num].x;
		pt[0].y += m_rect_line[num].y;
		pt[1].x += m_rect_line[num].x;
		pt[1].y += m_rect_line[num].y;

		if (pt[0].y == pt[1].y)
			continue;

		a = (double)(pt[0].x - pt[1].x) / (pt[0].y - pt[1].y);
		b = (double)(pt[0].x * pt[1].y - pt[1].x * pt[0].y) / (pt[1].y - pt[0].y);
		x = a *  m_birdeye_size.height + b;

		if (x < m_rect_line[num].x + m_rect_line[num].width / 2 && fabs(a) < m_limit_a)
		{
			cvLine(m_imgEdgeColor[num], pt[0], pt[1], CV_RGB(128, 128, 0), 2);
			m_vec_line_left[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
		}
		if (x >= m_rect_line[num].x + m_rect_line[num].width / 2 && fabs(a) < m_limit_a)
		{
			cvLine(m_imgEdgeColor[num], pt[0], pt[1], CV_RGB(128, 0, 128), 2);
			m_vec_line_right[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
		}
	}
	cvReleaseMemStorage(&storage);
}

void BirdEyeView::GetIntensityMap(int num)
{
	unsigned char *lanebase_gray, *lanebase_intensity;
	double sum;
	int nWidth = 4, col, row;
	bool* bNS = new bool[m_birdeye_size.width];
	
	lanebase_gray = (unsigned char*)m_imgGray[num]->imageData;
	lanebase_intensity = (unsigned char*)m_imgIntensity[num]->imageData;
	cvZero(m_imgIntensity[num]);

	for (row = 0; row < m_birdeye_size.height; row++)
	{
		for (col = nWidth; col <= m_birdeye_size.width - nWidth; col++)
		{
			sum = 0;
			for (int i = -nWidth; i < -nWidth / 2; i++)
			{
				sum -= lanebase_gray[col + i];
			}
			for (int i = -nWidth / 2; i < nWidth / 2; i++)
			{
				sum += lanebase_gray[col + i];
			}
			for (int i = nWidth / 2; i < nWidth; i++)
			{
				sum -= lanebase_gray[col + i];
			}
			sum /= nWidth;
			if (sum < 0) sum = 0;
			lanebase_intensity[col] = (unsigned char)sum;
		}

		for (col = 0; col < m_birdeye_size.width; col++)
		{
			bNS[col] = false;
		}

		for (col = nWidth; col <= m_birdeye_size.width - nWidth; col++)
		{
			if (lanebase_intensity[col] > lanebase_intensity[col - 1]
			&& lanebase_intensity[col] > lanebase_intensity[col + 1]
			&& lanebase_intensity[col] > 15)
			{
				bNS[col] = true;
			}
			else
			{
				bNS[col] = false;
			}
		}

		for (col = 0; col < m_birdeye_size.width; col++)
		{
			if (bNS[col])
			{
				lanebase_intensity[col] = 255;
			}
			else
			{
				lanebase_intensity[col] = 0;
			}
		}
		lanebase_gray += m_imgGray[num]->widthStep;
		lanebase_intensity += m_imgIntensity[num]->widthStep;
	}
	cvCvtColor(m_imgIntensity[num], m_imgIntensityColor[num], CV_GRAY2BGR);

	delete[] bNS;
}

void BirdEyeView::ProcessIntensity(int num)
{
	int minlength = m_birdeye_size.height / 20;
	int maxgap = minlength / 2;
	CvPoint *pt;
	double a, b, x;
	CvMemStorage* storage = cvCreateMemStorage(0);
	CvSeq *lines = 0;

	cvSetImageROI(m_imgIntensity[num], m_rect_line[num]);
	cvRectangle(m_imgIntensityColor[num], cvPoint(m_rect_line[num].x, m_rect_line[num].y),
		cvPoint(m_rect_line[num].x + m_rect_line[num].width, m_rect_line[num].y + m_rect_line[num].height), CV_RGB(0, 255, 0), 1);

	lines = cvHoughLines2(m_imgIntensity[num], storage, CV_HOUGH_PROBABILISTIC, 1, CV_PI/180, 30, minlength, maxgap);
	cvResetImageROI(m_imgIntensity[num]);

	for (int i = 0; i < lines->total; i++)
	{
		pt = (CvPoint*)cvGetSeqElem(lines, i);
		pt[0].x += m_rect_line[num].x;
		pt[0].y += m_rect_line[num].y;
		pt[1].x += m_rect_line[num].x;
		pt[1].y += m_rect_line[num].y;

		if (pt[1].y == pt[0].y)
			continue;

		a = (double)(pt[0].x - pt[1].x) / (pt[0].y - pt[1].y);
		b = (double)(pt[0].x * pt[1].y - pt[1].x * pt[0].y) / (pt[1].y - pt[0].y);
		x = a * m_birdeye_size.height + b;

		if (x < m_rect_line[num].x + m_rect_line[num].width / 2 && fabs(a) < m_limit_a)
		{
			cvLine(m_imgIntensityColor[num], pt[0], pt[1], CV_RGB(128, 128, 0), 2);
			m_vec_line_left[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
		}
		if (x >= m_rect_line[num].x + m_rect_line[num].width / 2 && fabs(a) < m_limit_a)
		{
			cvLine(m_imgIntensityColor[num], pt[0], pt[1], CV_RGB(128, 0, 128), 2);
			m_vec_line_right[num].push_back(cvScalar(a, b, abs(pt[0].y - pt[1].y), 1));
		}
	}
	cvReleaseMemStorage(&storage);
}

void BirdEyeView::GroupLineLeft(int num)
{
	m_line_left[num] = cvScalar(0);

	double a, b, x_lane = 0;
	double nHeight = m_rect_line[num].y;
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

		int class_count = cvSeqPartition(seq_line, 0, &labels, bev_is_equal, &nHeight);

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
					if (i == 0 || abs(a - a0) * 10 + abs(a * nHeight + b - a0 * nHeight - b0) < x_lane)
					{
						x_lane = abs(a - a0) * 10 + abs(a * nHeight + b - a0 * nHeight - b0);
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

void BirdEyeView::GroupLineRight(int num)
{
	m_line_right[num] = cvScalar(0);

	double a, b, x_lane = 0;
	double nHeight = m_rect_line[num].y;
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

		int class_count = cvSeqPartition(seq_line, 0, &labels, bev_is_equal, &nHeight);

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
					if (i == 0 || abs(a - a0) * 10 + abs(a * nHeight + b - a0 * nHeight - b0) < x_lane)
					{
						x_lane = abs(a - a0) * 10 + abs(a * nHeight + b - a0 * nHeight - b0);
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

void BirdEyeView::TrackLineLeft(int num)
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
			fabs(m_line_left[num].val[0] - line[num].val[0]) >= BEV_TRACKER_LINE_PARAM_A ||
			fabs(m_line_left[num].val[1] - line[num].val[1]) >= BEV_TRACKER_LINE_PARAM_B)
		{
			//no lines in the tracking area
			counter_notrack[num]++;
			line[num].val[3] = 0.5;
			if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM1)
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
			if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
			{
				counter_notrack[num] = 0;
				line[num] = cvScalar(0);
				changed_line[num] = cvScalar(0);
			}
		}
		if (changed_line[num].val[3] > 0.5 && m_line_left[num].val[3] > 0.5)
		{
			if(fabs(m_line_left[num].val[0] - changed_line[num].val[0]) >= BEV_TRACKER_LINE_PARAM_A ||
				fabs(m_line_left[num].val[1] - changed_line[num].val[1]) >= BEV_TRACKER_LINE_PARAM_B)
			{
				//the current line is far from the previous line
				changed_line[num] = m_line_left[num];
				counter_track[num] = 0;
				counter_notrack[num]++;
				if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
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
				if (counter_track[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
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

void BirdEyeView::TrackLineRight(int num)
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
			fabs(m_line_right[num].val[0] - line[num].val[0]) >= BEV_TRACKER_LINE_PARAM_A ||
			fabs(m_line_right[num].val[1] - line[num].val[1]) >= BEV_TRACKER_LINE_PARAM_B)
		{
			//no lines in the tracking area
			counter_notrack[num]++;
			line[num].val[3] = 0.5;
			if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM1)
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
			if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
			{
				counter_notrack[num] = 0;
				line[num] = cvScalar(0);
				changed_line[num] = cvScalar(0);
			}
		}
		if (changed_line[num].val[3] > 0.5 && m_line_right[num].val[3] > 0.5)
		{
			if(fabs(m_line_right[num].val[0] - changed_line[num].val[0]) >= BEV_TRACKER_LINE_PARAM_A ||
				fabs(m_line_right[num].val[1] - changed_line[num].val[1]) >= BEV_TRACKER_LINE_PARAM_B)
			{
				//the current line is far from the previous line
				changed_line[num] = m_line_right[num];
				counter_track[num] = 0;
				counter_notrack[num]++;
				if (counter_notrack[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
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
				if (counter_track[num] == BEV_TRACKER_TIMES_TO_CONFIRM2)
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

CvScalar BirdEyeView::FitCurve(const vector<CvPoint2D64f>& list)
{
	CvMat *matA = NULL, *matB = NULL, *matX = NULL, *matX_opt = NULL;
	CvMat *matA_full = NULL, *matB_full = NULL, *matX_full = NULL, *matDiff_full = NULL;
	CvMat *matA_filtered = NULL, *matB_filtered = NULL;

	const int RANSAC_LOOP = 100;
	const double RANSAC_TOL = 0.2;
	const double RANSAC_TOL_STRICT = 0.1;
	const double RANSAC_LEAST_SIZE = 0.5;
	const double scale = 100.0;

	int s[3], counter = 0, max_counter = 0;
	double x, y, a, b, c, belief;
	int y_min, y_max, y_cur;
	int size = (int)list.size();
	vector<int> filtered_index;
	CvScalar param = cvScalar(0);

	if (size < 10)
		return cvScalar(0);

	matA = cvCreateMat(3, 3, CV_32FC1);
	matB = cvCreateMat(3, 1, CV_32FC1);
	matX = cvCreateMat(3, 1, CV_32FC1);
	matX_opt = cvCreateMat(3, 1, CV_32FC1);
	matA_full = cvCreateMat(size, 3, CV_32FC1);
	matB_full = cvCreateMat(size, 1, CV_32FC1);
	matX_full = cvCreateMat(size, 1, CV_32FC1);
	matDiff_full = cvCreateMat(size, 1, CV_32FC1);

	for (int i = 0; i < size; i++)
	{
		x = list[i].x / scale;
		y = list[i].y / scale;
		cvmSet(matA_full, i, 0, y * y);
		cvmSet(matA_full, i, 1, y);
		cvmSet(matA_full, i, 2, 1);
		cvmSet(matB_full, i, 0, x);
	}

	for (int cycle_counter = 0; cycle_counter < RANSAC_LOOP; cycle_counter++)
	{
		s[0] = (int)((double)rand() / RAND_MAX * (size - 1));
		do
		{
			s[1] = (int)((double)rand() / RAND_MAX * (size - 1));
		}
		while (s[0] == s[1]);
		do
		{
			s[2] = (int)((double)rand() / RAND_MAX * (size - 1));
		}
		while(s[2] == s[0] || s[2] == s[1]);

		for (int i = 0; i < 3; i++)
		{
			x = list[s[i]].x / scale;
			y = list[s[i]].y / scale;
			cvmSet(matA, i, 0, y * y);
			cvmSet(matA, i, 1, y);
			cvmSet(matA, i, 2, 1);
			cvmSet(matB, i, 0, x);
		}

		cvSolve(matA, matB, matX);
		cvGEMM(matA_full, matX, 1, matB_full, -1, matDiff_full);

		counter = 0;
		for(int i = 0; i < size; i++)
		{
			if (fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL)
				counter++;
		}

		if(counter > max_counter)
		{
			cvCopy(matX, matX_opt);
			max_counter = counter;
			filtered_index.clear();
			for(int i = 0; i < size; i++)
			{
				if(fabs(cvmGet(matDiff_full, i, 0)) < RANSAC_TOL_STRICT)
					filtered_index.push_back(i);
			}
		}
	}

	if ((int)filtered_index.size() >= 3 && (int)filtered_index.size() > (int)(RANSAC_LEAST_SIZE * max_counter))
	{
		matA_filtered = cvCreateMat((int)filtered_index.size(), 3, CV_32FC1);
		matB_filtered = cvCreateMat((int)filtered_index.size(), 1, CV_32FC1);

		y_min = y_max = list[filtered_index[0]].y;
		for(int i = 0; i < (int)filtered_index.size(); i++)
		{
			y_cur = list[filtered_index[i]].y;
			if (y_cur > y_max)
				y_max = y_cur;
			if (y_cur < y_min)
				y_min = y_cur;

			cvmSet(matA_filtered, i, 0, cvmGet(matA_full, filtered_index[i], 0));
			cvmSet(matA_filtered, i, 1, cvmGet(matA_full, filtered_index[i], 1));
			cvmSet(matA_filtered, i, 2, cvmGet(matA_full, filtered_index[i], 2));
			cvmSet(matB_filtered, i, 0, cvmGet(matB_full, filtered_index[i], 0));
		}
		cvSolve(matA_filtered, matB_filtered, matX_opt, CV_SVD);

		cvReleaseMat(&matA_filtered);
		cvReleaseMat(&matB_filtered);

		a = cvmGet(matX_opt, 0, 0) / scale;
		b = cvmGet(matX_opt, 1, 0);
		c = cvmGet(matX_opt, 2, 0) * scale;
		belief = (double)filtered_index.size() / max_counter;

		if (fabs(a) <= 0.01 && fabs(b) <= 0.1 && (y_max - y_min) >= 200)
			param = cvScalar(a, b, c, belief);
		else
			param = cvScalar(0);
	}
	else
	{
		param = cvScalar(0);
	}

	cvReleaseMat(&matA);
	cvReleaseMat(&matB);
	cvReleaseMat(&matX);
	cvReleaseMat(&matX_opt);
	cvReleaseMat(&matA_full);
	cvReleaseMat(&matB_full);
	cvReleaseMat(&matX_full);
	cvReleaseMat(&matDiff_full);

	return param;
}

void BirdEyeView::EstimateCurves(int num)
{
	int y, j_l, j_r;
	unsigned char* lanebase;
	double x_left, x_right;
	double begin_x, current_lane_width, lane_width_step, lane_width_bottom, lane_width_top;
	bool left_found, right_found;

	m_curve_left[num] = cvScalar(0);
	m_curve_right[num] = cvScalar(0);
	m_curve_mid[num] = cvScalar(0);
	m_list_left[num].clear();
	m_list_right[num].clear();
	m_list_mid[num].clear();

	if (m_line_left[num].val[3] == 0 || m_line_right[num].val[3] == 0
		|| fabs(m_line_left[num].val[0] - m_line_right[num].val[0]) > 0.05
		|| m_lane_width[num] < 60 || m_lane_width[num] > 100)
		return;

	if (m_line_left[num].val[3] >= 0.5)
	{
		for (y = m_rect_line[num].y + m_rect_line[num].height; y > m_rect_line[num].y; y--)
		{
			x_left = m_line_left[num].val[0] * y + m_line_left[num].val[1];
			m_list_left[num].push_back(cvPoint2D64f(x_left, y));
		}
	}
	if (m_line_right[num].val[3] >= 0.5)
	{
		for (y = m_rect_line[num].y + m_rect_line[num].height; y > m_rect_line[num].y; y--)
		{
			x_right = m_line_right[num].val[0] * y + m_line_right[num].val[1];
			m_list_right[num].push_back(cvPoint2D64f(x_right, y));
		}
	}

	y = m_rect_line[num].y + m_rect_line[num].height;
	x_left = m_line_left[num].val[0] * y + m_line_left[num].val[1];
	x_right = m_line_right[num].val[0] * y + m_line_right[num].val[1];
	lane_width_bottom = 0.5 * (x_right - x_left);

	y = m_rect_line[num].y;
	x_left = m_line_left[num].val[0] * y + m_line_left[num].val[1];
	x_right = m_line_right[num].val[0] * y + m_line_right[num].val[1];
	lane_width_top = 0.5 * (x_right - x_left);

	lane_width_step = (lane_width_top - lane_width_bottom) / m_rect_line[num].height;

	lanebase = (unsigned char*)(&m_imgIntensity[num]->imageData[m_rect_line[num].y * m_imgIntensity[num]->widthStep]);
	begin_x = (x_left + x_right) / 2.0;
	current_lane_width = lane_width_top;

	for (y = m_rect_line[num].y; y >= 0; y--)
	{
		left_found = false;
		right_found = false;
		m_list_mid[num].push_back(cvPoint2D64f(begin_x, y));

		if (m_line_left[num].val[3] >= 0.5)
		{
			for (j_l = (int)(begin_x - current_lane_width * BEV_WIDTH_MIN_THRESHOLD); j_l >= (int)(begin_x - current_lane_width * BEV_WIDTH_MAX_THRESHOLD); j_l--)
			{
				if (lanebase[j_l] != 0)
				{
					left_found = true;
					break;
				}
			}
		}

		if (m_line_right[num].val[3] >= 0.5)
		{
			for (j_r = (int)(begin_x + current_lane_width * BEV_WIDTH_MIN_THRESHOLD); j_r <= (int)(begin_x + current_lane_width * BEV_WIDTH_MAX_THRESHOLD); j_r++)
			{
				if (lanebase[j_r] != 0)
				{
					right_found = true;
					break;
				}
			}
		}

		if(left_found)
		{
			if(right_found)
			{
				begin_x = (j_l + j_r) / 2.0;
				m_list_left[num].push_back(cvPoint2D64f(j_l - 2, y));
				m_list_right[num].push_back(cvPoint2D64f(j_r + 2, y));
			}
			else
			{
				begin_x = j_l + current_lane_width - m_dir[0];
				m_list_left[num].push_back(cvPoint2D64f(j_l - 2, y));
			}
		}
		else
		{
			if(right_found)
			{
				begin_x = j_r - current_lane_width - m_dir[0];
				m_list_right[num].push_back(cvPoint2D64f(j_r + 2, y));
			}
			else
			{
				begin_x += m_dir[0];
			}
		}
		lanebase -= m_imgIntensity[num]->widthStep;
		current_lane_width += lane_width_step;
	}

	m_curve_left[num] = FitCurve(m_list_left[num]);
	m_curve_right[num] = FitCurve(m_list_right[num]);

}

void BirdEyeView::MarkImage(int num)
{
	CvPoint p_last, p_current;
	int y, x_l, x_r, x_c;
	CvScalar color;

	cvRectangle(m_imgResult[num], cvPoint(m_rect_line[num].x, m_rect_line[num].y),
		cvPoint(m_rect_line[num].x + m_rect_line[num].width, m_rect_line[num].y + m_rect_line[num].height), CV_RGB(0, 255, 0), 1);
	cvRectangle(m_imgResult[num], cvPoint(m_rect_intensity[num].x, m_rect_intensity[num].y),
		cvPoint(m_rect_intensity[num].x + m_rect_intensity[num].width, m_rect_intensity[num].y + m_rect_intensity[num].height), CV_RGB(0, 255, 0), 1);

	if (m_curve_left[num].val[3] >= 0.4)
	{
		if (m_curve_left[num].val[3] >= 0.5)
			color = CV_RGB(255, 0, 0);
		else
			color = CV_RGB(255, 128, 0);

		y = m_birdeye_size.height - 1;
		x_l = (int)(m_curve_left[num].val[0] * y * y + m_curve_left[num].val[1] * y + m_curve_left[num].val[2]);
		p_last = cvPoint(x_l, y);
		for(y = m_birdeye_size.height - 2; y >= 0; y--)
		{
			x_l = (int)(m_curve_left[num].val[0] * y * y + m_curve_left[num].val[1] * y + m_curve_left[num].val[2]);
			p_current = cvPoint(x_l, y);
			cvLine(m_imgResult[num], p_last, p_current, color, 2, CV_AA);
			p_last = p_current;
		}
	}

	if (m_curve_right[num].val[3] >= 0.4)
	{
		if (m_curve_right[num].val[3] >= 0.5)
			color = CV_RGB(0, 0, 255);
		else
			color = CV_RGB(0, 128, 255);

		y =  m_birdeye_size.height - 1;
		x_r = (int)(m_curve_right[num].val[0] * y * y + m_curve_right[num].val[1] * y + m_curve_right[num].val[2]);
		p_last = cvPoint(x_r, y);
		for(y = m_birdeye_size.height - 2; y >= 0; y--)
		{
			x_r = (int)(m_curve_right[num].val[0] * y * y + m_curve_right[num].val[1] * y + m_curve_right[num].val[2]);
			p_current = cvPoint(x_r, y);
			cvLine(m_imgResult[num], p_last, p_current, color, 2, CV_AA);
			p_last = p_current;
		}
	}
	if (m_curve_mid[num].val[3] >= 0.5)
	{
		y =  m_birdeye_size.height - 1;
		x_c = (int)(m_curve_mid[num].val[0] * y * y + m_curve_mid[num].val[1] * y + m_curve_mid[num].val[2]);
		p_last = cvPoint(x_c, y);
		for(y = m_birdeye_size.height - 2; y >= 0; y--)
		{
			x_c = (int)(m_curve_mid[num].val[0] * y * y + m_curve_mid[num].val[1] * y + m_curve_mid[num].val[2]);
			p_current = cvPoint(x_c, y);
			cvLine(m_imgResult[num], p_last, p_current, CV_RGB(0, 255, 0), 2, CV_AA);
			p_last = p_current;
		}
	}
}

CvScalar BirdEyeView::GetLaneMarkVehicleCoord(const CvScalar& curve)
{
	double x_img, y_img, x_vehicle, y_vehicle;
	CvMat *A, *B, *X;
	double a, b;

	A = cvCreateMat(2, 2, CV_64FC1);
	B = cvCreateMat(2, 1, CV_64FC1);
	X = cvCreateMat(2, 1, CV_64FC1);

	y_img = m_birdeye_size.height;
	x_img = curve.val[0] * y_img + curve.val[1];
	x_vehicle = x_img / m_x_scale + m_x_min;
	y_vehicle = (m_birdeye_size.height - y_img) / m_y_scale + m_y_min;
	cvmSet(A, 0, 0, y_vehicle);
	cvmSet(A, 0, 1, 1);
	cvmSet(B, 0, 0, x_vehicle);

	y_img = 0;
	x_img = curve.val[0] * y_img + curve.val[1];
	x_vehicle = x_img / m_x_scale + m_x_min;
	y_vehicle = (m_birdeye_size.height - y_img) / m_y_scale + m_y_min;
	cvmSet(A, 1, 0, y_vehicle);
	cvmSet(A, 1, 1, 1);
	cvmSet(B, 1, 0, x_vehicle);

	cvSolve(A, B, X, CV_LU);

	a = cvmGet(X, 0, 0);
	b = cvmGet(X, 1, 0);
	
	cvReleaseMat(&A);
	cvReleaseMat(&B);
	cvReleaseMat(&X);

	return cvScalar(0, a, b, curve.val[3]);
}

void BirdEyeView::GetCurveVehicle()
{
	m_vehicle_left = GetLaneMarkVehicleCoord(m_line_left[0]);
	m_vehicle_right = GetLaneMarkVehicleCoord(m_line_right[0]);

	if (m_vehicle_left.val[3] >= 0.8 && m_vehicle_right.val[3] >= 0.8)
		m_lanewidth_vehicle = m_vehicle_right.val[2] - m_vehicle_left.val[2];

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if (m_curve_left[i].val[3] >= 0.8 && m_curve_right[i].val[3] == 0 && m_lane_width[i] >= 60 && m_lane_width[i] <= 100)
		{
			m_curve_right[i].val[0] = m_curve_left[i].val[0];
			m_curve_right[i].val[1] = m_curve_left[i].val[1];
			m_curve_right[i].val[2] = m_curve_left[i].val[2] + m_lane_width[i];
			m_curve_right[i].val[3] = 0.4;
		}
		if (m_curve_right[i].val[3] >= 0.8 && m_curve_left[i].val[3] == 0 && m_lane_width[i] >= 60 && m_lane_width[i] <= 100)
		{
			m_curve_left[i].val[0] = m_curve_right[i].val[0];
			m_curve_left[i].val[1] = m_curve_right[i].val[1];
			m_curve_left[i].val[2] = m_curve_right[i].val[2] - m_lane_width[i];
			m_curve_left[i].val[3] = 0.4;
		}
		for (int j = 0; j < 4; j++)
			m_curve_mid[i].val[j] = (m_curve_left[i].val[j] + m_curve_right[i].val[j]) / 2;
	}

	if (m_vehicle_left.val[3] > 0.8 && m_vehicle_right.val[3] == 0 && m_lanewidth_vehicle > 3 && m_lanewidth_vehicle < 4.5)
	{
		m_vehicle_right.val[0] = m_vehicle_left.val[0];
		m_vehicle_right.val[1] = m_vehicle_left.val[1];
		m_vehicle_right.val[2] = m_vehicle_left.val[2] + m_lanewidth_vehicle;
		m_vehicle_right.val[3] = 0.4;
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
		m_vehicle_mid.val[i] = (m_vehicle_left.val[i] + m_vehicle_right.val[i]) / 2;
	}
}
