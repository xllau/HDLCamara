#include "stdafx.h"
#include "RecognitionEngine.h"

pthread_t g_thread_opengl;
pthread_rwlock_t g_lock_opengl;


void* opengl_process(void *param)
{
	GLFunc::m_param_thread = (Param_thread_t*)param;
	glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGBA|GLUT_DEPTH);
	glutInitWindowSize(GLFunc::m_window_width, GLFunc::m_window_height);
	glutInitWindowPosition(250,10);
	glutCreateWindow("3D lidar obstacle detection");

	glutDisplayFunc(GLFunc::gl_draw_graphics);
	glutReshapeFunc(GLFunc::gl_resize_graphics);
	glutMouseFunc(GLFunc::gl_mouse);
	glutMotionFunc(GLFunc::gl_motion);
	glutKeyboardFunc(GLFunc::gl_keyboard);
	GLFunc::gl_init_graphics();
	glutMainLoop();

	return NULL;
}

CRecognitionEngine::CRecognitionEngine()
{
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		m_videoreader[i] = NULL;
		m_videowriter[i] = NULL;
		m_homography[i] = NULL;
		m_homography_inv[i] = NULL;
	}

	m_hdl = NULL;
	m_curb = NULL;

	pthread_rwlock_init(&g_lock_opengl, NULL);
}

CRecognitionEngine::~CRecognitionEngine()
{
	pthread_cancel(g_thread_opengl);
	pthread_join(g_thread_opengl, NULL);
	pthread_rwlock_destroy(&g_lock_opengl);

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if(m_videoreader[i])
		{
			cvReleaseCapture(&m_videoreader[i]);
			m_videoreader[i] = NULL;
		}
		if(m_videowriter[i])
		{
			cvReleaseVideoWriter(&m_videowriter[i]);
			m_videowriter[i] = NULL;
		}
		if (m_homography[i])
		{
			cvReleaseMat(&m_homography[i]);
			m_homography[i] = NULL;
		}
		if (m_homography_inv[i])
		{
			cvReleaseMat(&m_homography_inv[i]);
			m_homography_inv[i] = NULL;
		}
	}

	if (m_timestampfile_video)
		m_timestampfile_video.close();
	
	if (m_timestampfile_hdl)
		m_timestampfile_hdl.close();

	if (m_output)
		m_output.close();

	
	if (m_hdl)
	{
		delete m_hdl;
		m_hdl = NULL;
	}
	if (m_curb)
	{
		delete m_curb;
		m_curb = NULL;
	}
}

void CRecognitionEngine::Initialize(int nType, int nIPM)
{
	m_nType = nType;
	m_nIPM = nIPM;

	strcpy(m_device_name[0], "54gm_center");
	strcpy(m_device_name[1], "54gm_left");
	strcpy(m_device_name[2], "54gm_right");
}

bool CRecognitionEngine::OpenHomographyFile(const char* filename)
{
	char buffer_name[80];
	ifstream input;
	
	for (int i = 0; i < NUM_VIDEO; i++)
	{
		if (m_homography[i])
			cvReleaseMat(&m_homography[i]);
		m_homography[i] = cvCreateMat(3, 3, CV_64FC1);
		if (m_homography_inv[i])
			cvReleaseMat(&m_homography_inv[i]);
		m_homography_inv[i] = cvCreateMat(3, 3, CV_64FC1);

		sprintf(buffer_name, "%s_%s.txt", filename, m_device_name[i]);
		input.open(buffer_name);
		if (!input)
		{
			cout << "Cannot open Homography file " << buffer_name << endl;
			return false;
		}
		for (int j = 0; j < 9; j++)
		{
			input >> m_homography[i]->data.db[j];
			//cout<<m_homography[i]->data.db[j]<<endl;
		}
		for (int j = 0; j < 9; j++)
		{
			input >> m_homography_inv[i]->data.db[j];
		}
		input.close();
	}
	
	

	return true;
}

bool CRecognitionEngine::OpenARTFile(const char* filename)
{
	char buffer_name[80];
	ifstream input;

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		sprintf(buffer_name, "%s_%s.txt", filename, m_device_name[i]);
		input.open(buffer_name);
		if (!input)
		{
			cout << "Cannot open ART file " << buffer_name << endl;
			return false;
		}
		for (int j = 0; j < 9; j++)
		{
			input >> m_intrinsicA[i][j];
		}
		for (int j = 0; j < 9; j++)
		{
			input >> m_laser64Rc[i][j];
		}
		for (int j = 0; j < 3; j++)
		{
			input >> m_laser64Tc[i][j];
		}
		input.close();
	}

	return true;
}


bool CRecognitionEngine::OpenVideoFile(const char* filename)
{
	char videofilename[100];
	char videotimestamp[100];

	for (int i = 0; i < NUM_VIDEO; i++)
	{
		sprintf(videofilename, "%s_%s.avi", filename, m_device_name[i]);

		m_videoreader[i] = cvCreateFileCapture(videofilename);
		if(m_videoreader[i] == NULL)
		{
			cout << "Cannot open video file " << videofilename << endl;
			return false;
		}
	}

	sprintf(videotimestamp, "%s.avi_stamp", filename);
	m_timestampfile_video.open(videotimestamp);
	if (!m_timestampfile_video)
	{
		cout << "Cannot open video timestamp file " << videotimestamp << endl;
		return false;
	}
	
	m_size.width = (int)(cvGetCaptureProperty(m_videoreader[0], CV_CAP_PROP_FRAME_WIDTH));
	m_size.height = (int)(cvGetCaptureProperty(m_videoreader[0], CV_CAP_PROP_FRAME_HEIGHT));
	m_birdeye_size = cvSize(300, 600);
	m_x_min = -10;		//meter
	m_x_max = 10;		//meter
	m_y_min = 5;		//meter
	m_y_max = 45;		//meter
	m_x_scale = m_birdeye_size.width / (m_x_max - m_x_min);
	m_y_scale = m_birdeye_size.height / (m_y_max - m_y_min);


	return true;
}

bool CRecognitionEngine::OpenHDLFile(const char* filename)
{
	char hdl_filename[100];
	char hdl_timestamp_filename[100];

	sprintf(hdl_filename, "%s.hdl", filename);
	sprintf(hdl_timestamp_filename, "%s.hdl_stamp", filename);
		
	m_timestampfile_hdl.open(hdl_timestamp_filename);
	if (!m_timestampfile_hdl)
	{
		cout << "read timestamp file " << hdl_timestamp_filename << " error!" << endl;
		return false;
	}

	m_hdl = new HDLDisplay();
	m_curb = new CurbScanner();
	if (!m_hdl->Initialize(hdl_filename))
		return false;
	m_curb->Initialize();

	memcpy(GLFunc::m_lasercolor, m_hdl->m_lasercolor, sizeof(MyColor_t) * HDL_LASER_NUMBER);

	return true;
}

void CRecognitionEngine::RunVideoHDL()
{
	IplImage *img, *imgResult;
	char key = 0;
	bool bRecord = false;
	char window_result[NUM_VIDEO][100];
	Param_thread_t param_thread = {false, false};
	m_vehicle_speed = 4.0;

	pthread_create(&g_thread_opengl, NULL, opengl_process, &param_thread);
		
	m_timestampfile_video >> m_timestamp_video;
	m_timestampfile_hdl >> m_timestamp_hdl;

	//if the camara comes later than hdl, then run the hdl only
	while (m_timestamp_hdl < m_timestamp_video)
	{
		//Process HDL Data
		if (!m_hdl->ProcessFrame())
			break;
		
		m_preview_dist = m_vehicle_speed;

		m_curb->m_cloud_count = m_hdl->m_save_cloud_count;
		memcpy(m_curb->m_cloud, m_hdl->m_cloud, sizeof(LPoint_t) * m_hdl->m_save_cloud_count);
		m_curb->ProcessFrame(img);
		DrawOpenGL();
		
		if (!m_timestampfile_hdl.eof())
		{
			m_timestampfile_hdl >> m_timestamp_hdl;
		}
		else
			break;
	}
	
	while(1)
	{
		if (!param_thread.bPause || param_thread.bForward)
		{
			m_preview_dist = m_vehicle_speed;

			//get a frame of image
			if ((img = cvQueryFrame(m_videoreader[0])) == NULL)
				break;
					
			//m_bev->m_img[0] = img;


				/*m_bev->ProcessFrame();
				for (int i = 0; i < NUM_VIDEO; i++)
					DrawLidar(i);*/
			
			
			if (!m_timestampfile_video.eof())
			{
				m_timestampfile_video >> m_timestamp_video_next;
			}
			else
				break;

			if (m_timestamp_video <= m_timestamp_hdl && m_timestamp_video_next >= m_timestamp_hdl)
			{
				//Process HDL Data
				if (!m_hdl->ProcessFrame())
					break;

				m_curb->m_cloud_count = m_hdl->m_save_cloud_count;
				memcpy(m_curb->m_cloud, m_hdl->m_cloud, sizeof(LPoint_t) * m_hdl->m_save_cloud_count);
				m_curb->ProcessFrame(img);
				DrawOpenGL();

				//if (m_nIPM == IPM_ART)
				//{
				//	GetHomographyMatrix(false);
				//}

				if (!m_timestampfile_hdl.eof())
				{
					m_timestampfile_hdl >> m_timestamp_hdl;
				}
				else
					break;
			}
			m_timestamp_video = m_timestamp_video_next;

			//for (int i = 0; i < NUM_VIDEO; i++)
			//{
			//	imgResult = (m_nType == TYPE_BIRDEYE) ? m_bev->m_imgResult[i] : m_lms->m_imgResult[i];
			//	//cvShowImage(window_result[i], imgResult);
			//	CvPoint2D32f temp;
			//	for(int i = 0 ; i< m_curb->m_pt_left.size();i++)
			//	{
			//		
			//		temp.x = m_curb->m_pt_left.at(i).x;
			//		temp.y = m_curb->m_pt_left.at(i).y;
			//		temp.x /= 1000;temp.y /= 1000;
			//		PointConvert(&temp);
			//		cvCircle(img,cvPoint(temp.x,temp.y),3,CV_RGB(255, 0, 0), 2);
			//	}
			//	for(int i = 0 ; i< m_curb->m_pt_right.size();i++)
			//	{
			//		temp.x = m_curb->m_pt_right.at(i).x;
			//		temp.y = m_curb->m_pt_right.at(i).y;
			//		temp.x /= 1000;temp.y /= 1000;
			//		PointConvert(&temp);
			//		cvCircle(img,cvPoint(temp.x,temp.y),3,CV_RGB(0, 255, 0), 2);
			//	}

			//	for(int i = 0;i< 64;i++)
			//	{
			//		for(int j = 0 ;j < m_curb->m_point[i].pt_count - 1; j++)
			//		{
			//			if(m_curb->m_point[i].is_road[j] == 1 && m_curb->m_point[i].pt[j].y > 5)
			//			{
			//				temp.x = m_curb->m_point[i].pt[j].x;
			//				temp.y = m_curb->m_point[i].pt[j].y;
			//				temp.x /= 1000;temp.y /= 1000;
			//				PointConvert(&temp);
			//				cvCircle(img,cvPoint(temp.x,temp.y),1,CV_RGB(128, 255, 128), 1);
			//			}
			//		}
			//	}

			//	cvShowImage(window_result[i], img);
			//	if(bRecord)
			//	{
			//		cvWriteFrame(m_videowriter[i], imgResult);
			//	}
			//}
			cvShowImage("Original", img);							
			if (!param_thread.bForward)
				param_thread.bForward = false;
		}
		key = cvWaitKey(5);
		
		switch(key)
		{
		case 27:
			return;
		case 32:
			param_thread.bPause = !param_thread.bPause;
			break;
		case 'h':
		case 'H':
			//GetHomographyMatrix(true);
			break;
		case 's':
		case 'S':
			//SaveImage();
			SaveHDL();
			break;
		case 'f':
		case 'F':
			param_thread.bForward = !param_thread.bForward;
			break;
		case 'r':
		case 'R':
			bRecord = !bRecord;
			if (bRecord)
			{
				if (!InitVideo())
					return;
				cout << "Recording started!" << endl;
			}
			else
			{
				for (int i = 0; i < NUM_VIDEO; i++)
				{
					if (m_videowriter[i])
					{
						cvReleaseVideoWriter(&m_videowriter[i]);
						m_videowriter[i] = NULL;
					}
				}
				cout << "Recording stopped!" << endl;
			}
			break;


		}
	}
}


bool CRecognitionEngine::InitVideo()
{
	char buffer[30];
	time_t now = time(0);
	tm* tnow = localtime(&now);

	if (m_nType == TYPE_BIRDEYE)
	{
		for (int i = 0; i < NUM_VIDEO; i++)
		{
			sprintf(buffer, "%04d%02d%02d_%02d%02d%02d_%s.avi", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
			m_videowriter[i] = cvCreateVideoWriter(buffer, CV_FOURCC('X', 'V', 'I', 'D'), 54, m_birdeye_size);
			if (!m_videowriter[i])
			{
				cout << "Error! Cannot open video file " << buffer << endl;
				return false;
			}
		}
	}
	else
	{
		for (int i = 0; i < NUM_VIDEO; i++)
		{
			sprintf(buffer, "%04d%02d%02d_%02d%02d%02d_%s.avi", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
			m_videowriter[i] = cvCreateVideoWriter(buffer, CV_FOURCC('X', 'V', 'I', 'D'), 54, m_size);
			if (!m_videowriter[i])
			{
				cout << "Error! Cannot open video file " << buffer << endl;
				return false;
			}
		}
	}
	

	return true;
}

//void CRecognitionEngine::SaveImage()
//{
//	char buf[80];
//	time_t now = time(0);
//	tm* tnow = localtime(&now);
//
//	if (m_nType == TYPE_BIRDEYE)
//	{
//		for (int i = 0; i < NUM_VIDEO; i++)
//		{
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_OriginGray.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgOriginGray[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgOriginGray[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_OriginResult.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgOriginResult[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgOriginResult[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Gray.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgGray[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgGray[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Edge.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgEdge[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgEdge[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_EdgeColor.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgEdgeColor[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgEdgeColor[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Intensity.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgIntensity[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgIntensity[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_IntensityColor.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgIntensityColor[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgIntensityColor[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Result.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_bev->m_imgResult[i])
//			{
//				cvSaveImage(buf, m_bev->m_imgResult[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//		}
//	}
//	else
//	{
//		for (int i = 0; i < NUM_VIDEO; i++)
//		{
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgGray[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgGray[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Edge.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgEdge[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgEdge[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_EdgeColor.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgEdgeColor[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgEdgeColor[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Intensity.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgIntensity[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgIntensity[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_IntensityColor.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgIntensityColor[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgIntensityColor[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//			sprintf(buf, "%04d%02d%02d_%02d%02d%02d_%s_Result.jpg", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec, m_device_name[i]);
//			if (m_lms->m_imgResult[i])
//			{
//				cvSaveImage(buf, m_lms->m_imgResult[i]);
//				cout << "Image file " << buf << " successfully saved!" << endl;
//			}
//		}
//	}
//}

void CRecognitionEngine::SaveHDL()
{
	char buf[80];
	time_t now = time(0);
	tm* tnow = localtime(&now);
	ofstream output;
	LPoint_t p;

	sprintf(buf, "%04d%02d%02d_%02d%02d%02d.txt", 1900+tnow->tm_year, tnow->tm_mon+1, tnow->tm_mday, tnow->tm_hour, tnow->tm_min, tnow->tm_sec);
	output.open(buf);
	if (!output)
	{
		cout << "Error! Cannot open HDL file " << buf << " for output!" << endl;
		return;
	}

	output << m_hdl->m_save_cloud_count << endl;
	for (int i = 0; i < m_hdl->m_save_cloud_count; i++)
	{
		p = m_hdl->m_cloud[i];
		output << (float)p.x / 10 << ", " << (float)p.y / 10 << ", " << (float)p.z / 10 << ", " << (int)p.c << endl;
	}
	cout << "HDL file " << buf << " successfully saved!" << endl;
}

void CRecognitionEngine::DrawOpenGL()
{
	pthread_rwlock_wrlock(&g_lock_opengl);

	GLFunc::m_cloud_count = m_curb->m_cloud_count;
	//memcpy(GLFunc::m_cloud, m_curb->m_cloud, sizeof(LPoint_t) * m_curb->m_cloud_count);
	memcpy(GLFunc::m_point, m_curb->m_point, sizeof(Point_laser_t) * HDL_LASER_NUMBER);
	
	pthread_rwlock_unlock(&g_lock_opengl);
}

void CRecognitionEngine::PointConvert(CvPoint2D32f * point)
{
	double pt[3], PT[3];

	pt[0] = (double)point->x;
	pt[1] = (double)point->y;
	pt[2] = 1;
	for (int i = 0; i < 3; i++)
	{
		PT[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			PT[i] += m_homography[0]->data.db[i*3+j] * pt[j];
		}
	}
	PT[0] = PT[0] / PT[2];
	PT[1] = PT[1] / PT[2];
	point->x = PT[0];
	point->y = PT[1];
}


void CRecognitionEngine::PointReverseConvert(CvPoint2D32f * point)
{
	double pt[3], PT[3];

	if (abs(m_homography_inv[1]->data.db[8]) < 1e-6 )
		return ;

	pt[0] = (double)point->x;
	pt[1] = (double)point->y;
	pt[2] = 1;
	for (int i = 0; i < 3; i++)
	{
		PT[i] = 0;
		for (int j = 0; j < 3; j++)
		{
			PT[i] += m_homography_inv[0]->data.db[i*3+j] * pt[j];
		}
	}
	PT[0] = PT[0] / PT[2];
	PT[1] = PT[1] / PT[2];
	point-> x= PT[0];
	point-> y= PT[1];
}
