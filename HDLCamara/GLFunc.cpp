#include "stdafx.h"
#include "GLFunc.h"
#include "RecognitionEngine.h"


extern CRecognitionEngine *m_engine;

//LPoint_t GLFunc::m_cloud[HDL_MAX_POINT_NUMBER];
Point_laser_t GLFunc::m_point[HDL_LASER_NUMBER];
int GLFunc::m_cloud_count = 0;

MyColor_t GLFunc::m_lasercolor[HDL_LASER_NUMBER];
bool GLFunc::m_bIntensity = false;
Param_thread_t *GLFunc::m_param_thread = NULL;

GLint GLFunc::plain_elevation = -2100;
GLint GLFunc::axis_elevation = -2100;
int GLFunc::m_window_width = 1024;
int GLFunc::m_window_height = 735;
float GLFunc::m_zoom = 700.0f;  //700
float GLFunc::m_rotx = 32.0f;  //35
float GLFunc::m_roty = 89.2f; //0.001
float GLFunc::m_tx = 935;  //0
float GLFunc::m_ty = -43515; //-30000
int GLFunc::m_lastx = 0;
int GLFunc::m_lasty = 0;
unsigned char GLFunc::m_gl_buttons[3];

void GLFunc::gl_draw_text(char* text, short x, short y)
{
	glRasterPos2s( x, y );
	for(const char*c = text; *c != '\0'; c++)
	{
		glutBitmapCharacter(GLUT_BITMAP_8_BY_13, *c);
	}
}

void GLFunc::gl_init_graphics()
{
	glClearColor(0, 0, 0, 0.5);
	glClearDepth(1.0);

	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
}

void GLFunc::gl_resize_graphics(int width, int height)
{
	if (width == 0)
		height = 1;
	m_window_width = width;
	m_window_height = height;

	// Adjust graphics to window size
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(45, ((double)width)/height, 0.05, 400000);


	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void GLFunc::DrawPoints()
{
	int c_cloud;
	MyColor mycolor;

	//Draw original points
	//glPointSize(5.0f);
	glBegin(GL_POINTS);
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		for (int j = 0; j < HDL_MAX_POINT_LASER ; j++)//m_point[i].pt_count
		{
			c_cloud = m_point[i].pt[j].c;
			mycolor = m_lasercolor[c_cloud];

			if (!m_bIntensity)
			{
				int t = m_point[i].pt[j].road;
				if (t > 0)
					glColor3ub(200, 100, 100);
				else
					glColor3ub(mycolor.r, mycolor.g, mycolor.b);
			}
			else
				glColor3ub(m_point[i].pt[j].i, m_point[i].pt[j].i, m_point[i].pt[j].i);

			glVertex3i(m_point[i].pt[j].y, m_point[i].pt[j].z, m_point[i].pt[j].x);
		}
	}
	glColor3ub(255, 255, 255);
	
	glVertex3i(m_engine->m_curb->y0,m_engine->m_curb->z0,0);
	glVertex3i(0, axis_elevation, 0);
	
	glEnd();
	//glPointSize(1.0f);
}


void GLFunc::DrawGuidePoint()
{
	double x_max = 0.0, x_preview = 0.0;
	double y_max = 30.0, y_preview = 0.0;
	double a = 0.0, b = 0.0, c = 0.0;


}

void GLFunc::DrawText()
{
	//char text[200];

	/*sprintf(text, "m_lane_left = %f, %f, %f, %f, m_lane_left_type = %d",
		m_lane_left.val[0], m_lane_left.val[1], m_lane_left.val[2], m_lane_left.val[3], m_lane_left_type);
	gl_draw_text(text, 10, 20);

	sprintf(text, "m_lane_right = %f, %f, %f, %f, m_lane_right_type = %d",
		m_lane_right.val[0], m_lane_right.val[1], m_lane_right.val[2], m_lane_right.val[3], m_lane_right_type);
	gl_draw_text(text, 10, 40);

	sprintf(text, "m_lane_mid = %f, %f, %f, %f, m_lane_width = %f",
		m_lane_mid.val[0], m_lane_mid.val[1], m_lane_mid.val[2], m_lane_mid.val[3], m_lane_width);
	gl_draw_text(text, 10, 60);

	sprintf(text, "m_road_left = %f, %f, %f, %f, m_road_left_type = %d", 
		m_road_left.val[0], m_road_left.val[1], m_road_left.val[2], m_road_left.val[3], m_road_left_type);
	gl_draw_text(text, 10, 80);

	sprintf(text, "m_road_right = %f, %f, %f, %f, m_road_right_type = %d", 
		m_road_right.val[0], m_road_right.val[1], m_road_right.val[2], m_road_right.val[3], m_road_right_type);
	gl_draw_text(text, 10, 100);

	sprintf(text, "m_road_mid = %f, %f, %f, %f, m_road_width = %f", 
		m_road_mid.val[0], m_road_mid.val[1], m_road_mid.val[2], m_road_mid.val[3], m_road_width);
	gl_draw_text(text, 10, 120);

	sprintf(text, "m_ratio = %f, m_curvature = %f",
		m_ratio, m_curvature);
	gl_draw_text(text, 10, 140);*/
}

void GLFunc::SaveData()
{
	FILE *fp = fopen("hdl.txt", "wt");
	int intensity;

	for (int i = 5; i < 45; i++)
	{
		for (int j = 0; j < m_point[i].pt_count; j++)
		{
			if (m_point[i].pt[j].rot >= -9000 && m_point[i].pt[j].rot <= 9000)
			{
				intensity = m_point[i].pt[j].i;
				fprintf(fp, "%d ", intensity);
			}
		}
		fprintf(fp, "\n\n");
	}

	fclose(fp);
}

void GLFunc::gl_draw_graphics()
{
	//static int font = (int)GLUT_BITMAP_8_BY_13;  //draw text font
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// Set location in front of camera
	glLoadIdentity();

	glTranslatef(0, -m_zoom, 0);
	glTranslatef(m_tx, 0, m_ty);
	glRotatef(m_rotx, 1, 0, 0);
	glRotatef(m_roty, 0, 1, 0);

	//draw x,z axis
	glBegin(GL_LINES);
	glColor3ub( 88, 29, 29);  //dark red x axis
	glVertex3i(0, axis_elevation, 0);
	glVertex3i(100000, axis_elevation, 0);
	glColor3ub( 29, 88, 29);   //dark green  z axis
	glVertex3i(0, axis_elevation, 0);
	glVertex3i(0, axis_elevation, 100000);
	glEnd();

	pthread_rwlock_rdlock(&g_lock_opengl);
	DrawPoints();
	DrawGuidePoint();
	//DrawText();
	pthread_rwlock_unlock(&g_lock_opengl);

	//show infomation on screen
	glColor3ub(255, 255, 0);    	//font color
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();
	gluOrtho2D( 0, (GLdouble)m_window_width, 0, (GLdouble)m_window_height);
	glScalef(1, -1, 1);
	glTranslatef(0, -m_window_height, 0);
	glMatrixMode(GL_MODELVIEW);
	glPushMatrix();
	glLoadIdentity();
	
	
	glPopMatrix();
	glMatrixMode(GL_PROJECTION);
	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);

	// Show the new scene
	glutSwapBuffers();

	glutPostRedisplay();
	
	Sleep(50);
}

void GLFunc::gl_mouse(int b, int s, int x, int y)
{
	m_lastx = x;
	m_lasty = y;

	switch(b)
	{
	case GLUT_LEFT_BUTTON:
		m_gl_buttons[0] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	case GLUT_MIDDLE_BUTTON:
		m_gl_buttons[1] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	case GLUT_RIGHT_BUTTON:
		m_gl_buttons[2] = ((GLUT_DOWN == s) ? 1 : 0);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

void GLFunc::gl_motion(int x,int y)
{
	int diffx = x - m_lastx;
	int diffy = y - m_lasty;
	m_lastx = x;
	m_lasty = y;

	if( m_gl_buttons[0] && m_gl_buttons[2] ) //transition
	{
		m_zoom += 25.0f * diffy;
	}
	else if( m_gl_buttons[0] ) //rotation
	{
		m_rotx += 0.4f * diffy;
		m_roty += 0.4f * diffx;
	}
	else if( m_gl_buttons[2] ) //zoom
	{
		m_tx += 85.0f * diffx;
		m_ty -= 85.0f * diffy;
	}

	glutPostRedisplay();
}

void GLFunc::gl_keyboard(unsigned char key, int x, int y)
{
	switch(key)
	{
	case 'i':
	case 'I':
		m_bIntensity = !m_bIntensity;
		break;
	case 32:
		m_param_thread->bPause = !m_param_thread->bPause;
		break;
	case 'f':
	case 'F':
		m_param_thread->bForward = !m_param_thread->bForward;
		break;
	case 's':
	case 'S':
		SaveData();
		break;
	case 27:
		exit(0);
	}
}

