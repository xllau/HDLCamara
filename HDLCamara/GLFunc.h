#pragma once

#include "CurbScanner.h"


typedef struct MyColor
{
	unsigned int r;
	unsigned int g;
	unsigned int b;
}MyColor_t;

typedef struct Param_thread
{
	bool bPause;
	bool bForward;
}Param_thread_t;

class GLFunc
{
public:
	static MyColor_t m_lasercolor[HDL_LASER_NUMBER];
	//static LPoint_t m_cloud[HDL_MAX_POINT_NUMBER];
	static Point_laser_t m_point[HDL_LASER_NUMBER];
	static int m_cloud_count;
	
	static int m_window_width;
	static int m_window_height;
	static Param_thread_t *m_param_thread;
	
private:
	//opengl
	static GLint plain_elevation;
	static GLint axis_elevation;

	//initial view point
	static float m_zoom;  //700
	static float m_rotx;  //35
	static float m_roty; //0.001
	static float m_tx;  //0
	static float m_ty; //-30000
	static int m_lastx;
	static int m_lasty;
	static unsigned char m_gl_buttons[3];
	static bool m_bIntensity;
	
public:	
	static void gl_draw_text(char* text, short x, short y);
	static void gl_init_graphics();
	static void gl_resize_graphics(int width, int height);
	static void gl_draw_graphics();
	static void gl_mouse(int b, int s, int x, int y);
	static void gl_motion(int x,int y);
	static void gl_keyboard(unsigned char key, int x, int y);

private:
	static void DrawPoints();
	static void DrawGuidePoint();
	static void DrawText();
	static void SaveData();
};

