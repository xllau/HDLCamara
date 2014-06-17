// stdafx.h : 标准系统包含文件的包含文件，
// 或是经常使用但不常更改的
// 特定于项目的包含文件
//

#pragma once

#define _USE_MATH_DEFINES
#include "targetver.h"


#include <stdio.h>
#include "windows.h"
#include <tchar.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <vector>
#include <string>
#include <math.h>
#include <opencv/cv.h>
#include <opencv/cxcore.h>
#include <opencv/highgui.h>
#include <glut.h>
#include <pthread.h>


using namespace std;
using namespace cv;

extern pthread_t g_thread_opengl;
extern pthread_rwlock_t g_lock_opengl;

// TODO: 在此处引用程序需要的其他头文件

