// LaneMarkBirdView.cpp : 定义控制台应用程序的入口点。
//

#include "stdafx.h"
#include "RecognitionEngine.h"

CRecognitionEngine *m_engine = NULL;

int main(int argc, char** argv)
{
	glutInit(&argc, argv);
	
	//CRecognitionEngine *m_engine = new CRecognitionEngine();
	m_engine = new CRecognitionEngine();
	
	m_engine->Initialize(TYPE_BIRDEYE, IPM_HOMO);
	if (!m_engine->OpenVideoFile("F:/项目相关/Video/20110921_161909"))
		return -1;
	
	if (!m_engine->OpenHDLFile("F:/项目相关/Video/20110921_161911"))
		return -1;

	if(!m_engine->OpenHomographyFile("F:/项目相关/Video/Homography_20110916"))
		return -1;
	
	m_engine->RunVideoHDL();
	
	delete m_engine;

	return 0;
}

