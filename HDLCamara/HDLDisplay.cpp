#include "StdAfx.h"
#include "HDLDisplay.h"

ofstream point("point.txt");

HDLDisplay::HDLDisplay(void)
{
	m_hdldata = NULL;
	m_save_cloud = NULL;
	m_cloud = NULL;
	m_cos_raw = NULL;
	m_sin_raw = NULL;
}

HDLDisplay::~HDLDisplay(void)
{
	if (m_hdldata)
	{
		fclose(m_hdldata);
		m_hdldata = NULL;
	}
	if (m_save_cloud)
	{
		delete[] m_save_cloud;
		m_save_cloud = NULL;
	}
	if (m_cloud)
	{
		delete[] m_cloud;
		m_cloud = NULL;
	}
	if (m_cos_raw)
	{
		delete[] m_cos_raw;
		m_cos_raw = NULL;
	}
	if (m_sin_raw)
	{
		delete[] m_sin_raw;
		m_sin_raw = NULL;
	}
}

bool HDLDisplay::Initialize(const char* hdlfilepath)
{
	m_save_cloud = new PointSave_t[HDL_MAX_POINT_NUMBER];
	m_cloud = new LPoint_t[HDL_MAX_POINT_NUMBER];
	m_cos_raw = new float[ANGLE_NUM];
	m_sin_raw = new float[ANGLE_NUM];
	m_save_cloud_count = 0;

	if (!load_laser_color("colormatrix.txt"))
		return false;

	if (!load_laser_info("new_xml.txt"))
		return false;

	if ((m_hdldata = fopen(hdlfilepath, "rb")) == NULL)
	{
		cout << "read hdl file " << hdlfilepath << " error!" << endl;
		return false;
	}

	return true;
}

//get color for each laser
bool HDLDisplay::load_laser_color(const char* colorpath)
{
	int index;
	ifstream fdb(colorpath);
	if (!fdb)
	{
		cout << "read color matrix file error!" << endl;
		return false;
	}

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		fdb >> index >> m_lasercolor[i].r >> m_lasercolor[i].g >> m_lasercolor[i].b;
	}
	fdb.close();

	return true;
}

//read the fixed info for each laser
//pre-processing cos, sin
bool HDLDisplay::load_laser_info(const char* data_path)
{
	//the dat file is converted from db.xml, only have the array info
	//used parse_xml_db solution to convert xml to dat file
	ifstream fdb(data_path);
	if (!fdb)
	{
		cout << "read laser info file error!" << endl;
		return false;
	}

	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		fdb >> m_rot[i] >> m_vert[i] >> m_dist[i] >> m_z_off[i] >> m_x_off[i] >> m_min_i[i] >> m_max_i[i] >> m_distX[i] >> m_distY[i] >> m_f_d[i] >> m_f_s[i];
	}
	fdb.close();

	//regulate unit to mm
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_dist[i]  *= 10;
		m_z_off[i] *= 10;
		m_x_off[i] *= 10;
		m_distX[i] *= 10;
		m_distY[i] *= 10;
	}

	//pre-processing sin, cos array
	for (int i = 0; i < HDL_LASER_NUMBER; i++)
	{
		m_cos_rot[i] = cos( m_rot[i] / 180.0f * M_PI);
		m_sin_rot[i] = sin( m_rot[i] / 180.0f * M_PI);
		m_cos_vert[i] = cos( m_vert[i] / 180.0f * M_PI);
		m_sin_vert[i] = sin( m_vert[i] / 180.0f * M_PI);
	}

	for (int i = 0; i < ANGLE_NUM; i++)
	{
		m_cos_raw[i] = cos( i / 18000.0f * M_PI);
		m_sin_raw[i] = sin( i / 18000.0f * M_PI);
	}

	return true;
}

bool HDLDisplay::ProcessFrame()
{
	unsigned short rot;
	unsigned char c;

	if ((int)fread(&m_save_cloud_count, sizeof(int), 1, m_hdldata) < 1)
		return false;
	if ((int)fread(m_save_cloud, sizeof(PointSave_t), m_save_cloud_count, m_hdldata) < m_save_cloud_count)
		return false;

	for (int i = 0; i < m_save_cloud_count; i++)
	{
		m_cloud[i].i = m_save_cloud[i].i;
		m_cloud[i].dist = m_save_cloud[i].dist;
		c = m_cloud[i].c = m_save_cloud[i].c;
		rot = m_save_cloud[i].rot;
		int rot1 = (36000 + m_save_cloud[i].rot - 9*((int)(m_rot[c]*100/9)))%36000;
		m_cloud[i].rot = rot1;
		/*cout<<m_save_cloud[i].i<<endl;
		cout<<m_save_cloud[i].dist<<endl;
		cout<<m_save_cloud[i].c<<endl;          
		cout<<m_save_cloud[i].rot<<endl;    */ 

		float cos_phi = m_cos_vert[c];
		float sin_phi = m_sin_vert[c];
		float cos_theta = m_cos_raw[rot] * m_cos_rot[c] + m_sin_raw[rot] * m_sin_rot[c];
		float sin_theta = m_sin_raw[rot] * m_cos_rot[c] - m_cos_raw[rot] * m_sin_rot[c];
		float r1 = m_save_cloud[i].dist * 2.0f;
		float r = r1 + m_dist[c];

		float rxy = r * cos_phi;
		float xx = abs(rxy * sin_theta - m_x_off[c] * cos_theta);
		float yy = abs(rxy * cos_theta + m_x_off[c] * sin_theta);

		float rx = (m_dist[c] - m_distX[c]) * (xx/22640.0f - 0.106007f) + m_distX[c];
		float ry = (m_dist[c] - m_distY[c]) * (yy/23110.0f - 0.083514f) + m_distY[c];

		//x:
		r = r1 + rx;
		rxy = r * cos_phi;
		int x = (int)(rxy * sin_theta - m_x_off[c] * cos_theta);

		//y:
		r = r1 + ry;
		rxy = r * cos_phi;
		int y = (int)(rxy * cos_theta + m_x_off[c] * sin_theta);

		//z:
		r = r1 + m_dist[c];
		int z = (int)(r * sin_phi + m_z_off[c]);

		m_cloud[i].x = x;
		m_cloud[i].y = y;
		m_cloud[i].z = z;
		//if(i == 10)
			//point<<x<<" "<<y<<" "<<z<<" "<<endl;
	}
	//cout<<"kfdjsaklf"<<endl;
	return true;
}
