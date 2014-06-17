#pragma once

#define DSMF_TYPE_DOUBLE 0
#define DSMF_TYPE_SINGLE 1

#define DSMF_RANSAC_TOL 4.0
#define DSMF_RANSAC_ACPT_RATE 0.5
#define DSMF_RANSAC_DET_THRESHOLD 10.0
#define DSMF_RANSAC_MAX_DET_TIMES 5
#define DSMF_RANSAC_MIN_TIMES 100
#define DSMF_RANSAC_MAX_TIMES 400
#define DSMF_RANSAC_WEIGHT_ACPT_RATE 0.5
#define DSMF_RANSAC_MIN_POINTS 60
#define DSMF_RANSAC_TOL_STRICT 2.0

#define DSMF_LIST_MIN_LENGTH 20

#define DSMF_SUCCESS 0
#define DSMF_FAIL_NO_ACCEPTED_RESULT 1
#define DSMF_FAIL_TOO_SHORT 2

#define max(a,b)            (((a) > (b)) ? (a) : (b))
#define min(a,b)            (((a) < (b)) ? (a) : (b))

typedef vector<CvPoint> CvPointList;

/************************************************************************
 Class DoubleSideModelFitting
 The class for fitting the hyper-bola pair model from a pair of lists 
 consisting of points on the lane boundaries.
************************************************************************/

class DoubleSidesModelFitting
{
public:
	DoubleSidesModelFitting(void);
	~DoubleSidesModelFitting(void);
	
	void Initialization(int type);
	//Initialization
	
	int FitList(const vector<CvPoint> & list_left, const vector<CvPoint> & list_right, int horizon);
	//Fit the model from the pair of lists.

	int SingleFitList(const vector<CvPoint>& list, int horizon);
	//Fit the model from a single list.

	const CvScalar GetLeftResult() const;
	//Return the parameter of the fitted left boundary.

	const CvScalar GetRightResult() const;
	//Return the parameter of the fitted right boundary.

	const CvScalar GetSingleResult() const;
	//Return the parameter of the single lane boundary.

protected:
	CvMat * matA;
	CvMat * matB;
	CvMat * matX;
	CvMat * matOptX;
	CvMat * matFullA;
	CvMat * matFullB;
	CvMat * matFullDiff;
	CvMat * matFilteredA;
	CvMat * matFilteredB;

	vector<int> filtered_index;
	CvScalar curve;
	CvScalar curve_left;
	CvScalar curve_right;
	
	size_t total_length;
		
	CvRNG rng;

	int FullFit(const vector<CvPoint> & list_left, const vector<CvPoint> & list_right, int horizon, double& belief);
	//Method to fit the model with all points.

	int SingleFullFit(const vector<CvPoint>& list, int horizon, double& belief);
	//Method to fit the model with all points.

	int FilteredFit();
	//Method to fit the model with selected(filtered) points by RANSAC using LSM.
	
	int SingleFilteredFit();
	//Method to fit the model with selected(filtered) points by RANSAC using LSM.
};
