#include "stdafx.h"
#include "DoubleSidesModelFitting.h"

DoubleSidesModelFitting::DoubleSidesModelFitting()
{
	matA = NULL;
	matB = NULL;
	matX = NULL;
	matOptX = NULL;
}

DoubleSidesModelFitting::~DoubleSidesModelFitting(void)
{
	if (matA)
	{
		cvReleaseMat(&matA);
		matA = NULL;
	}
	if (matB)
	{
		cvReleaseMat(&matB);
		matB = NULL;
	}
	if (matX)
	{
		cvReleaseMat(&matX);
		matX = NULL;
	}
	if (matOptX)
	{
		cvReleaseMat(&matOptX);
		matOptX = NULL;
	}
}

void DoubleSidesModelFitting::Initialization(int type)
{
	if (type == DSMF_TYPE_SINGLE)
	{
		matA = cvCreateMat(3, 3, CV_32FC1);
		matB = cvCreateMat(3, 1, CV_32FC1);
		matX = cvCreateMat(3, 1, CV_32FC1);
		matOptX = cvCreateMat(3, 1, CV_32FC1);
	}
	if (type == DSMF_TYPE_DOUBLE)
	{
		matA = cvCreateMat(4, 4, CV_32FC1);
		matB = cvCreateMat(4, 1, CV_32FC1);
		matX = cvCreateMat(4, 1, CV_32FC1);
		matOptX = cvCreateMat(4, 1, CV_32FC1);
	}
	rng = cvRNG(-1);
}

const CvScalar DoubleSidesModelFitting::GetLeftResult() const
{
	return curve_left;
}

const CvScalar DoubleSidesModelFitting::GetRightResult() const
{
	return curve_right;
}

const CvScalar DoubleSidesModelFitting::GetSingleResult() const
{
	return curve;
}

int DoubleSidesModelFitting::SingleFitList(const vector<CvPoint>& list, int horizon)
{
	double belief;

	if (list.empty())
	{
		return DSMF_FAIL_TOO_SHORT;
	}

	if (list.size() < DSMF_RANSAC_MIN_POINTS)
	{
		return DSMF_FAIL_TOO_SHORT;
	}

	matFullA = cvCreateMat(list.size(), 3, CV_32FC1);
	matFullB = cvCreateMat(list.size(), 1, CV_32FC1);
	matFullDiff = cvCreateMat(list.size(), 1, CV_32FC1);

	for (size_t i = 0; i < list.size(); i++)
	{
		cvmSet(matFullA, i, 0, 1.0 / (list.at(i).y - horizon));
		cvmSet(matFullA, i, 1, (list.at(i).y - horizon));
		cvmSet(matFullA, i, 2, 1.0);
		cvmSet(matFullB, i, 0, list.at(i).x);
	}

	int result = SingleFullFit(list, horizon, belief);

	if(result == DSMF_SUCCESS)
	{
		curve = cvScalar(cvmGet(matOptX, 0, 0), cvmGet(matOptX, 1, 0), cvmGet(matOptX, 2, 0), belief);
	}
	else
	{
		curve = cvScalar(0);
	}

	if (matFullA)
	{
		cvReleaseMat(&matFullA);
		matFullA = NULL;
	}
	if (matFullB)
	{
		cvReleaseMat(&matFullB);
		matFullB = NULL;
	}
	if (matFullDiff)
	{
		cvReleaseMat(&matFullDiff);
		matFullDiff = NULL;
	}

	return result;
}

int DoubleSidesModelFitting::SingleFullFit(const vector<CvPoint>& list, int horizon, double& belief)
{
	int max_counter = 0;

	for(int cycle_counter = 0; cycle_counter < DSMF_RANSAC_MAX_TIMES; ++cycle_counter)
	{
		//int det_times = 0;

		//Randomly find a sample of points that are good for solving the equation.
		int rndind[3];
		rndind[0] = cvFloor(cvRandReal(&rng) * list.size());
		do
		{
			rndind[1] = cvFloor(cvRandReal(&rng) * list.size());
		}
		while(rndind[1] == rndind[0]);
		do
		{
			rndind[2] = cvFloor(cvRandReal(&rng) * list.size());
		}
		while(rndind[2] == rndind[0] || rndind[2] == rndind[1]);

		for(int i = 0; i < 3; i++)
		{
			const CvPoint & p = list.at(rndind[i]);
			cvmSet(matA, i, 0, 1.0 / (p.y - horizon));
			cvmSet(matA, i, 1, p.y - horizon);
			cvmSet(matA, i, 2, 1.0);
			cvmSet(matB, i, 0, p.x);
		}

		//Get a model as candidate.
		if(cvSolve(matA, matB, matX) == 0) continue;

		//Calculate the difference between the model and the actual points.
		cvGEMM(matFullA, matX, 1, matFullB, -1, matFullDiff);

		//Check whether the candidate is a good fit.
		int counter = 0;
		for(int i = 0; i < (int)list.size(); i++)
		{
			if (fabs(cvmGet(matFullDiff, i, 0)) < DSMF_RANSAC_TOL)
			{
				counter++;
			}
		}

		//Check whether the model is the best one ever. If so, remember the points that are close to it.
		if(counter > max_counter)
		{
			cvCopy(matX, matOptX);
			max_counter = counter;
			filtered_index.clear();
			for(int i = 0; i < (int)list.size(); i++)
			{
				if(fabs(cvmGet(matFullDiff, i, 0)) < DSMF_RANSAC_TOL_STRICT)
				{
					filtered_index.push_back(i);
				}
			}
		}
	}

	//Check whether the best found model is a good fit (acceptable).
	if(filtered_index.size() > max(3, DSMF_RANSAC_WEIGHT_ACPT_RATE * max_counter))
	{
		belief = (double)filtered_index.size() / max_counter;
		SingleFilteredFit();
		return DSMF_SUCCESS;
	}
	else
	{
		return DSMF_FAIL_NO_ACCEPTED_RESULT;
	}

	return DSMF_SUCCESS;
}

int DoubleSidesModelFitting::SingleFilteredFit()
{
	//Use the Lease Square Method to fit the model.
	matFilteredA = cvCreateMat(filtered_index.size(), 3, CV_32FC1);
	matFilteredB = cvCreateMat(filtered_index.size(), 1, CV_32FC1);

	for(size_t i = 0; i < filtered_index.size(); i++)
	{
		cvmSet(matFilteredA, i, 0, cvmGet(matFullA, filtered_index.at(i), 0));
		cvmSet(matFilteredA, i, 1, cvmGet(matFullA, filtered_index.at(i), 1));
		cvmSet(matFilteredA, i, 2, cvmGet(matFullA, filtered_index.at(i), 2));
		cvmSet(matFilteredB, i, 0, cvmGet(matFullB, filtered_index.at(i), 0));
	}

	cvSolve(matFilteredA, matFilteredB, matOptX, CV_SVD);

	cvReleaseMat(&matFilteredA);
	cvReleaseMat(&matFilteredB);

	return DSMF_SUCCESS;
}

int DoubleSidesModelFitting::FitList(const vector<CvPoint> & list_left, const vector<CvPoint> & list_right, int horizon)
{
	double belief;

	if(list_left.empty() || list_right.empty())
	{
		return DSMF_FAIL_TOO_SHORT;
	}

	total_length = list_left.size() + list_right.size();
	if(total_length < DSMF_RANSAC_MIN_POINTS)
	{
		return DSMF_FAIL_TOO_SHORT;
	}


	//Assign the points to the model matrices.
	matFullA = cvCreateMat(total_length, 4, CV_32FC1);
	matFullB = cvCreateMat(total_length, 1, CV_32FC1);
	matFullDiff = cvCreateMat(total_length, 1, CV_32FC1);

	for(size_t i = 0; i < list_left.size(); i++)
	{
		cvmSet(matFullA, i, 0, 1.0 / (list_left.at(i).y - horizon));
		cvmSet(matFullA, i, 1, (list_left.at(i).y - horizon));
		cvmSet(matFullA, i, 2, 0.0);
		cvmSet(matFullA, i, 3, 1.0);
		cvmSet(matFullB, i, 0, list_left.at(i).x);
	}
	for(size_t i2 = 0, i = list_left.size() ; i < total_length; i++, i2++)
	{
		cvmSet(matFullA, i, 0, 1.0 / (list_right.at(i2).y - horizon));
		cvmSet(matFullA, i, 1, 0.0);
		cvmSet(matFullA, i, 2, (list_right.at(i2).y - horizon));
		cvmSet(matFullA, i, 3, 1.0);
		cvmSet(matFullB, i, 0, list_right.at(i2).x);
	}

	int result = FullFit(list_left, list_right, horizon, belief);

	if(result == DSMF_SUCCESS)
	{
		curve_left = cvScalar(cvmGet(matOptX, 0, 0), cvmGet(matOptX, 1, 0), cvmGet(matOptX, 3, 0), belief);
		curve_right = cvScalar(cvmGet(matOptX, 0, 0), cvmGet(matOptX, 2, 0), cvmGet(matOptX, 3, 0), belief);
	}
	else
	{
		curve_left = curve_right = cvScalar(0);
	}

	if (matFullA)
	{
		cvReleaseMat(&matFullA);
		matFullA = NULL;
	}
	if (matFullB)
	{
		cvReleaseMat(&matFullB);
		matFullB = NULL;
	}
	if (matFullDiff)
	{
		cvReleaseMat(&matFullDiff);
		matFullDiff = NULL;
	}

	return result;
}

int DoubleSidesModelFitting::FullFit(const vector<CvPoint> & list_left, const vector<CvPoint> & list_right, int horizon, double& belief)
{
	int max_counter = 0;
	//double max_weight = 0;
	for(int cycle_counter = 0; cycle_counter < DSMF_RANSAC_MAX_TIMES; ++cycle_counter)
	{
		int det_times = 0;

		//Randomly find a sample of points that are good for solving the equation.
		do
		{
			int rndind[4];
			rndind[0] = cvFloor(cvRandReal(&rng) * total_length);
			do
			{
				rndind[1] = cvFloor(cvRandReal(&rng) * total_length);
			}
			while(rndind[1] == rndind[0]);
			do
			{
				rndind[2] = cvFloor(cvRandReal(&rng) * total_length);
			}
			while(rndind[2] == rndind[0] || rndind[2] == rndind[1]);
			do
			{
				rndind[3] = cvFloor(cvRandReal(&rng) * total_length);
			}
			while(rndind[3] == rndind[2] || rndind[3] == rndind[1] || rndind[3] == rndind[0]);

			for(int i = 0; i < 4; i++)
			{
				if(rndind[i] < static_cast<int>(list_left.size()))
				{
					const CvPoint & p = list_left.at(rndind[i]);
					cvmSet(matA, i, 0, 1.0 / (p.y - horizon));
					cvmSet(matA, i, 1, p.y - horizon);
					cvmSet(matA, i, 2, 0.0);
					cvmSet(matA, i, 3, 1.0);
					cvmSet(matB, i, 0, p.x);
				}
				else
				{
					const CvPoint & p = list_right.at(rndind[i] - list_left.size());
					cvmSet(matA, i, 0, 1.0 / (p.y - horizon));
					cvmSet(matA, i, 1, 0.0);
					cvmSet(matA, i, 2, p.y - horizon);
					cvmSet(matA, i, 3, 1.0);
					cvmSet(matB, i, 0, p.x);
				}
			}
		}
		while(fabs(cvDet(matA)) < DSMF_RANSAC_DET_THRESHOLD && ++det_times < DSMF_RANSAC_MAX_DET_TIMES);

		if(det_times >= DSMF_RANSAC_MAX_DET_TIMES) continue;


		//Get a model as candidate.
		if(cvSolve(matA, matB, matX) == 0) continue;

		//Calculate the difference between the model and the actual points.
		cvGEMM(matFullA, matX, 1, matFullB, -1, matFullDiff);

		//Check whether the candidate is a good fit.
		int counter = 0;
		for(int i = 0; i < (int)total_length; i++)
		{
			if(fabs(cvmGet(matFullDiff, i, 0)) < DSMF_RANSAC_TOL)
			{
				counter++;
			}
		}

		//Check whether the model is the best one ever. If so, remember the points that are close to it.
		if(counter > max_counter)
		{
			cvCopy(matX, matOptX);
			max_counter = counter;
			filtered_index.clear();
			for(int i = 0; i < (int)total_length; i++)
			{
				if(fabs(cvmGet(matFullDiff, i, 0)) < DSMF_RANSAC_TOL_STRICT)
				{
					filtered_index.push_back(i);
				}
			}
		}
	}

	//Check whether the best found model is a good fit (acceptable).
	if(filtered_index.size() > max(4, DSMF_RANSAC_WEIGHT_ACPT_RATE * max_counter))
	{
		belief = (double)filtered_index.size() / max_counter;
		FilteredFit();
		return DSMF_SUCCESS;
	}
	else
	{
		return DSMF_FAIL_NO_ACCEPTED_RESULT;
	}
	return DSMF_SUCCESS;
}

int DoubleSidesModelFitting::FilteredFit()
{
	//Use the Lease Square Method to fit the model.
	matFilteredA = cvCreateMat(filtered_index.size(), 4, CV_32FC1);
	matFilteredB = cvCreateMat(filtered_index.size(), 1, CV_32FC1);

	for(size_t i = 0; i < filtered_index.size(); i++)
	{
		cvmSet(matFilteredA, i, 0, cvmGet(matFullA, filtered_index.at(i), 0));
		cvmSet(matFilteredA, i, 1, cvmGet(matFullA, filtered_index.at(i), 1));
		cvmSet(matFilteredA, i, 2, cvmGet(matFullA, filtered_index.at(i), 2));
		cvmSet(matFilteredA, i, 3, cvmGet(matFullA, filtered_index.at(i), 3));
		cvmSet(matFilteredB, i, 0, cvmGet(matFullB, filtered_index.at(i), 0));
	}

	cvSolve(matFilteredA, matFilteredB, matOptX, CV_SVD);

	cvReleaseMat(&matFilteredA);
	cvReleaseMat(&matFilteredB);
	return DSMF_SUCCESS;
}
