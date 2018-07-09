#include "viso.h"

#include <math.h>
#include <iostream>

using namespace std;

VisualOdometry::VisualOdometry(parameters param) : param(param)
{
	J = 0;
	p_observe = 0;
	p_predict = 0;
	matcher = new Matcher(param.match);
	tr_delta.resize(6);
	// Tr_delta = Matrix::eye(4);
	Tr_valid = false;
	srand(0);
}

VisualOdometry::~VisualOdometry()
{
	delete matcher;
}

bool VisualOdometry::updateMotion()
{
	if ( p_matched.size() < 4 )
	{
		std::cerr << "too few matches < 4" << std::endl;
		inliers.clear();
		return false;
	}
	// estimate motion
	vector<double> _tr_delta = estimateMotion(p_matched);  // p_matched = p_matched_2 (only dense matched features used)

	// on failure
	if (_tr_delta.size() != 6)
	{
		cerr << "ERROR updateMotion(): _tr_delta.size()=" << _tr_delta.size()<< endl;
		return false;
	}

	// catch wrong RT computation
	// for replace mode, a higher limit is expected
	if (fabs(_tr_delta[3])>=10 || fabs(_tr_delta[4])>=10 || fabs(_tr_delta[5])>=10)
	{
		cerr << "ERROR updateMotion(): abs(_tr_delta)>=10" << endl;
		return false;
	}

	
	// set transformation matrix (previous to current frame)
	tr_delta = _tr_delta;
	Tr_valid = true;

	// success
	return true;
}

Matrix VisualOdometry::transformationVectorToMatrix(vector<double> tr)
{
	// extract parameters
	double rx = tr[0];
	double ry = tr[1];
	double rz = tr[2];
	double tx = tr[3];
	double ty = tr[4];
	double tz = tr[5];

	// precompute sine/cosine
	double sx = sin(rx);
	double cx = cos(rx);
	double sy = sin(ry);
	double cy = cos(ry);
	double sz = sin(rz);
	double cz = cos(rz);

	// compute transformation
	Matrix Tr(4, 4);
	Tr.val[0][0] = +cy*cz;          Tr.val[0][1] = -cy*sz;          Tr.val[0][2] = +sy;    Tr.val[0][3] = tx;
	Tr.val[1][0] = +sx*sy*cz + cx*sz; Tr.val[1][1] = -sx*sy*sz + cx*cz; Tr.val[1][2] = -sx*cy; Tr.val[1][3] = ty;
	Tr.val[2][0] = -cx*sy*cz + sx*sz; Tr.val[2][1] = +cx*sy*sz + sx*cz; Tr.val[2][2] = +cx*cy; Tr.val[2][3] = tz;
	Tr.val[3][0] = 0;               Tr.val[3][1] = 0;               Tr.val[3][2] = 0;      Tr.val[3][3] = 1;
	return Tr;
}

vector<int32_t> VisualOdometry::getRandomSample(int32_t N, int32_t num)
{

	// init sample and totalset
	vector<int32_t> sample;
	vector<int32_t> totalset;

	// create vector containing all indices
	for (int32_t i = 0; i < N; i++)
		totalset.push_back(i);

	// add num indices to current sample
	sample.clear();

	for (int32_t i = 0; i < num; i++)
	{
		int32_t j = rand() % totalset.size();
		sample.push_back(totalset[j]);
		totalset.erase(totalset.begin() + j);        // delete already selected position before next selection
	}

	// return sample
	return sample;
}
