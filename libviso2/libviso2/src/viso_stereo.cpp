#include "viso_stereo.h"
#include <iostream>

#include <chrono>

using namespace std;

VisualOdometryStereo::VisualOdometryStereo(parameters param) : param(param), VisualOdometry(param)
{
	matcher->setIntrinsics(param.calib.f, param.calib.cu, param.calib.cv, param.base);
}

VisualOdometryStereo::~VisualOdometryStereo()
{
}

bool VisualOdometryStereo::process(uint8_t *I1, uint8_t *I2, int32_t* dims, bool replace)
{
	auto t1 = std::chrono::system_clock::now();
	matcher->pushBack(I1, I2, dims, replace);

	int left_pre, right_pre, left_curr, right_curr;
	matcher->getFeatureNumber(left_pre, right_pre, left_curr, right_curr);
	cout << "previous feature: " << left_pre << ", " << right_pre << "; current feature: "<< left_curr << ", " << right_curr << endl;

	auto t2 = std::chrono::system_clock::now();
	matcher->matchFeatures();
	auto t3 = std::chrono::system_clock::now();

	//cout << "size before = " << matcher->getMatches().size();
	if (param.bucket.bucket_ornot)
		matcher->bucketFeatures(param.bucket.max_features, param.bucket.bucket_width, param.bucket.bucket_height);
	
	p_matched = matcher->getMatches();
	//	cout << "size after = " << p_matched.size() ;

	cout << "p_matched.size()="<< p_matched.size()  << endl;
	auto t4 = std::chrono::system_clock::now();
	if (p_matched.size() < 6 )
	{
		inliers.clear();
		return false;
	}
	// std::chrono::duration<double> diff1 = t2 - t1;
	// std::chrono::duration<double> diff2 = t3 - t2;
	// std::chrono::duration<double> diff3 = t4 - t3;
	//cout << "diff1=" << diff1.count() << ", diff2=" << diff2.count() << ", diff3=" << diff3.count() << endl;

	return updateMotion();
}

vector<double> VisualOdometryStereo::estimateMotion(vector<Matcher::p_match> p_matched)
{

	// return value
	bool success = true;

	// compute minimum distance for RANSAC samples
	double width_max = 0, height_max = 0;
	double width_min = 1e5, height_min = 1e5;

	for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it != p_matched.end(); it++)
	{
		if (it->u1c > width_max)  width_max = it->u1c;
		if (it->u1c < width_min)  width_min = it->u1c;

		if (it->v1c > height_max) height_max = it->v1c;
		if (it->v1c < height_min) height_min = it->v1c;

	}

	// random pick 3 matches that a min-dist between them

	double min_dist = min(width_max-width_min, height_max-height_min) / 5.0;	// default divided by 3.0

	// get number of matches
	int32_t N = p_matched.size();
	if (N < 6)
		return vector<double>();

	// allocate dynamic memory
	X = new double[N];
	Y = new double[N];
	Z = new double[N];
	J = new double[4 * N * 6];					// yx: save Jacobian matrix for each point (6*4: 6 functions and 4 unknowns)
	p_predict = new double[4 * N];
	p_observe = new double[4 * N];
	p_residual = new double[4 * N];

	if (matcher->param.half_resolution)
	{
		_cu = param.calib.cu / 2;
		_cv = param.calib.cv / 2;
		_f = param.calib.f / 2;
	}
	else
	{
		_cu = param.calib.cu;
		_cv = param.calib.cv;
		_f = param.calib.f; // in pixels
	}

	// project matches of previous image into 3d
	for (int32_t i = 0; i < N; i++)
	{
		double d = max(p_matched[i].u1p - p_matched[i].u2p, 0.0001f);				// d = xl - xr
		X[i] = (p_matched[i].u1p - _cu)*param.base / d;								// X = (u1p - calib.cu)*baseline/d
		Y[i] = (p_matched[i].v1p - _cv)*param.base / d;								// Y = (v1p - calib.cv)*baseline/d
		Z[i] = _f*param.base / d;													// Z = f*baseline/d
	}

	// loop variables
	vector<double> tr_delta;				// yx: ransac: bestfit
	vector<double> tr_delta_curr;			// yx: ransac: maybemodel (for current sample)
	tr_delta_curr.resize(6);

	// clear parameter vector
	inliers.clear();

	// yx: for model based ransac
	double Pbest = 0.0;
	double p = 0.99;
	int32_t N_ransac = param.ransac_iters;
	int32_t N_ransac2;

	// initial RANSAC estimate
	for (int32_t k = 0; k < N_ransac; k++)
	{

		vector<int32_t> active;

		// draw random sample set
		// active = getRandomSample(N, 3);

		
		int32_t selection_iter = 0;
		while (true)
		{
			active = getRandomSample(N, 3);
			double x0 = (p_matched[active[0]].u1p - p_matched[active[1]].u1p)*(p_matched[active[0]].u1p - p_matched[active[1]].u1p);
			double y0 = (p_matched[active[0]].v1p - p_matched[active[1]].v1p)*(p_matched[active[0]].v1p - p_matched[active[1]].v1p);
			double x1 = (p_matched[active[1]].u1p - p_matched[active[2]].u1p)*(p_matched[active[1]].u1p - p_matched[active[2]].u1p);
			double y1 = (p_matched[active[1]].v1p - p_matched[active[2]].v1p)*(p_matched[active[1]].v1p - p_matched[active[2]].v1p);
			double x2 = (p_matched[active[2]].u1p - p_matched[active[0]].u1p)*(p_matched[active[2]].u1p - p_matched[active[0]].u1p);
			double y2 = (p_matched[active[2]].v1p - p_matched[active[0]].v1p)*(p_matched[active[2]].v1p - p_matched[active[0]].v1p);
			double d0 = sqrt(x0 + y0); double d1 = sqrt(x1 + y1); double d2 = sqrt(x2 + y2);

			if (d0 >= min_dist && d1 >= min_dist && d2 >= min_dist)
				break;
			selection_iter++;
			if (selection_iter >= (N * (N - 1) * (N - 2)) / (3 * 2)) break;			// total combination C(N, 3) = N!/(3!*(N-3)!))
		}
		

		// clear parameter vector                            // for every iteration, good model has been returned to bestfit, therefore clear current model for next iteration
		for (int32_t i = 0; i < 6; i++)
			tr_delta_curr[i] = 0;

		// minimize reprojection errors
		VisualOdometryStereo::result result = UPDATED;
		int32_t iter = 0;
		while (result == UPDATED)
		{
			result = updateParameters(p_matched, active, tr_delta_curr, 1, 1e-6);
			if (result == CONVERGED)
				break;

			if (iter++ > 20)
			{
				// hm: this happens very frequently
				//cerr << "ERROR estimateMotion(): updateParameters EXCEED 20 iterations" << endl;
				break;
			}
				
		}

		// overwrite best parameters if we have more inliers
		if (result != FAILED)
		{
			vector<int32_t> inliers_curr = getInlier(p_matched, tr_delta_curr);
			if (inliers_curr.size() > inliers.size())
			{
				inliers = inliers_curr;
				tr_delta = tr_delta_curr;
			}
		}
		//  else
		//  	cerr << "ERROR estimateMotion(): updateParameters result FAILED" << endl;

		// probility of observing an inlier
		/*
		double Pin = double(inliers.size()) / double(N);
		if (Pin > Pbest) Pbest = Pin;
		N_ransac2 = log(1 - p) / log(1 - pow(Pbest, 3));

		// N_ransac = 200 means, Pbest > 0.28
		if (N_ransac2 < N_ransac &&  k < N_ransac2)
		{
			N_ransac = N_ransac2;
		}
		*/
	}

	// final optimization (refinement)
	if (inliers.size() >= 6)
	{
		if (tr_delta.size() != 6 )
			cerr << "ERROR estimateMotion(): inlier VALID but tr_delta INVALID" << endl;
		int32_t iter = 0;
		VisualOdometryStereo::result result = UPDATED;
		while (result == UPDATED)
		{
			result = updateParameters(p_matched, inliers, tr_delta, 1, 1e-8);
			if (result == CONVERGED)
				break;
			if (iter++ > 100)
			{
				cerr << "ERROR estimateMotion(): optimisation step -> updateParameters EXCEED 100 iterations" << endl;
				break;
			}
		}

		// not converged
		if (result != CONVERGED)
		{
			cerr << "ERROR estimateMotion(): optimisation step -> updateParameters NOT converged = " << result << endl;
			success = false;
		}
			
		if (result == FAILED)
			cerr << "ERROR estimateMotion(): optimisation step -> updateParameters FAILED" << endl;
		if (result == UPDATED)
			cerr << "ERROR estimateMotion(): optimisation step -> updateParameters UPDATED" << endl;

		// not enough inliers
	}
	else
	{
		cerr << "ERROR estimateMotion(): inliers.size()<6" << endl;
		success = false;
	}

	// release dynamic memory
	delete[] X;
	delete[] Y;
	delete[] Z;
	delete[] J;
	delete[] p_predict;
	delete[] p_observe;
	delete[] p_residual;

	if (tr_delta.size() !=6)
	{
		cerr << "ERROR estimateMotion(): tr_delta.size() !=6" << endl;
	}


	// parameter estimate succeeded?
	if (success) return tr_delta;
	else         return vector<double>();
}

vector<int32_t> VisualOdometryStereo::getInlier(vector<Matcher::p_match> &p_matched, vector<double> &tr)
{

	// mark all observations active
	vector<int32_t> active;
	for (int32_t i = 0; i < (int32_t)p_matched.size(); i++)
		active.push_back(i);

	// extract observations and compute predictions
	computeObservations(p_matched, active);
	computeResidualsAndJacobian(tr, active);

	// compute inliers
	vector<int32_t> inliers;
	for (int32_t i = 0; i < (int32_t)p_matched.size(); i++)
		if (pow(p_observe[4 * i + 0] - p_predict[4 * i + 0], 2) + pow(p_observe[4 * i + 1] - p_predict[4 * i + 1], 2) +
			pow(p_observe[4 * i + 2] - p_predict[4 * i + 2], 2) + pow(p_observe[4 * i + 3] - p_predict[4 * i + 3], 2) < param.inlier_threshold*param.inlier_threshold)
			inliers.push_back(i);
	return inliers;
}

VisualOdometryStereo::result VisualOdometryStereo::updateParameters(vector<Matcher::p_match> &p_matched, vector<int32_t> &active, vector<double> &tr, double step_size, double eps)
{
	// we need at least 3 observations
	if (active.size() < 3)
		return FAILED;

	// extract observations and compute predictions
	computeObservations(p_matched, active);                     // save coordinates of current left and right features that in the ransac sample
	computeResidualsAndJacobian(tr, active);

	// init
	Matrix A(6, 6);
	Matrix B(6, 1);

	// fill matrices A and B
	// JT*J = A
	for (int32_t m = 0; m < 6; m++)
	{
		for (int32_t n = 0; n < 6; n++)
		{
			double a = 0;
			for (int32_t i = 0; i < 4 * (int32_t)active.size(); i++)
			{
				a += J[i * 6 + m] * J[i * 6 + n];
			}
			A.val[m][n] = a;
		}
		double b = 0;
		for (int32_t i = 0; i < 4 * (int32_t)active.size(); i++)
		{
			b += J[i * 6 + m] * (p_residual[i]);
		}
		B.val[m][0] = b;
	}
	//double deta = A.det;
	// perform elimination
	if (B.solve(A))
	{
		bool converged = true;
		for (int32_t m = 0; m < 6; m++)
		{
			tr[m] += step_size*B.val[m][0];
			if (fabs(B.val[m][0]) > eps)
				converged = false;
		}
		if (converged)
			return CONVERGED;
		else
			return UPDATED;
	}
	else
	{
		return FAILED;
	}
}

void VisualOdometryStereo::computeObservations(vector<Matcher::p_match> &p_matched, vector<int32_t> &active)
{

	// set all observations
	for (int32_t i = 0; i < (int32_t)active.size(); i++)
	{
		p_observe[4 * i + 0] = p_matched[active[i]].u1c; // active saves index of ransac sample
		p_observe[4 * i + 1] = p_matched[active[i]].v1c; // 
		p_observe[4 * i + 2] = p_matched[active[i]].u2c; // 
		p_observe[4 * i + 3] = p_matched[active[i]].v2c; // 
	}
}

void VisualOdometryStereo::computeResidualsAndJacobian(vector<double> &tr, vector<int32_t> &active)
{

	// extract motion parameters
	double rx = tr[0]; double ry = tr[1]; double rz = tr[2];
	double tx = tr[3]; double ty = tr[4]; double tz = tr[5];

	// precompute sine/cosine
	double sx = sin(rx); double cx = cos(rx); double sy = sin(ry);     // rx = alpha, ry = beta, rz = gamma
	double cy = cos(ry); double sz = sin(rz); double cz = cos(rz);

	// compute rotation matrix and derivatives
	// rotation matrix = Rz*Ry*Rx
	double r00 = +cy*cz;          double r01 = -cy*sz;          double r02 = +sy;
	double r10 = +sx*sy*cz + cx*sz; double r11 = -sx*sy*sz + cx*cz; double r12 = -sx*cy;
	double r20 = -cx*sy*cz + sx*sz; double r21 = +cx*sy*sz + sx*cz; double r22 = +cx*cy;

	double rdrx10 = +cx*sy*cz - sx*sz; double rdrx11 = -cx*sy*sz - sx*cz; double rdrx12 = -cx*cy;
	double rdrx20 = +sx*sy*cz + cx*sz; double rdrx21 = -sx*sy*sz + cx*cz; double rdrx22 = -sx*cy;
	double rdry00 = -sy*cz;          double rdry01 = +sy*sz;          double rdry02 = +cy;
	double rdry10 = +sx*cy*cz;       double rdry11 = -sx*cy*sz;       double rdry12 = +sx*sy;
	double rdry20 = -cx*cy*cz;       double rdry21 = +cx*cy*sz;       double rdry22 = -cx*sy;
	double rdrz00 = -cy*sz;          double rdrz01 = -cy*cz;
	double rdrz10 = -sx*sy*sz + cx*cz; double rdrz11 = -sx*sy*cz - cx*sz;
	double rdrz20 = +cx*sy*sz + sx*cz; double rdrz21 = +cx*sy*cz - sx*sz;

	// loop variables
	double X1p, Y1p, Z1p;
	double X1c, Y1c, Z1c, X2c;
	double X1cd, Y1cd, Z1cd;

	// for all observations do
	for (int32_t i = 0; i < (int32_t)active.size(); i++)
	{

		// get 3d point in previous coordinate system
		X1p = X[active[i]];
		Y1p = Y[active[i]];
		Z1p = Z[active[i]];

		// compute 3d point in current left coordinate system
		X1c = r00*X1p + r01*Y1p + r02*Z1p + tx;
		Y1c = r10*X1p + r11*Y1p + r12*Z1p + ty;
		Z1c = r20*X1p + r21*Y1p + r22*Z1p + tz;

		// weighting
		double weight = 1.0;
		if (param.reweighting)
			weight = 1.0 / (fabs(p_observe[4 * i + 0] - _cu) / fabs(_cu) + 0.05);   // only for current left image

		// compute 3d point in current right coordinate system
		X2c = X1c - param.base;

		// for all paramters do  // six parameters: 3 rotations and 3 translations
		for (int32_t j = 0; j < 6; j++)
		{

			// derivatives of 3d pt. in curr. left coordinates wrt. param j
			switch (j)
			{
			case 0: X1cd = 0;
				Y1cd = rdrx10*X1p + rdrx11*Y1p + rdrx12*Z1p;
				Z1cd = rdrx20*X1p + rdrx21*Y1p + rdrx22*Z1p;
				break;
			case 1: X1cd = rdry00*X1p + rdry01*Y1p + rdry02*Z1p;
				Y1cd = rdry10*X1p + rdry11*Y1p + rdry12*Z1p;
				Z1cd = rdry20*X1p + rdry21*Y1p + rdry22*Z1p;
				break;
			case 2: X1cd = rdrz00*X1p + rdrz01*Y1p;
				Y1cd = rdrz10*X1p + rdrz11*Y1p;
				Z1cd = rdrz20*X1p + rdrz21*Y1p;
				break;
			case 3: X1cd = 1; Y1cd = 0; Z1cd = 0; break;
			case 4: X1cd = 0; Y1cd = 1; Z1cd = 0; break;
			case 5: X1cd = 0; Y1cd = 0; Z1cd = 1; break;
			}

			// set jacobian entries (project via K)
			J[(4 * i + 0) * 6 + j] = weight*_f*(X1cd*Z1c - X1c*Z1cd) / (Z1c*Z1c); // left u'
			J[(4 * i + 1) * 6 + j] = weight*_f*(Y1cd*Z1c - Y1c*Z1cd) / (Z1c*Z1c); // left v'
			J[(4 * i + 2) * 6 + j] = weight*_f*(X1cd*Z1c - X2c*Z1cd) / (Z1c*Z1c); // right u'
			J[(4 * i + 3) * 6 + j] = weight*_f*(Y1cd*Z1c - Y1c*Z1cd) / (Z1c*Z1c); // right v'
		}

		// set prediction (project via K)
		p_predict[4 * i + 0] = _f*X1c / Z1c + _cu; // left center u
		p_predict[4 * i + 1] = _f*Y1c / Z1c + _cv; // left v
		p_predict[4 * i + 2] = _f*X2c / Z1c + _cu; // right u
		p_predict[4 * i + 3] = _f*Y1c / Z1c + _cv; // right v

		// set residuals
		p_residual[4 * i + 0] = weight*(p_observe[4 * i + 0] - p_predict[4 * i + 0]);
		p_residual[4 * i + 1] = weight*(p_observe[4 * i + 1] - p_predict[4 * i + 1]);
		p_residual[4 * i + 2] = weight*(p_observe[4 * i + 2] - p_predict[4 * i + 2]);
		p_residual[4 * i + 3] = weight*(p_observe[4 * i + 3] - p_predict[4 * i + 3]);
	}
}

