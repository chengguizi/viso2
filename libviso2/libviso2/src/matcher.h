#ifndef __MATCHER_H__
#define __MATCHER_H__

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <iostream>
#include <math.h>
#include <emmintrin.h>
#include <algorithm>
#include <vector>

#include "matrix.h"

using namespace std;

class Matcher
{

public:

	class Rectangle{
		

	public:

		int _x1, _y1;
		int _x2, _y2;
		Rectangle(int x1, int y1, int x2, int y2) : _x1(x1), _y1(y1), _x2(x2), _y2(y2) {
		}

		bool contains(int x, int y){
			if ( x > _x1 && y > _y1 && x < _x2 && y < _y2)
				return true;
			return false;
		}
	};

	// bucketing parameters
	struct bucketing {  
		bool bucket_ornot;
		int32_t max_features;  // maximal number of features per bucket 
		bucketing () 
		{
			bucket_ornot = 0;
			max_features  = 5;
		}
	};

	// parameter settings
	struct parameters
	{
		int32_t match_binsize;          // matching bin width/height (affects efficiency only)
		int32_t match_radius;           // matching radius (du/dv in pixels)
		int32_t match_disp_tolerance;   // dv tolerance for stereo matches (in pixels)
		int32_t outlier_disp_tolerance; // outlier removal: disparity tolerance (in pixels)
		int32_t outlier_flow_tolerance; // outlier removal: flow tolerance (in pixels)
		bool multi_stage;            // 0=disabled,1=multistage matching (denser and faster)
		bool half_resolution;        // 0=disabled,1=match at half resolution, refine at full resolution
		bool remove_outliers;		// 0=disabled,1=remove outliers using triangulation
		int32_t refinement;             // refinement (0=none,1=pixel,2=subpixel)
		int32_t fast_threshold_dense;
		int32_t fast_threshold_sparse;
		int32_t numFastFeature_sparse;
		int32_t numFastFeature_dense;
		double  f, cu, cv, base;           // calibration (only for match prediction)
		bucketing bucket;

		// default settings
		parameters()
		{
			match_binsize = 50;
			match_radius = 200;
			match_disp_tolerance = 2;		// default is 2
			outlier_disp_tolerance = 5;		// default is 5
			outlier_flow_tolerance = 5;
			multi_stage = false;
			half_resolution = false;
			remove_outliers = false;
			fast_threshold_dense = 10;
			fast_threshold_sparse = 40;
			numFastFeature_sparse = 200;
			numFastFeature_dense = 400;
			refinement = 0;
		}
	};

	// constructor (with default parameters)
	Matcher(parameters param);

	// deconstructor
	~Matcher();

	// intrinsics
	void setIntrinsics(double f, double cu, double cv, double base)
	{
		param.f = f;
		param.cu = cu;
		param.cv = cv;
		param.base = base;
	}

	// structure for storing matches
	struct p_match
	{
		float   u1p, v1p; // u,v-coordinates in previous left  image
		int32_t i1p;     // feature index (for tracking)
		float   u2p, v2p; // u,v-coordinates in previous right image
		int32_t i2p;     // feature index (for tracking)
		float   u1c, v1c; // u,v-coordinates in current  left  image
		int32_t i1c;     // feature index (for tracking)
		float   u2c, v2c; // u,v-coordinates in current  right image
		int32_t i2c;     // feature index (for tracking)

		p_match()
		{
		}
		p_match(float u1p, float v1p, int32_t i1p, float u2p, float v2p, int32_t i2p,
			float u1c, float v1c, int32_t i1c, float u2c, float v2c, int32_t i2c):
			u1p(u1p), v1p(v1p), i1p(i1p), u2p(u2p), v2p(v2p), i2p(i2p),
			u1c(u1c), v1c(v1c), i1c(i1c), u2c(u2c), v2c(v2c), i2c(i2c)
		{
		}
	};

	// computes features from left/right images and pushes them back to a ringbuffer,
	// which interally stores the features of the current and previous image pair
	// use this function for stereo or quad matching
	// input: I1,I2 .......... pointers to left and right image (row-aligned), range [0..255]
	//        dims[0,1] ...... image width and height (both images must be rectified and of same size)
	//        dims[2] ........ bytes per line (often equals width)
	
	void pushBack(uint8_t *I1, uint8_t* I2, double frame_time,int32_t* dims, const bool replace);

	// computes features from a single image and pushes it back to a ringbuffer,
	// which interally stores the features of the current and previous image pair
	// use this function for flow computation
	// parameter description see above
	// void pushBack(uint8_t *I1, int32_t* dims, const bool replace)
	// {
	// 	pushBack(I1, 0, dims, replace);
	// }

	// match features currently stored in ring buffer (current and previous frame)
	
	void matchFeatures();

	// return vector with matched feature points and indices
	std::vector<Matcher::p_match> getMatches()
	{
		return p_matched_2;			// yx: default is p_matched_2
	}

	int getNumRematches()
	{
		return _rematched;
	}

	// given a vector of inliers computes gain factor between the current and
	// the previous frame. this function is useful if you want to reconstruct 3d
	// and you want to cancel the change of (unknown) camera gain.
	float getGain(std::vector<int32_t> inliers);

	void getFeatureNumber(int& left_pre, int& right_pre, int& left_curr, int& right_curr)
	{
		left_pre = n1p2;
		right_pre = n2p2;
		left_curr = n1c2;
		right_curr = n2c2;
	}

	double getPercentageFilledBin(){return percentage_filled_bin;}

	double getPreviousFrameTimestamp(){return time_i1;}

	double getCurrentFrameTimestamp(){return time_i2;}

	void setMovingObjects(std::vector<Rectangle> rects){moving_objects = rects;}

	std::vector<Rectangle> getMovingObjects(){return moving_objects;}

	// make it public
	parameters param;

private:

	// structure for storing interest points
	struct maximum
	{
		int32_t u;   // u-coordinate
		int32_t v;   // v-coordinate
		int32_t val; // value
		int32_t c;   // empty = 0
		int32_t d1, d2, d3, d4, d5, d6, d7, d8; // descriptor
		maximum()
		{
		}
		maximum(int32_t u, int32_t v, int32_t val, int32_t c) :u(u), v(v), val(val), c(c)
		{
		}
	};

	// u/v ranges for matching stage 0-3
	struct range
	{
		float u_min[4];
		float u_max[4];
		float v_min[4];
		float v_max[4];
	};

	struct delta
	{
		float val[8];
		delta()
		{
		}
		delta(float v)
		{
			for (int32_t i = 0; i < 8; i++)
				val[i] = v;
		}
	};

	// computes the address offset for coordinates u,v of an image of given width
	inline int32_t getAddressOffsetImage(const int32_t& u, const int32_t& v, const int32_t& width)
	{
		return v*width + u;
	}

	// FAST Detector
	void FastFeatures(uint8_t* I, const int32_t* dims, vector<maximum> &maxima, int32_t threshold, int32_t numFastFeature);


	// descriptor functions
	//inline uint8_t saturate(int16_t in);
	//void filterImageAll(uint8_t* I, uint8_t* I_du, uint8_t* I_dv, int16_t* I_f1, int16_t* I_f2, const int* dims);
	//void filterImageSobel(uint8_t* I, uint8_t* I_du, uint8_t* I_dv, const int* dims);
	inline void computeDescriptor(const uint8_t* I_du, const uint8_t* I_dv, const int32_t &bpl, const int32_t &u, const int32_t &v, uint8_t *desc_addr);
	inline void computeSmallDescriptor(const uint8_t* I_du, const uint8_t* I_dv, const int32_t &bpl, const int32_t &u, const int32_t &v, uint8_t *desc_addr);

	void computeDescriptors(uint8_t* I_du, uint8_t* I_dv, const int32_t bpl, std::vector<Matcher::maximum> &maxima);

	void getHalfResolutionDimensions(const int32_t *dims, int32_t *dims_half);
	uint8_t* createHalfResolutionImage(uint8_t *I, const int32_t* dims);

	void computeFeatures(uint8_t *I, const int32_t* dims, int32_t* &max1, int32_t &num1, int32_t* &max2, int32_t &num2, uint8_t* &I_du, uint8_t* &I_dv, uint8_t* &I_du_full, uint8_t* &I_dv_full);

	// matching functions
	void computePriorStatistics(std::vector<Matcher::p_match> &p_matched);
	void createIndexVector(int32_t* m, int32_t n, std::vector<int32_t> *k, const int32_t &u_bin_num, const int32_t &v_bin_num);
	inline void findMatch(int32_t* m1, const int32_t &i1, int32_t* m2, const int32_t &step_size,
		std::vector<int32_t> *k2, const int32_t &u_bin_num, const int32_t &v_bin_num, const int32_t &stat_bin,
		int32_t& min_ind, int32_t stage, bool flow, bool use_prior, double u_ = -1, double v_ = -1);
	void matching(int32_t *m1p, int32_t *m2p, int32_t *m1c, int32_t *m2c,
		int32_t n1p, int32_t n2p, int32_t n1c, int32_t n2c,
		std::vector<Matcher::p_match> &p_matched, bool use_prior);

	// outlier removal
	void removeOutliers(std::vector<Matcher::p_match> &p_matched);

	// parabolic fitting
	bool parabolicFitting(const uint8_t* I1_du, const uint8_t* I1_dv, const int32_t* dims1,
		const uint8_t* I2_du, const uint8_t* I2_dv, const int32_t* dims2,
		const float &u1, const float &v1,
		float       &u2, float       &v2,
		Matrix At, Matrix AtA,
		uint8_t* desc_buffer);
	void relocateMinimum(const uint8_t* I1_du, const uint8_t* I1_dv, const int32_t* dims1,
		const uint8_t* I2_du, const uint8_t* I2_dv, const int32_t* dims2,
		const float &u1, const float &v1,
		float       &u2, float       &v2,
		uint8_t* desc_buffer);
	void refinement(std::vector<Matcher::p_match> &p_matched);
	
	// mean for gain computation
	inline float mean(const uint8_t* I, const int32_t &bpl, const int32_t &u_min, const int32_t &u_max, const int32_t &v_min, const int32_t &v_max);

	// parameters
	int32_t    margin;

	// hm: below are the results from the fast features
	// last number is passes, so only *2 are used for dense matching
	int32_t *m1p1, *m2p1, *m1c1, *m2c1;     // *m1p1 points to (1st pass) sparse maxima features for previous left image
	int32_t *m1p2, *m2p2, *m1c2, *m2c2;	   // *m2c2 points to (2nd pass) dense maxima features for current right image
	int32_t n1p1, n2p1, n1c1, n2c1;		   // n1p1 is the number of sparse maxima features for previous letf image
	int32_t n1p2, n2p2, n1c2, n2c2;		   // n2p2 is the number of dense maxima features for preivous right image
											// n1c1 is the number of sparse maxima features for current left image
	uint8_t *I1p, *I2p, *I1c, *I2c;
	uint8_t *I1p_du, *I2p_du, *I1c_du, *I2c_du;
	uint8_t *I1p_dv, *I2p_dv, *I1c_dv, *I2c_dv;
	uint8_t *I1p_du_full, *I2p_du_full, *I1c_du_full, *I2c_du_full; // only needed for
	uint8_t *I1p_dv_full, *I2p_dv_full, *I1c_dv_full, *I2c_dv_full; // half-res matching

	double time_i1, time_i2;

	std::vector<Rectangle> moving_objects;

	int32_t dims_p[3], dims_c[3];

	std::vector<Matcher::p_match> p_matched_1;
	std::vector<Matcher::p_match> p_matched_2;
	std::vector<Matcher::range>   ranges;

	int _rematched;

	double percentage_filled_bin;
};

#endif

