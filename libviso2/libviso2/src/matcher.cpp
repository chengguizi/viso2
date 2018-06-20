#include "matcher.h"
#include "triangle.h"
#include "filter.h"

// fast feature detector
#include <opencv2/core/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc/imgproc.hpp> 
#include <opencv2/features2d/features2d.hpp>

#include <algorithm>    // std::sort

// fast cpp version
#include "fast.h"

using namespace std;
using namespace cv;

// constructor (with default parameters)
Matcher::Matcher(parameters param) : param(param)
{
	// init match ring buffer to zero
	m1p1 = 0; n1p1 = 0;
	m1p2 = 0; n1p2 = 0;
	m2p1 = 0; n2p1 = 0;
	m2p2 = 0; n2p2 = 0;
	m1c1 = 0; n1c1 = 0;
	m1c2 = 0; n1c2 = 0;
	m2c1 = 0; n2c1 = 0;
	m2c2 = 0; n2c2 = 0;
	I1p = 0; I2p = 0;
	I1c = 0; I2c = 0;

	I1p_du = 0; I1p_dv = 0;
	I2p_du = 0; I2p_dv = 0;
	I1c_du = 0; I1c_dv = 0;
	I2c_du = 0; I2c_dv = 0;
	I1p_du_full = 0; I1p_dv_full = 0;
	I2p_du_full = 0; I2p_dv_full = 0;
	I1c_du_full = 0; I1c_dv_full = 0;
	I2c_du_full = 0; I2c_dv_full = 0;

	// margin needed to compute descriptor + sobel responses
	margin = 5 + 1;

	// adjust match radius on half resolution
	if (param.half_resolution)
		this->param.match_radius /= 2;
}

// deconstructor
Matcher::~Matcher()
{
	if (I1p)          _mm_free(I1p);
	if (I2p)          _mm_free(I2p);
	if (I1c)          _mm_free(I1c);
	if (I2c)          _mm_free(I2c);
	if (m1p1)         _mm_free(m1p1);
	if (m1p2)         _mm_free(m1p2);
	if (I1p_du)       _mm_free(I1p_du);
	if (I1p_dv)       _mm_free(I1p_dv);
	if (I1p_du_full)  _mm_free(I1p_du_full);
	if (I1p_dv_full)  _mm_free(I1p_dv_full);
	if (m2p1)         _mm_free(m2p1);
	if (m2p2)         _mm_free(m2p2);
	if (I2p_du)       _mm_free(I2p_du);
	if (I2p_dv)       _mm_free(I2p_dv);
	if (I2p_du_full)  _mm_free(I2p_du_full);
	if (I2p_dv_full)  _mm_free(I2p_dv_full);
	if (m1c1)         _mm_free(m1c1);
	if (m1c2)         _mm_free(m1c2);
	if (I1c_du)       _mm_free(I1c_du);
	if (I1c_dv)       _mm_free(I1c_dv);
	if (I1c_du_full)  _mm_free(I1c_du_full);
	if (I1c_dv_full)  _mm_free(I1c_dv_full);
	if (m2c1)         _mm_free(m2c1);
	if (m2c2)         _mm_free(m2c2);
	if (I2c_du)       _mm_free(I2c_du);
	if (I2c_dv)       _mm_free(I2c_dv);
	if (I2c_du_full)  _mm_free(I2c_du_full);
	if (I2c_dv_full)  _mm_free(I2c_dv_full);

}

void Matcher::pushBack(uint8_t *I1, uint8_t* I2, int32_t* dims, const bool replace)
{
	// image dimensions
	int32_t width = dims[0];
	int32_t height = dims[1];
	int32_t bpl = dims[2];

	// sanity check
	if (width <= 0 || height <= 0 || bpl < width || I1 == 0)
	{
		cerr << "ERROR: Image dimension mismatch!" << endl;
		return;
	}

	// free preivous image pair 
 if (replace) {
    if (I1c)         _mm_free(I1c);
    if (I2c)         _mm_free(I2c);
    if (m1c1)        _mm_free(m1c1);
    if (m1c2)        _mm_free(m1c2);
    if (I1c_du)      _mm_free(I1c_du);
    if (I1c_dv)      _mm_free(I1c_dv);
    if (I1c_du_full) _mm_free(I1c_du_full);
    if (I1c_dv_full) _mm_free(I1c_dv_full);
    if (m2c1)        _mm_free(m2c1);
    if (m2c2)        _mm_free(m2c2);
    if (I2c_du)      _mm_free(I2c_du);
    if (I2c_dv)      _mm_free(I2c_dv);
    if (I2c_du_full) _mm_free(I2c_du_full);
    if (I2c_dv_full) _mm_free(I2c_dv_full);
  } else {
    if (I1p)         _mm_free(I1p);
    if (I2p)         _mm_free(I2p);
    if (m1p1)        _mm_free(m1p1);
    if (m1p2)        _mm_free(m1p2);
    if (I1p_du)      _mm_free(I1p_du);
    if (I1p_dv)      _mm_free(I1p_dv);
    if (I1p_du_full) _mm_free(I1p_du_full);
    if (I1p_dv_full) _mm_free(I1p_dv_full);
    if (m2p1)        _mm_free(m2p1);
    if (m2p2)        _mm_free(m2p2);
    if (I2p_du)      _mm_free(I2p_du);
    if (I2p_dv)      _mm_free(I2p_dv);
    if (I2p_du_full) _mm_free(I2p_du_full);
    if (I2p_dv_full) _mm_free(I2p_dv_full);
    m1p1 = m1c1; n1p1 = n1c1;
    m1p2 = m1c2; n1p2 = n1c2;
    m2p1 = m2c1; n2p1 = n2c1;
    m2p2 = m2c2; n2p2 = n2c2;
    I1p         = I1c;
    I2p         = I2c;
    I1p_du      = I1c_du;
    I1p_dv      = I1c_dv;
    I1p_du_full = I1c_du_full;
    I1p_dv_full = I1c_dv_full;
    I2p_du      = I2c_du;
    I2p_dv      = I2c_dv;
    I2p_du_full = I2c_du_full;
    I2p_dv_full = I2c_dv_full;
    dims_p[0]   = dims_c[0];
    dims_p[1]   = dims_c[1];
    dims_p[2]   = dims_c[2];
  }


	// set new dims (bytes per line must be multiple of 16)
	dims_c[0] = width;
	dims_c[1] = height;
	dims_c[2] = width + 15 - (width - 1) % 16;

	// copy images to byte aligned memory
	I1c = (uint8_t*)_mm_malloc(dims_c[2] * dims_c[1] * sizeof(uint8_t), 16);
	I2c = (uint8_t*)_mm_malloc(dims_c[2] * dims_c[1] * sizeof(uint8_t), 16);

	if (dims_c[2] == bpl)
	{
		memcpy(I1c, I1, dims_c[2] * dims_c[1] * sizeof(uint8_t));
		if (I2 != 0)
			memcpy(I2c, I2, dims_c[2] * dims_c[1] * sizeof(uint8_t));
	}
	else
	{
		for (int32_t v = 0; v < height; v++)
		{
			memcpy(I1c + v*dims_c[2], I1 + v*bpl, dims_c[0] * sizeof(uint8_t));
			if (I2 != 0)
				memcpy(I2c + v*dims_c[2], I2 + v*bpl, dims_c[0] * sizeof(uint8_t));
		}
	}

	// compute new features for current frame
	computeFeatures(I1c, dims_c, m1c1, n1c1, m1c2, n1c2, I1c_du, I1c_dv, I1c_du_full, I1c_dv_full);
	if (I2 != 0)
		computeFeatures(I2c, dims_c, m2c1, n2c1, m2c2, n2c2, I2c_du, I2c_dv, I2c_du_full, I2c_dv_full);
	
}

void Matcher::matchFeatures()
{
	p_matched_1.clear();
	p_matched_2.clear();

	if (m1p2 == 0 || n1p2 == 0 || m2p2 == 0 || n2p2 == 0 || m1c2 == 0 || n1c2 == 0 || m2c2 == 0 || n2c2 == 0)
		return;
	if (param.multi_stage)
		if (m1p1 == 0 || n1p1 == 0 || m2p1 == 0 || n2p1 == 0 || m1c1 == 0 || n1c1 == 0 || m2c1 == 0 || n2c1 == 0)
			return;

	if (param.multi_stage)
	{
		matching(m1p1, m2p1, m1c1, m2c1, n1p1, n2p1, n1c1, n2c1, p_matched_1, false);
		if (param.remove_outliers)
			removeOutliers(p_matched_1);

		computePriorStatistics(p_matched_1);

		matching(m1p2, m2p2, m1c2, m2c2, n1p2, n2p2, n1c2, n2c2, p_matched_2, true);
		if (param.refinement > 0)
			refinement(p_matched_2);
		if (param.remove_outliers)
			removeOutliers(p_matched_2);
	}
	else
	{
		matching(m1p2, m2p2, m1c2, m2c2, n1p2, n2p2, n1c2, n2c2, p_matched_2, false);
		if (param.refinement > 0)
			refinement(p_matched_2);

		if (param.remove_outliers)
			removeOutliers(p_matched_2);
	}
}

void Matcher::bucketFeatures(int32_t max_features, float bucket_width, float bucket_height)
{

	// find max values
	float u_max = 0;
	float u_min = 1e5;
	float v_max = 0;
	float v_min = 1e5;
	for (auto it :  p_matched_2)
	{
		if (it.u1c > u_max)
			u_max = it.u1c;             // maximum u,v feature position in current left image
		if (it.u1c < u_min)
			u_min = it.u1c;

		if (it.v1c > v_max)
			v_max = it.v1c;
		if (it.v1c < v_min)
			v_min = it.v1c;
	}

	// allocate number of buckets needed
	int32_t bucket_cols = (int32_t)floor( (u_max-u_min) / bucket_width) + 1;        // number of bucket in cols to cover all matched features
	int32_t bucket_rows = (int32_t)floor( (v_max-v_min) / bucket_height) + 1;	   // number of bucket in rows to cover all matched features
	vector<p_match> *buckets = new vector<p_match>[bucket_cols*bucket_rows];	// total number of bucket stored in a vector

	// assign matches to their buckets
	for (auto it : p_matched_2)
	{
		int32_t u = (int32_t)floor( (it.u1c - u_min ) / bucket_width);                // extract u,v coordinates of matched features and find their buckets
		int32_t v = (int32_t)floor( (it.v1c - v_min ) / bucket_height);
		buckets[v*bucket_cols + u].push_back(it);							// store matched features <p_macth> in corresponding buckets
	}

	// refill p_matched from buckets
	p_matched_2.clear();													// clear and prepare for assignments from buckets

	for (int32_t i = 0; i < bucket_cols*bucket_rows; i++)
	{
		// shuffle bucket indices randomly
		//std::random_shuffle(buckets[i].begin(), buckets[i].end());

		// add up to max_features features from this bucket to p_matched
		int32_t k = 0;
		for (vector<p_match>::iterator it = buckets[i].begin(); it != buckets[i].end(); it++)
		{
			p_matched_2.push_back(*it);
			k++;
			if (k >= max_features)                        // by default, max_features = 2 (in viso.h); randomly store 2 matched features in each bucket, pushback to p_matched_2;
				break;
		}
	}

	// free buckets
	delete[]buckets;
}

float Matcher::getGain(vector<int32_t> inliers)
{

	// check if two images are provided and matched
	if (I1p == 0 || I1c == 0 || p_matched_2.size() == 0 || inliers.size() == 0)
		return 1;

	int32_t window_size = 3;
	float   gain = 0;
	int32_t num = 0;
	int32_t u_min, u_max, v_min, v_max;
	float   mean_prev, mean_curr;

	for (vector<int32_t>::iterator it = inliers.begin(); it != inliers.end(); it++)
	{
		if (*it < (int32_t)p_matched_2.size())
		{
			// mean in previous image
			u_min = min(max((int32_t)p_matched_2[*it].u1p - window_size, 0), dims_p[0]);
			u_max = min(max((int32_t)p_matched_2[*it].u1p + window_size, 0), dims_p[0]);
			v_min = min(max((int32_t)p_matched_2[*it].v1p - window_size, 0), dims_p[1]);
			v_max = min(max((int32_t)p_matched_2[*it].v1p + window_size, 0), dims_p[1]);
			mean_prev = mean(I1p, dims_p[2], u_min, u_max, v_min, v_max);

			// mean in current image
			u_min = min(max((int32_t)p_matched_2[*it].u1c - window_size, 0), dims_p[0]);
			u_max = min(max((int32_t)p_matched_2[*it].u1c + window_size, 0), dims_p[0]);
			v_min = min(max((int32_t)p_matched_2[*it].v1c - window_size, 0), dims_p[1]);
			v_max = min(max((int32_t)p_matched_2[*it].v1c + window_size, 0), dims_p[1]);
			mean_curr = mean(I1c, dims_c[2], u_min, u_max, v_min, v_max);

			if (mean_prev > 10)
			{
				gain += mean_curr / mean_prev;
				num++;
			}
		}
	}

	if (num > 0) return gain /= (float)num;
	else       return 1;
}

inline void Matcher::computeDescriptor(const uint8_t* I_du, const uint8_t* I_dv, const int32_t &bpl, const int32_t &u, const int32_t &v, uint8_t *desc_addr)
{

	// get address indices
	int32_t addr_m1 = getAddressOffsetImage(u, v - 1, bpl);
	int32_t addr_m3 = addr_m1 - 2 * bpl;
	int32_t addr_m5 = addr_m3 - 2 * bpl;
	int32_t addr_p1 = addr_m1 + 2 * bpl;
	int32_t addr_p3 = addr_p1 + 2 * bpl;
	int32_t addr_p5 = addr_p3 + 2 * bpl;

	// compute descriptor
	uint32_t k = 0;
	desc_addr[k++] = I_du[addr_m1 - 3];
	desc_addr[k++] = I_dv[addr_m1 - 3];
	desc_addr[k++] = I_du[addr_p1 - 3];
	desc_addr[k++] = I_dv[addr_p1 - 3];
	desc_addr[k++] = I_du[addr_m1 - 1];
	desc_addr[k++] = I_dv[addr_m1 - 1];
	desc_addr[k++] = I_du[addr_p1 - 1];
	desc_addr[k++] = I_dv[addr_p1 - 1];
	desc_addr[k++] = I_du[addr_m1 + 3];
	desc_addr[k++] = I_dv[addr_m1 + 3];
	desc_addr[k++] = I_du[addr_p1 + 3];
	desc_addr[k++] = I_dv[addr_p1 + 3];
	desc_addr[k++] = I_du[addr_m1 + 1];
	desc_addr[k++] = I_dv[addr_m1 + 1];
	desc_addr[k++] = I_du[addr_p1 + 1];
	desc_addr[k++] = I_dv[addr_p1 + 1];
	desc_addr[k++] = I_du[addr_m5 - 1];
	desc_addr[k++] = I_dv[addr_m5 - 1];
	desc_addr[k++] = I_du[addr_p5 - 1];
	desc_addr[k++] = I_dv[addr_p5 - 1];
	desc_addr[k++] = I_du[addr_m5 + 1];
	desc_addr[k++] = I_dv[addr_m5 + 1];
	desc_addr[k++] = I_du[addr_p5 + 1];
	desc_addr[k++] = I_dv[addr_p5 + 1];
	desc_addr[k++] = I_du[addr_m3 - 5];
	desc_addr[k++] = I_dv[addr_m3 - 5];
	desc_addr[k++] = I_du[addr_p3 - 5];
	desc_addr[k++] = I_dv[addr_p3 - 5];
	desc_addr[k++] = I_du[addr_m3 + 5];
	desc_addr[k++] = I_dv[addr_m3 + 5];
	desc_addr[k++] = I_du[addr_p3 + 5];
	desc_addr[k++] = I_dv[addr_p3 + 5];
}

inline void Matcher::computeSmallDescriptor(const uint8_t* I_du, const uint8_t* I_dv, const int32_t &bpl, const int32_t &u, const int32_t &v, uint8_t *desc_addr)
{

	// get address indices
	int32_t addr2 = getAddressOffsetImage(u, v, bpl);
	int32_t addr1 = addr2 - bpl;
	int32_t addr0 = addr1 - bpl;
	int32_t addr3 = addr2 + bpl;
	int32_t addr4 = addr3 + bpl;

	// compute ELAS-descriptor
	uint32_t k = 0;
	desc_addr[k++] = I_du[addr0];
	desc_addr[k++] = I_du[addr1 - 2];
	desc_addr[k++] = I_du[addr1];
	desc_addr[k++] = I_du[addr1 + 2];
	desc_addr[k++] = I_du[addr2 - 1];
	desc_addr[k++] = I_du[addr2];
	desc_addr[k++] = I_du[addr2];
	desc_addr[k++] = I_du[addr2 + 1];
	desc_addr[k++] = I_du[addr3 - 2];
	desc_addr[k++] = I_du[addr3];
	desc_addr[k++] = I_du[addr3 + 2];
	desc_addr[k++] = I_du[addr4];
	desc_addr[k++] = I_dv[addr1];
	desc_addr[k++] = I_dv[addr2 - 1];
	desc_addr[k++] = I_dv[addr2 + 1];
	desc_addr[k++] = I_dv[addr3];
}

void Matcher::computeDescriptors(uint8_t* I_du, uint8_t* I_dv, const int32_t bpl, std::vector<Matcher::maximum> &maxima)
{

	// loop variables
	int32_t u, v;
	uint8_t *desc_addr;

	// for all maxima do
	for (vector<Matcher::maximum>::iterator it = maxima.begin(); it != maxima.end(); it++)
	{
		u = (*it).u;
		v = (*it).v;
		desc_addr = (uint8_t*)(&((*it).d1));
		computeDescriptor(I_du, I_dv, bpl, u, v, desc_addr);
		// computeSmallDescriptor(I_du, I_dv, bpl, u, v, desc_addr);
	}
}


void Matcher::getHalfResolutionDimensions(const int32_t *dims, int32_t *dims_half)
{
	dims_half[0] = dims[0] / 2;
	dims_half[1] = dims[1] / 2;
	dims_half[2] = dims_half[0] + 15 - (dims_half[0] - 1) % 16;
}

uint8_t* Matcher::createHalfResolutionImage(uint8_t *I, const int32_t* dims)
{
	int32_t dims_half[3];
	getHalfResolutionDimensions(dims, dims_half);      // dims_half[i] = dims[i]/2;
	uint8_t* I_half = (uint8_t*)_mm_malloc(dims_half[2] * dims_half[1] * sizeof(uint8_t), 16);   // pixel vals of half_resolution image stored in a vector I_half[];
	for (int32_t v = 0; v < dims_half[1]; v++)
		for (int32_t u = 0; u < dims_half[0]; u++)
			I_half[v*dims_half[2] + u] = (uint8_t)(((int32_t)I[(v * 2 + 0)*dims[2] + u * 2 + 0] +
			(int32_t)I[(v * 2 + 0)*dims[2] + u * 2 + 1] +
			(int32_t)I[(v * 2 + 1)*dims[2] + u * 2 + 0] +
			(int32_t)I[(v * 2 + 1)*dims[2] + u * 2 + 1]) / 4);                         // find half_resolution image, val = (I(itself)+I(right neighbor)+I(neighbor below)+I(neighbor right below))/4;

	return I_half;
}

void Matcher::computeFeatures(uint8_t *I, const int32_t* dims, int32_t* &max1, int32_t &num1, int32_t* &max2, int32_t &num2, uint8_t* &I_du, uint8_t* &I_dv, uint8_t* &I_du_full, uint8_t* &I_dv_full)
{
	vector<Matcher::maximum> maxima1;
	vector<Matcher::maximum> maxima2;

	if (param.half_resolution)
	{
		int32_t dims_matching[3];
		memcpy(dims_matching, dims, 3 * sizeof(int32_t));
		uint8_t* I_matching = createHalfResolutionImage(I, dims);
		getHalfResolutionDimensions(dims, dims_matching);

		I_du = (uint8_t*)_mm_malloc(dims_matching[2] * dims_matching[1] * sizeof(uint8_t*), 16);
		I_dv = (uint8_t*)_mm_malloc(dims_matching[2] * dims_matching[1] * sizeof(uint8_t*), 16);
		I_du_full = (uint8_t*)_mm_malloc(dims[2] * dims[1] * sizeof(uint8_t*), 16);
		I_dv_full = (uint8_t*)_mm_malloc(dims[2] * dims[1] * sizeof(uint8_t*), 16);

		filter::sobel5x5(I_matching, I_du, I_dv, dims_matching[2], dims_matching[1]);
		filter::sobel5x5(I, I_du_full, I_dv_full, dims[2], dims[1]);

		if (param.multi_stage)
		{
			FastFeatures(I_matching, dims_matching, maxima1, param.fast_threshold_sparse, param.numFastFeature_sparse);
			FastFeatures(I_matching, dims_matching, maxima2, param.fast_threshold_dense, param.numFastFeature_dense);
			computeDescriptors(I_du, I_dv, dims_matching[2], maxima1);
			computeDescriptors(I_du, I_dv, dims_matching[2], maxima2);
		}
		else
		{
			FastFeatures(I_matching, dims_matching, maxima2, param.fast_threshold_dense, param.numFastFeature_dense);
			computeDescriptors(I_du, I_dv, dims_matching[2], maxima2);
		}
		_mm_free(I_matching);
	}
	else
	{
		I_du = (uint8_t*)_mm_malloc(dims[2] * dims[1] * sizeof(uint8_t*), 16);
		I_dv = (uint8_t*)_mm_malloc(dims[2] * dims[1] * sizeof(uint8_t*), 16);
	
		filter::sobel5x5(I, I_du, I_dv, dims[2], dims[1]);
		if (param.multi_stage)
		{
			FastFeatures(I, dims, maxima1, param.fast_threshold_sparse, param.numFastFeature_sparse);
			FastFeatures(I, dims, maxima2, param.fast_threshold_dense, param.numFastFeature_dense);
			computeDescriptors(I_du, I_dv, dims[2], maxima1);
			computeDescriptors(I_du, I_dv, dims[2], maxima2);
		}
		else
		{
			FastFeatures(I, dims, maxima2, param.fast_threshold_dense, param.numFastFeature_dense);
			computeDescriptors(I_du, I_dv, dims[2], maxima2);
			// computeDescriptors2(lbp, I_du, I_dv, dims[2], maxima2);
		}
	}

	num1 = maxima1.size();
	num2 = maxima2.size();
	max1 = 0;
	max2 = 0;

	if (num1 != 0)
	{
		max1 = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num1, 16);
		int32_t k = 0;
		for (vector<Matcher::maximum>::iterator it = maxima1.begin(); it != maxima1.end(); it++)
		{
			// if (flow.data != 0 && fabs(flow.at<float>(it->v, it->v)) < 2) continue;
			*(max1 + k++) = it->u;  *(max1 + k++) = it->v;  *(max1 + k++) = it->val;        *(max1 + k++) = 0;
			*(max1 + k++) = it->d1;   *(max1 + k++) = it->d2;   *(max1 + k++) = it->d3;   *(max1 + k++) = it->d4;
			*(max1 + k++) = it->d5;   *(max1 + k++) = it->d6;   *(max1 + k++) = it->d7;   *(max1 + k++) = it->d8;
		}
	}

	if (num2 != 0)
	{
		max2 = (int32_t*)_mm_malloc(sizeof(Matcher::maximum)*num2, 16);
		int32_t k = 0;
		for (vector<Matcher::maximum>::iterator it = maxima2.begin(); it != maxima2.end(); it++)
		{
			// if (flow.data != 0 && (flow.at<Vec2b>(it->v, it->v)[0] * flow.at<Vec2b>(it->v, it->v)[0] + flow.at<Vec2b>(it->v, it->v)[1] * flow.at<Vec2b>(it->v, it->v)[1]) < 8) continue;
			*(max2 + k++) = it->u;  *(max2 + k++) = it->v;  *(max2 + k++) = it->val;        *(max2 + k++) = 0;
			*(max2 + k++) = it->d1;   *(max2 + k++) = it->d2;   *(max2 + k++) = it->d3;   *(max2 + k++) = it->d4;
			*(max2 + k++) = it->d5;   *(max2 + k++) = it->d6;   *(max2 + k++) = it->d7;   *(max2 + k++) = it->d8;
		}
	}

}

void Matcher::computePriorStatistics(vector<Matcher::p_match> &p_matched)
{

	// compute number of bins
	int32_t u_bin_num = (int32_t)ceil((float)dims_c[0] / (float)param.match_binsize);   //default binsize = 50
	int32_t v_bin_num = (int32_t)ceil((float)dims_c[1] / (float)param.match_binsize);
	int32_t bin_num = v_bin_num*u_bin_num;

	// number of matching stages
	int32_t num_stages = 4;

	// allocate bin accumulator memory
	vector<Matcher::delta> *delta_accu = new vector<Matcher::delta>[bin_num];

	// fill bin accumulator
	Matcher::delta delta_curr;
	for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it != p_matched.end(); it++)
	{


		delta_curr.val[0] = it->u2p - it->u1p;		// stereo only u_dis		(previous left to previous right)
		delta_curr.val[1] = 0;
		delta_curr.val[2] = it->u2c - it->u2p;		// flow, u_dis and v_dis	(previous right to current right)
		delta_curr.val[3] = it->v2c - it->v2p;
		delta_curr.val[4] = it->u1c - it->u2c;		// stereo					(current right to current left)
		delta_curr.val[5] = 0;
		delta_curr.val[6] = it->u1p - it->u1c;		// flow						(current left to previous left)
		delta_curr.val[7] = it->v1p - it->v1c;


		// compute row and column of bin to which this observation belongs
		int32_t u_bin_min, u_bin_max, v_bin_min, v_bin_max;


		u_bin_min = min(max((int32_t)floor(it->u1p / (float)param.match_binsize) - 1, 0), u_bin_num - 1);
		u_bin_max = min(max((int32_t)floor(it->u1p / (float)param.match_binsize) + 1, 0), u_bin_num - 1);
		v_bin_min = min(max((int32_t)floor(it->v1p / (float)param.match_binsize) - 1, 0), v_bin_num - 1);
		v_bin_max = min(max((int32_t)floor(it->v1p / (float)param.match_binsize) + 1, 0), v_bin_num - 1);


		// add to accumulator
		for (int32_t v_bin = v_bin_min; v_bin <= v_bin_max; v_bin++)
			for (int32_t u_bin = u_bin_min; u_bin <= u_bin_max; u_bin++)
				delta_accu[v_bin*u_bin_num + u_bin].push_back(delta_curr);
	}

	// clear ranges
	ranges.clear();

	// for all bins compute statistics
	for (int32_t v_bin = 0; v_bin < v_bin_num; v_bin++)
	{
		for (int32_t u_bin = 0; u_bin < u_bin_num; u_bin++)
		{
			// use full range in case there are no observations
			delta delta_min(-param.match_radius);       // for all values in deltra struct, set to -100
			delta delta_max(+param.match_radius);		// for all values in deltra struct, set to 100

			// otherwise determine delta min and delta max
			if (delta_accu[v_bin*u_bin_num + u_bin].size() > 0)
			{

				// init displacements 'delta' to 'infinite'
				delta_min = delta(+1000000);
				delta_max = delta(-1000000);

				// find minimum and maximum displacements
				for (vector<Matcher::delta>::iterator it = delta_accu[v_bin*u_bin_num + u_bin].begin();
					it != delta_accu[v_bin*u_bin_num + u_bin].end(); it++)
				{
					for (int32_t i = 0; i < num_stages * 2; i++)
					{
						if (it->val[i] < delta_min.val[i]) delta_min.val[i] = it->val[i];
						if (it->val[i] > delta_max.val[i]) delta_max.val[i] = it->val[i];
					}
				}
			}

			// set search range for this bin
			range r;
			for (int32_t i = 0; i < num_stages; i++)
			{

				// bound minimum search range to 20x20
				float delta_u = delta_max.val[i * 2 + 0] - delta_min.val[i * 2 + 0];
				if (delta_u < 20)
				{
					delta_min.val[i * 2 + 0] -= ceil((20 - delta_u) / 2);
					delta_max.val[i * 2 + 0] += ceil((20 - delta_u) / 2);
				}
				float delta_v = delta_max.val[i * 2 + 1] - delta_min.val[i * 2 + 1];
				if (delta_v < 20)
				{
					delta_min.val[i * 2 + 1] -= ceil((20 - delta_v) / 2);
					delta_max.val[i * 2 + 1] += ceil((20 - delta_v) / 2);
				}

				// set range for this bin
				r.u_min[i] = delta_min.val[i * 2 + 0];
				r.u_max[i] = delta_max.val[i * 2 + 0];
				r.v_min[i] = delta_min.val[i * 2 + 1];
				r.v_max[i] = delta_max.val[i * 2 + 1];
			}
			ranges.push_back(r);
		}
	}

	// free bin accumulator memory
	delete[]delta_accu;
}

// find each feature maxima belonging to which bin and save to vector k
void Matcher::createIndexVector(int32_t* m, int32_t n, vector<int32_t> *k, const int32_t &u_bin_num, const int32_t &v_bin_num)
{
	// descriptor step size
	int32_t step_size = sizeof(Matcher::maximum) / sizeof(int32_t);   //size of the maximum struct

	// vector<int32_t> *k2c = new vector<int32_t>[u_bin_num*v_bin_num];

	// for all points do
	for (int32_t i = 0; i < n; i++)
	{
		// extract coordinates and class from maxima struct
		int32_t u = *(m + step_size*i + 0); // u-coordinate
		int32_t v = *(m + step_size*i + 1); // v-coordinate
		int32_t c = *(m + step_size*i + 3); // HM: used as the marker for previous matched quad points

		// compute row and column of bin to which this observation belongs
		int32_t u_bin = min((int32_t)floor((float)u / (float)param.match_binsize), u_bin_num - 1);
		int32_t v_bin = min((int32_t)floor((float)v / (float)param.match_binsize), v_bin_num - 1);

		// save index
		if (c) // it is a good point
			k[v_bin*u_bin_num + u_bin].insert(k[v_bin*u_bin_num + u_bin].begin(),i);  // good points have priority in matching
		else
			k[v_bin*u_bin_num + u_bin].push_back(i);
	}
}


// findMatch is already in a for loop, for one feature in one frame to find best matches in other frames within a window, and i1 is the increasing index
inline void Matcher::findMatch(int32_t* m1, const int32_t &i1, int32_t* m2, const int32_t &step_size, vector<int32_t> *k2,
	const int32_t &u_bin_num, const int32_t &v_bin_num, const int32_t &stat_bin,
	int32_t& min_ind, int32_t stage, bool flow, bool use_prior, double u_, double v_)
{
	// init and load image coordinates + feature
	min_ind = -1;
	double  min_cost = 10000000;
	// u,v,c stored in maxima struct
	int32_t u1 = *(m1 + step_size*i1 + 0);
	int32_t v1 = *(m1 + step_size*i1 + 1);

	__m128i xmm1 = _mm_load_si128((__m128i*)(m1 + step_size*i1 + 4));
	__m128i xmm2 = _mm_load_si128((__m128i*)(m1 + step_size*i1 + 8));

	float u_min, u_max, v_min, v_max;

	// restrict search range with prior
	if (use_prior)      // dense matching --> use_prior = true
	{
		// for each feature, it has its own stat_bin 
		u_min = u1 + ranges[stat_bin].u_min[stage];
		u_max = u1 + ranges[stat_bin].u_max[stage];
		v_min = v1 + ranges[stat_bin].v_min[stage];
		v_max = v1 + ranges[stat_bin].v_max[stage];
		// smaller area for searching

	}
	else                // sparse matching
	{
		u_min = u1 - param.match_radius;          // match_radius = 100;
		u_max = u1 + param.match_radius;
		v_min = v1 - param.match_radius;
		v_max = v1 + param.match_radius;
	}

	// if stereo search => constrain to 1d  // if not flow, search area in vertical direction should be small 
	if (!flow)
	{
		v_min = v1 - param.match_disp_tolerance;
		v_max = v1 + param.match_disp_tolerance;
	}

	// bins of interest // for this feature, which bins are searched (blocks of bins not circle).
	int32_t u_bin_min = min(max((int32_t)floor(u_min / (float)param.match_binsize), 0), u_bin_num - 1);
	int32_t u_bin_max = min(max((int32_t)floor(u_max / (float)param.match_binsize), 0), u_bin_num - 1);
	int32_t v_bin_min = min(max((int32_t)floor(v_min / (float)param.match_binsize), 0), v_bin_num - 1);
	int32_t v_bin_max = min(max((int32_t)floor(v_max / (float)param.match_binsize), 0), v_bin_num - 1);

	// for all bins of interest do
	for (int32_t u_bin = u_bin_min; u_bin <= u_bin_max; u_bin++)       // start from u_bin_min
	{
		for (int32_t v_bin = v_bin_min; v_bin <= v_bin_max; v_bin++)	// start from v_bin_min
		{
			int32_t k2_ind = (v_bin)*u_bin_num + u_bin;  // find the index of this feature in *k
			for (vector<int32_t>::const_iterator i2_it = k2[k2_ind].begin(); i2_it != k2[k2_ind].end(); i2_it++)  // for all features index in each bin of interests  
			{
				int32_t u2 = *(m2 + step_size*(*i2_it) + 0);													// track the index and extract u,v of features in the other image
				int32_t v2 = *(m2 + step_size*(*i2_it) + 1);
				int32_t c2 = *(m2 + step_size*(*i2_it) + 3); // HM: give priority to matched points

				if (u2 >= u_min && u2 <= u_max && v2 >= v_min && v2 <= v_max)
				{
					__m128i xmm3 = _mm_load_si128((__m128i*)(m2 + step_size*(*i2_it) + 4));
					__m128i xmm4 = _mm_load_si128((__m128i*)(m2 + step_size*(*i2_it) + 8));
					xmm3 = _mm_sad_epu8(xmm1, xmm3);                                                    // Sum of Absolute Differences
					xmm4 = _mm_sad_epu8(xmm2, xmm4);
					xmm4 = _mm_add_epi16(xmm3, xmm4);
					double cost = (double)(_mm_extract_epi16(xmm4, 0) + _mm_extract_epi16(xmm4, 4));

					if (u_ >= 0 && v_ >= 0)      // if Tr_delta --> 3D project estimation
					{
						double du = (double)u2 - u_;                  // difference approximation
						double dv = (double)v2 - v_;
						double dist = sqrt(du*du + dv*dv);
						cost += 4 * dist;
					}

					//HM: give priority to matched points
					if (c2)
						cost = cost * 0.95;

					if (cost < min_cost)
					{
						min_ind = *i2_it;					// keep updating the best match feature index
						min_cost = cost;                    // keep updating min_cost. default = 10000000
					}
				}
			}
		}
	}
}

void Matcher::matching(int32_t *m1p, int32_t *m2p, int32_t *m1c, int32_t *m2c,
	int32_t n1p, int32_t n2p, int32_t n1c, int32_t n2c,
	vector<Matcher::p_match> &p_matched, bool use_prior)
{
	// descriptor step size (number of int32_t elements in struct)
	int32_t step_size = sizeof(Matcher::maximum) / sizeof(int32_t);                 // in struct maximum, total 12 int32_t variables, so step_size = 12; 

	// compute number of bins
	int32_t u_bin_num = (int32_t)ceil((float)dims_c[0] / (float)param.match_binsize);     // match_binsize = 50;
	int32_t v_bin_num = (int32_t)ceil((float)dims_c[1] / (float)param.match_binsize);
	int32_t bin_num = v_bin_num*u_bin_num;

	// allocate memory for index vectors (needed for efficient search)
	vector<int32_t> *k1p = new vector<int32_t>[bin_num];                     // to store all four classes features for each image
	vector<int32_t> *k2p = new vector<int32_t>[bin_num];
	vector<int32_t> *k1c = new vector<int32_t>[bin_num];
	vector<int32_t> *k2c = new vector<int32_t>[bin_num];

	vector<int32_t> matched_bin(bin_num);

	// loop variables
	int32_t* M = (int32_t*)calloc(dims_c[0] * dims_c[1], sizeof(int32_t));                  // prepare for p_match
	int32_t i1p=0, i2p=0, i1c=0, i2c=0, i1p2=0;
	int32_t u1p, v1p, u2p, v2p, u1c, v1c, u2c, v2c;

	// create position/class bin index vectors (position/class of features are stored in m1p1,m1p2,m2p1,m2p2,m1c1,m1c2,m2c1,m2c2);
	createIndexVector(m1p, n1p, k1p, u_bin_num, v_bin_num);  // for previous left features
	createIndexVector(m2p, n2p, k2p, u_bin_num, v_bin_num);	 // for previous right features
	createIndexVector(m1c, n1c, k1c, u_bin_num, v_bin_num);  // for current left features
	createIndexVector(m2c, n2c, k2c, u_bin_num, v_bin_num);  // for current right feayures


	_rematched = 0;


	// for all points do
	for (i1p = 0; i1p < n1p; i1p++)     // i1p: feature index
	{

		// coordinates
		u1p = *(m1p + step_size*i1p + 0);
		v1p = *(m1p + step_size*i1p + 1);
		int32_t c1p = *(m1p + step_size*i1p + 3); // test if this point is matched before

		// compute row and column of statistics bin to which this observation belongs
		int32_t u_bin = min((int32_t)floor((float)u1p / (float)param.match_binsize), u_bin_num - 1);
		int32_t v_bin = min((int32_t)floor((float)v1p / (float)param.match_binsize), v_bin_num - 1);
		int32_t stat_bin = v_bin*u_bin_num + u_bin;

		if (matched_bin[stat_bin] >= 5 && !c1p)
		{
			//std::cout << i1p << " exceeded bin quota 2" << std::endl;
			continue;
		}
			

		// match in circle
		findMatch(m1p, i1p, m2p, step_size, k2p, u_bin_num, v_bin_num, stat_bin, i2p, 0, false, use_prior);       // use_prior == false; based on previous left features, find best match in previous right// -->i2p

		u2p = *(m2p + step_size*i2p + 0);
		v2p = *(m2p + step_size*i2p + 1);

		findMatch(m2p, i2p, m2c, step_size, k2c, u_bin_num, v_bin_num, stat_bin, i2c, 1, true, use_prior); // based on previous right features, find best match in current right   // --> i2c

		findMatch(m2c, i2c, m1c, step_size, k1c, u_bin_num, v_bin_num, stat_bin, i1c, 2, false, use_prior); // based on current right features, find best match in current left			// --> i1c

		findMatch(m1c, i1c, m1p, step_size, k1p, u_bin_num, v_bin_num, stat_bin, i1p2, 3, true, use_prior);
		// based on current left features, find best match in previous left  // --> i1p2 (i1p is used in outer for loop)

		if (i2p < 0 || i2c < 0 || i1c < 0 || i1p2 < 0 )
			continue;


		if (i1p2 != i1p)
		{

			int32_t u1p2 = *(m1p + step_size*i1p2 + 0);
			int32_t v1p2 = *(m1p + step_size*i1p2 + 1);

			if ( fabs( u1p2 - u1p) >  2 || fabs( v1p2 - v1p) > 2) // the two points does not match
				continue;
			// else
			// 	cout <<  "non-exact match" << i1p2 << endl;
		}else // exact matching!
		{
			// Marking the closed loop features as good features
			int32_t* c2c = (m2c + step_size*i2c + 3);   // current right image
			int32_t* c1c = (m1c + step_size*i1c + 3);	// current left image
			*c2c = i1p;
			*c1c = i1p;
			_rematched++;
		}
		// circle closure success?
		// i1p2 == i1p if last feature coincides with first feature


		// extract coordinates of matched feature
		u2c = *(m2c + step_size*i2c + 0); v2c = *(m2c + step_size*i2c + 1);     // current right image
		u1c = *(m1c + step_size*i1c + 0); v1c = *(m1c + step_size*i1c + 1);		// current left image

		// if disparities are positive
		if (u1p >= u2p && u1c >= u2c)    // 
		{
			// add all four images' matched feaures position and index 
			p_matched.push_back(Matcher::p_match(u1p, v1p, i1p, u2p, v2p, i2p,
				u1c, v1c, i1c, u2c, v2c, i2c));

			matched_bin[stat_bin]++;
		}

	}
	
	// free memory
	free(M);
	delete[]k1p;
	delete[]k2p;
	delete[]k1c;
	delete[]k2c;
}

void Matcher::removeOutliers(vector<Matcher::p_match> &p_matched)
{

	// do we have enough points for outlier removal?
	if (p_matched.size() <= 3)
		return;

	// input/output structure for triangulation
	struct triangulateio in, out;

	// inputs
	in.numberofpoints = p_matched.size();
	in.pointlist = (float*)malloc(in.numberofpoints * 2 * sizeof(float));
	int32_t k = 0;

	// create copy of p_matched, init vector with number of support points
	// and fill triangle point vector for delaunay triangulation
	vector<Matcher::p_match> p_matched_copy;
	vector<int32_t> num_support;
	for (vector<Matcher::p_match>::iterator it = p_matched.begin(); it != p_matched.end(); it++)
	{
		p_matched_copy.push_back(*it);
		num_support.push_back(0);
		in.pointlist[k++] = it->u1c;
		in.pointlist[k++] = it->v1c;
	}

	// input parameters
	in.numberofpointattributes = 0;
	in.pointattributelist = NULL;
	in.pointmarkerlist = NULL;
	in.numberofsegments = 0;
	in.numberofholes = 0;
	in.numberofregions = 0;
	in.regionlist = NULL;

	// outputs
	out.pointlist = NULL;
	out.pointattributelist = NULL;
	out.pointmarkerlist = NULL;
	out.trianglelist = NULL;
	out.triangleattributelist = NULL;
	out.neighborlist = NULL;
	out.segmentlist = NULL;
	out.segmentmarkerlist = NULL;
	out.edgelist = NULL;
	out.edgemarkerlist = NULL;

	// do triangulation (z=zero-based, n=neighbors, Q=quiet, B=no boundary markers)
	// attention: not using the B switch or using the n switch creates a memory leak (=> use valgrind!)
	char parameters[] = "zQB";
	triangulate(parameters, &in, &out, NULL);

	// for all triangles do
	for (int32_t i = 0; i < out.numberoftriangles; i++)
	{

		// extract triangle corner points
		int32_t p1 = out.trianglelist[i * 3 + 0];
		int32_t p2 = out.trianglelist[i * 3 + 1];
		int32_t p3 = out.trianglelist[i * 3 + 2];


		// method: quad matching

		// 1. corner disparity and flow
		float p1_flow_u = p_matched_copy[p1].u1c - p_matched_copy[p1].u1p;
		float p1_flow_v = p_matched_copy[p1].v1c - p_matched_copy[p1].v1p;
		float p1_disp = p_matched_copy[p1].u1p - p_matched_copy[p1].u2p;

		// 2. corner disparity and flow
		float p2_flow_u = p_matched_copy[p2].u1c - p_matched_copy[p2].u1p;
		float p2_flow_v = p_matched_copy[p2].v1c - p_matched_copy[p2].v1p;
		float p2_disp = p_matched_copy[p2].u1p - p_matched_copy[p2].u2p;

		// 3. corner disparity and flow
		float p3_flow_u = p_matched_copy[p3].u1c - p_matched_copy[p3].u1p;
		float p3_flow_v = p_matched_copy[p3].v1c - p_matched_copy[p3].v1p;
		float p3_disp = p_matched_copy[p3].u1p - p_matched_copy[p3].u2p;

		// consistency of 1. edge
		if (fabs(p1_disp - p2_disp) < param.outlier_disp_tolerance && fabs(p1_flow_u - p2_flow_u) + fabs(p1_flow_v - p2_flow_v) < param.outlier_flow_tolerance)
		{
			num_support[p1]++;
			num_support[p2]++;
		}

		// consistency of 2. edge
		if (fabs(p2_disp - p3_disp) < param.outlier_disp_tolerance && fabs(p2_flow_u - p3_flow_u) + fabs(p2_flow_v - p3_flow_v) < param.outlier_flow_tolerance)
		{
			num_support[p2]++;
			num_support[p3]++;
		}

		// consistency of 3. edge
		if (fabs(p1_disp - p3_disp) < param.outlier_disp_tolerance && fabs(p1_flow_u - p3_flow_u) + fabs(p1_flow_v - p3_flow_v) < param.outlier_flow_tolerance)
		{
			num_support[p1]++;
			num_support[p3]++;
		}
	}


	// refill p_matched
	p_matched.clear();
	for (int i = 0; i < in.numberofpoints; i++)
		if (num_support[i] >= 4)
			p_matched.push_back(p_matched_copy[i]);

	// free memory used for triangulation
	free(in.pointlist);
	free(out.pointlist);
	free(out.trianglelist);
}

bool Matcher::parabolicFitting(const uint8_t* I1_du, const uint8_t* I1_dv, const int32_t* dims1,
	const uint8_t* I2_du, const uint8_t* I2_dv, const int32_t* dims2,
	const float &u1, const float &v1,
	float       &u2, float       &v2,
	Matrix At, Matrix AtA,
	uint8_t* desc_buffer) {

	// check if parabolic fitting is feasible (descriptors are within margin)
	if (u2 - 3 < margin || u2 + 3 > dims2[0] - 1 - margin || v2 - 3 < margin || v2 + 3 > dims2[1] - 1 - margin)
		return false;

	// compute reference descriptor
	__m128i xmm1, xmm2;
	computeSmallDescriptor(I1_du, I1_dv, dims1[2], (int32_t)u1, (int32_t)v1, desc_buffer);
	xmm1 = _mm_load_si128((__m128i*)(desc_buffer));

	// compute cost matrix
	int32_t cost[49];
	for (int32_t dv = 0; dv < 7; dv++) {
		for (int32_t du = 0; du < 7; du++) {
			computeSmallDescriptor(I2_du, I2_dv, dims2[2], (int32_t)u2 + du - 3, (int32_t)v2 + dv - 3, desc_buffer);
			xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
			xmm2 = _mm_sad_epu8(xmm1, xmm2);
			cost[dv * 7 + du] = _mm_extract_epi16(xmm2, 0) + _mm_extract_epi16(xmm2, 4);
		}
	}

	// compute minimum
	int32_t min_ind = 0;
	int32_t min_cost = cost[0];
	for (int32_t i = 1; i < 49; i++) {
		if (cost[i] < min_cost) {
			min_ind = i;
			min_cost = cost[i];
		}
	}

	// get indices
	int32_t du = min_ind % 7;
	int32_t dv = min_ind / 7;

	// if minimum is at borders => remove this match
	if (du == 0 || du == 6 || dv == 0 || dv == 6)
		return false;

	// solve least squares system
	Matrix c(9, 1);
	for (int32_t i = -1; i <= +1; i++) {
		for (int32_t j = -1; j <= +1; j++) {
			int32_t cost_curr = cost[(dv + i) * 7 + (du + j)];
			// if (i!=0 && j!=0 && cost_curr<=min_cost+150)
			// return false;
			c.val[(i + 1) * 3 + (j + 1)][0] = cost_curr;
		}
	}
	Matrix b = At*c;
	if (!b.solve(AtA))
		return false;

	// extract relative coordinates
	float divisor = (b.val[2][0] * b.val[2][0] - 4.0*b.val[0][0] * b.val[1][0]);
	if (fabs(divisor) < 1e-8 || fabs(b.val[2][0]) < 1e-8)
		return false;
	float ddv = (2.0*b.val[0][0] * b.val[4][0] - b.val[2][0] * b.val[3][0]) / divisor;
	float ddu = -(b.val[4][0] + 2.0*b.val[1][0] * ddv) / b.val[2][0];
	if (fabs(ddu) >= 1.0 || fabs(ddv) >= 1.0)
		return false;

	// update target
	u2 += (float)du - 3.0 + ddu;
	v2 += (float)dv - 3.0 + ddv;

	// return true on success
	return true;
}

void Matcher::relocateMinimum(const uint8_t* I1_du, const uint8_t* I1_dv, const int32_t* dims1,
	const uint8_t* I2_du, const uint8_t* I2_dv, const int32_t* dims2,
	const float &u1, const float &v1,
	float       &u2, float       &v2,
	uint8_t* desc_buffer) {

	// check if parabolic fitting is feasible (descriptors are within margin)
	if (u2 - 2 < margin || u2 + 2 > dims2[0] - 1 - margin || v2 - 2 < margin || v2 + 2 > dims2[1] - 1 - margin)
		return;

	// compute reference descriptor
	__m128i xmm1, xmm2;
	computeSmallDescriptor(I1_du, I1_dv, dims1[2], (int32_t)u1, (int32_t)v1, desc_buffer);
	xmm1 = _mm_load_si128((__m128i*)(desc_buffer));

	// compute cost matrix
	int32_t cost[25];
	for (int32_t dv = 0; dv < 5; dv++) {
		for (int32_t du = 0; du < 5; du++) {
			computeSmallDescriptor(I2_du, I2_dv, dims2[2], (int32_t)u2 + du - 2, (int32_t)v2 + dv - 2, desc_buffer);
			xmm2 = _mm_load_si128((__m128i*)(desc_buffer));
			xmm2 = _mm_sad_epu8(xmm1, xmm2);
			cost[dv * 5 + du] = _mm_extract_epi16(xmm2, 0) + _mm_extract_epi16(xmm2, 4);
		}
	}

	// compute minimum
	int32_t min_ind = 0;
	int32_t min_cost = cost[0];
	for (int32_t i = 1; i < 25; i++) {
		if (cost[i] < min_cost) {
			min_ind = i;
			min_cost = cost[i];
		}
	}

	// update target
	u2 += (float)(min_ind % 5) - 2.0;
	v2 += (float)(min_ind / 5) - 2.0;
}

void Matcher::refinement(vector<Matcher::p_match> &p_matched) {

	// allocate aligned memory (32 bytes for 1 descriptors)
	uint8_t* desc_buffer = (uint8_t*)_mm_malloc(32 * sizeof(uint8_t), 16);

	// copy vector (for refill)
	vector<Matcher::p_match> p_matched_copy = p_matched;
	p_matched.clear();

	// create matrices for least square fitting
	FLOAT A_data[9 * 6] = { 1, 1, 1, -1, -1, 1,
		0, 1, 0, 0, -1, 1,
		1, 1, -1, 1, -1, 1,
		1, 0, 0, -1, 0, 1,
		0, 0, 0, 0, 0, 1,
		1, 0, 0, 1, 0, 1,
		1, 1, -1, -1, 1, 1,
		0, 1, 0, 0, 1, 1,
		1, 1, 1, 1, 1, 1 };
	Matrix A(9, 6, A_data);
	Matrix At = ~A;
	Matrix AtA = At*A;

	uint8_t* I1p_du_fit = I1p_du;
	uint8_t* I1p_dv_fit = I1p_dv;
	uint8_t* I2p_du_fit = I2p_du;
	uint8_t* I2p_dv_fit = I2p_dv;
	uint8_t* I1c_du_fit = I1c_du;
	uint8_t* I1c_dv_fit = I1c_dv;
	uint8_t* I2c_du_fit = I2c_du;
	uint8_t* I2c_dv_fit = I2c_dv;
	if (param.half_resolution) {
		I1p_du_fit = I1p_du_full;
		I1p_dv_fit = I1p_dv_full;
		I2p_du_fit = I2p_du_full;
		I2p_dv_fit = I2p_dv_full;
		I1c_du_fit = I1c_du_full;
		I1c_dv_fit = I1c_dv_full;
		I2c_du_fit = I2c_du_full;
		I2c_dv_fit = I2c_dv_full;
	}

	// for all matches do
	for (vector<Matcher::p_match>::iterator it = p_matched_copy.begin(); it != p_matched_copy.end(); it++) {
		// method: flow or quad matching

		if (param.refinement == 2) {
			if (!parabolicFitting(I1c_du_fit, I1c_dv_fit, dims_c, I1p_du_fit, I1p_dv_fit, dims_p,
				it->u1c, it->v1c, it->u1p, it->v1p, At, AtA, desc_buffer))
				continue;
		}
		else {
			relocateMinimum(I1c_du_fit, I1c_dv_fit, dims_c, I1p_du_fit, I1p_dv_fit, dims_p,
				it->u1c, it->v1c, it->u1p, it->v1p, desc_buffer);
		}

		if (param.refinement == 2) {
			if (!parabolicFitting(I1c_du_fit, I1c_dv_fit, dims_c, I2c_du_fit, I2c_dv_fit, dims_c,
				it->u1c, it->v1c, it->u2c, it->v2c, At, AtA, desc_buffer))
				continue;
		}
		else {
			relocateMinimum(I1c_du_fit, I1c_dv_fit, dims_c, I2c_du_fit, I2c_dv_fit, dims_c,
				it->u1c, it->v1c, it->u2c, it->v2c, desc_buffer);
		}

		if (param.refinement == 2) {
			if (!parabolicFitting(I1c_du_fit, I1c_dv_fit, dims_c, I2p_du_fit, I2p_dv_fit, dims_p,
				it->u1c, it->v1c, it->u2p, it->v2p, At, AtA, desc_buffer))
				continue;
		}
		else {
			relocateMinimum(I1c_du_fit, I1c_dv_fit, dims_c, I2p_du_fit, I2p_dv_fit, dims_p,
				it->u1c, it->v1c, it->u2p, it->v2p, desc_buffer);
		}

		// add this match
		p_matched.push_back(*it);
	}

	// free memory
	_mm_free(desc_buffer);
}


float Matcher::mean(const uint8_t* I, const int32_t &bpl, const int32_t &u_min, const int32_t &u_max, const int32_t &v_min, const int32_t &v_max)
{
	float mean = 0;
	for (int32_t v = v_min; v <= v_max; v++)
		for (int32_t u = u_min; u <= u_max; u++)
			mean += (float)*(I + getAddressOffsetImage(u, v, bpl));
	return
		mean /= (float)((u_max - u_min + 1)*(v_max - v_min + 1));
}

// FAST C++ version
void Matcher::FastFeatures(uint8_t* I, const int32_t* dims, vector<Matcher::maximum> &maxima, int32_t t, int32_t numFastFeature)
{
	// cv::Mat src(cv::Size(dims[0], dims[1]),CV_8UC1,(void *)I, cv::Mat::AUTO_STEP);
	// std::vector<cv::KeyPoint> keypoint_vec;
	// cv::FAST(src,keypoint_vec,t,true,cv::FastFeatureDetector::TYPE_7_12);

	// sort(keypoint_vec.begin(),keypoint_vec.end(),[](const cv::KeyPoint& i, const cv::KeyPoint& j){ return i.response > j.response;});

	// int num = 0;
	// for (auto keypoint : keypoint_vec)
	// {
	// 	auto x = keypoint.pt.x;
	// 	auto y = keypoint.pt.y;
	// 	if ( x > 4 && x < dims[0] - 4 && y > 4  && y < dims[1] - 4 )
	// 		maxima.push_back(Matcher::maximum(x, y, keypoint.response, 0));
	// 	num++;
	// 	if (num == numFastFeature) break;
	// }

	// static int count;

	// std::vector<cv::KeyPoint> keypoint_vec;
	// cv::namedWindow("fast feature", cv::WINDOW_AUTOSIZE);

	Fast* f = new Fast();
	f->detect_nonmax(I, dims[0], dims[1], dims[2], t, numFastFeature);

	for (int32_t i = 0; i < f->numCornersNonmax; i++)
	{
		if (f->c[i].xCoords > 4 && f->c[i].xCoords < dims[0] - 4 && f->c[i].yCoords > 4 && f->c[i].yCoords < dims[1] - 4)
		{
			maxima.push_back(Matcher::maximum((int32_t)(f->c[i].xCoords), (int32_t)(f->c[i].yCoords), (int32_t)(f->c[i].score), 0));
			// cv::KeyPoint key;
			// key.pt.x = f->c[i].xCoords;
			// key.pt.y = f->c[i].yCoords;
			// key.response = f->c[i].score;
			// keypoint_vec.push_back(key);
		}
			
	}
	delete f;

	// cv::Mat image(cv::Size(dims[0],dims[1]),CV_8UC1,(void *)I, dims_c[2]);
	// cv::Mat outImg(cv::Size(dims[0],dims[1]),CV_8UC1);
	// drawKeypoints(image,keypoint_vec,outImg);

	// if (!(count++ % 2))
	// {
	// 	cv::imshow("fast feature",outImg);
	// 	cvWaitKey(1);
	// }
	

}
