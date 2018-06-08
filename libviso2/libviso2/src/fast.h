#ifndef __FAST_H__
#define __FAST_H__

#include <iostream>
#include <stdint.h>
#include <stdlib.h>
#include <stdio.h>
#include <emmintrin.h>

using namespace std;

class Fast 
{
public:
	typedef struct { int32_t xCoords, yCoords, score; } corner;

    int32_t numCornersNonmax;
	corner* c;

	// constructor
	Fast();

	// destructor
	~Fast();

	// method
	void detect_nonmax(uint8_t* img, int32_t width, int32_t height, int32_t stride, int32_t threshold, int32_t numFastFeature);

protected:
	void detect(int32_t* ptr, uint8_t* img, int32_t width, int32_t height, int32_t stride, int32_t threshold, int32_t* numCorners);
	void computeScore(uint8_t* img, int32_t stride, int32_t* ptr, int32_t numCorners, int32_t threshold);
	int32_t computeCornerScore(uint8_t* p, const int32_t pixel[], int32_t bstart);
	void nonmax_suppression(int32_t* pointList, int32_t numCorners, int32_t* ret_pointList);
	void make_offsets(int32_t pixel[], int32_t row_stride);
	void sortPointList(int32_t* pointlist, int32_t npoints);
	void quicksort(int32_t *pointlist, int32_t n);
	void SWAP3(int32_t* list, int32_t i, int32_t j);

};

#endif   // FAST_H
