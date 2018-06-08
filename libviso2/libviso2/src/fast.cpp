#include "fast.h"
#define Compare(X, Y) ((X)>=(Y))

Fast::Fast()
{
	numCornersNonmax = 0;
}

Fast::~Fast()
{

}

void Fast::detect_nonmax(uint8_t* img, int32_t width, int32_t height, int32_t stride, int32_t threshold, int32_t numFastFeature)
{
	// corner* temp;
	int32_t numCorners;
	int32_t* pointList;
	int32_t* ret_pointList;

	pointList = (int32_t*)_mm_malloc(width * height * 3 * sizeof(int32_t), 16);

	detect(pointList, img, width, height, stride, threshold, &numCorners);
	ret_pointList = (int32_t*)_mm_malloc(numCorners * 3 * sizeof(int32_t), 16);

	computeScore(img, stride, pointList, numCorners, threshold);
	nonmax_suppression(pointList, numCorners, ret_pointList);

	sortPointList(ret_pointList, numCornersNonmax);

	if (numCornersNonmax >= numFastFeature)
		numCornersNonmax = numFastFeature;

	c = (corner*)_mm_malloc(numCornersNonmax * sizeof(corner), 16);

	for (int32_t i = 0; i < numCornersNonmax; i++)
	{
		c[i].xCoords = *(ret_pointList + i * 3 + 0);
		c[i].yCoords = *(ret_pointList + i * 3 + 1);
		c[i].score = *(ret_pointList + i * 3 + 2);
	}

	_mm_free(pointList);
	_mm_free(ret_pointList);
}


int32_t Fast::computeCornerScore(uint8_t* p, const int32_t pixel[], int32_t bstart)
{
	int32_t bmin = bstart;
	int32_t bmax = 255;
	int32_t b = (bmax + bmin) / 2;

	/*Compute the score using binary search*/
	for (;;)
	{
		int32_t cb = *p + b;
		int32_t c_b = *p - b;


		if (p[pixel[0]] > cb)
			if (p[pixel[1]] > cb)
				if (p[pixel[2]] > cb)
					if (p[pixel[3]] > cb)
						if (p[pixel[4]] > cb)
							if (p[pixel[5]] > cb)
								if (p[pixel[6]] > cb)
									if (p[pixel[7]] > cb)
										if (p[pixel[8]] > cb)
											goto is_a_corner;
										else
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												goto is_not_a_corner;
									else if (p[pixel[7]] < c_b)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else if (p[pixel[14]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																if (p[pixel[13]] < c_b)
																	if (p[pixel[15]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else if (p[pixel[6]] < c_b)
									if (p[pixel[15]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else if (p[pixel[13]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																	if (p[pixel[14]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																if (p[pixel[13]] < c_b)
																	if (p[pixel[14]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else if (p[pixel[13]] < c_b)
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																if (p[pixel[14]] < c_b)
																	if (p[pixel[15]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else if (p[pixel[5]] < c_b)
								if (p[pixel[14]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	if (p[pixel[11]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else if (p[pixel[12]] < c_b)
										if (p[pixel[6]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[13]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[14]] < c_b)
									if (p[pixel[7]] < c_b)
										if (p[pixel[8]] < c_b)
											if (p[pixel[9]] < c_b)
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
															if (p[pixel[13]] < c_b)
																if (p[pixel[6]] < c_b)
																	goto is_a_corner;
																else
																	if (p[pixel[15]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									if (p[pixel[6]] < c_b)
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																if (p[pixel[13]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	if (p[pixel[11]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[12]] < c_b)
									if (p[pixel[7]] < c_b)
										if (p[pixel[8]] < c_b)
											if (p[pixel[9]] < c_b)
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
																if (p[pixel[6]] < c_b)
																	goto is_a_corner;
																else
																	if (p[pixel[15]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else if (p[pixel[4]] < c_b)
							if (p[pixel[13]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[11]] < c_b)
									if (p[pixel[5]] < c_b)
										if (p[pixel[6]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[12]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else if (p[pixel[13]] < c_b)
								if (p[pixel[7]] < c_b)
									if (p[pixel[8]] < c_b)
										if (p[pixel[9]] < c_b)
											if (p[pixel[10]] < c_b)
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[6]] < c_b)
															if (p[pixel[5]] < c_b)
																goto is_a_corner;
															else
																if (p[pixel[14]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
														else
															if (p[pixel[14]] < c_b)
																if (p[pixel[15]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								if (p[pixel[5]] < c_b)
									if (p[pixel[6]] < c_b)
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																if (p[pixel[10]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else if (p[pixel[11]] < c_b)
								if (p[pixel[7]] < c_b)
									if (p[pixel[8]] < c_b)
										if (p[pixel[9]] < c_b)
											if (p[pixel[10]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[6]] < c_b)
															if (p[pixel[5]] < c_b)
																goto is_a_corner;
															else
																if (p[pixel[14]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
														else
															if (p[pixel[14]] < c_b)
																if (p[pixel[15]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
					else if (p[pixel[3]] < c_b)
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else if (p[pixel[10]] < c_b)
							if (p[pixel[7]] < c_b)
								if (p[pixel[8]] < c_b)
									if (p[pixel[9]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[6]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[4]] < c_b)
														goto is_a_corner;
													else
														if (p[pixel[12]] < c_b)
															if (p[pixel[13]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															if (p[pixel[15]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															if (p[pixel[9]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else if (p[pixel[10]] < c_b)
							if (p[pixel[7]] < c_b)
								if (p[pixel[8]] < c_b)
									if (p[pixel[9]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[5]] < c_b)
														if (p[pixel[4]] < c_b)
															goto is_a_corner;
														else
															if (p[pixel[13]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
													else
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															if (p[pixel[15]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
				else if (p[pixel[2]] < c_b)
					if (p[pixel[9]] > cb)
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else if (p[pixel[9]] < c_b)
						if (p[pixel[7]] < c_b)
							if (p[pixel[8]] < c_b)
								if (p[pixel[10]] < c_b)
									if (p[pixel[6]] < c_b)
										if (p[pixel[5]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[3]] < c_b)
													goto is_a_corner;
												else
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														if (p[pixel[15]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					if (p[pixel[9]] > cb)
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														if (p[pixel[8]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else if (p[pixel[9]] < c_b)
						if (p[pixel[7]] < c_b)
							if (p[pixel[8]] < c_b)
								if (p[pixel[10]] < c_b)
									if (p[pixel[11]] < c_b)
										if (p[pixel[6]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[4]] < c_b)
													if (p[pixel[3]] < c_b)
														goto is_a_corner;
													else
														if (p[pixel[12]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														if (p[pixel[15]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
			else if (p[pixel[1]] < c_b)
				if (p[pixel[8]] > cb)
					if (p[pixel[9]] > cb)
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[2]] > cb)
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else if (p[pixel[8]] < c_b)
					if (p[pixel[7]] < c_b)
						if (p[pixel[9]] < c_b)
							if (p[pixel[6]] < c_b)
								if (p[pixel[5]] < c_b)
									if (p[pixel[4]] < c_b)
										if (p[pixel[3]] < c_b)
											if (p[pixel[2]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[10]] < c_b)
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[10]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[10]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[10]] < c_b)
									if (p[pixel[11]] < c_b)
										if (p[pixel[12]] < c_b)
											if (p[pixel[13]] < c_b)
												if (p[pixel[14]] < c_b)
													if (p[pixel[15]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
			else
				if (p[pixel[8]] > cb)
					if (p[pixel[9]] > cb)
						if (p[pixel[10]] > cb)
							if (p[pixel[11]] > cb)
								if (p[pixel[12]] > cb)
									if (p[pixel[13]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[15]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[2]] > cb)
									if (p[pixel[3]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[7]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else if (p[pixel[8]] < c_b)
					if (p[pixel[7]] < c_b)
						if (p[pixel[9]] < c_b)
							if (p[pixel[10]] < c_b)
								if (p[pixel[6]] < c_b)
									if (p[pixel[5]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[3]] < c_b)
												if (p[pixel[2]] < c_b)
													goto is_a_corner;
												else
													if (p[pixel[11]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[11]] < c_b)
										if (p[pixel[12]] < c_b)
											if (p[pixel[13]] < c_b)
												if (p[pixel[14]] < c_b)
													if (p[pixel[15]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
		else if (p[pixel[0]] < c_b)
			if (p[pixel[1]] > cb)
				if (p[pixel[8]] > cb)
					if (p[pixel[7]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[6]] > cb)
								if (p[pixel[5]] > cb)
									if (p[pixel[4]] > cb)
										if (p[pixel[3]] > cb)
											if (p[pixel[2]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[10]] > cb)
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[10]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[10]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[10]] > cb)
									if (p[pixel[11]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[14]] > cb)
													if (p[pixel[15]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else if (p[pixel[8]] < c_b)
					if (p[pixel[9]] < c_b)
						if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[2]] < c_b)
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
			else if (p[pixel[1]] < c_b)
				if (p[pixel[2]] > cb)
					if (p[pixel[9]] > cb)
						if (p[pixel[7]] > cb)
							if (p[pixel[8]] > cb)
								if (p[pixel[10]] > cb)
									if (p[pixel[6]] > cb)
										if (p[pixel[5]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[3]] > cb)
													goto is_a_corner;
												else
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														if (p[pixel[15]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else if (p[pixel[9]] < c_b)
						if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else if (p[pixel[2]] < c_b)
					if (p[pixel[3]] > cb)
						if (p[pixel[10]] > cb)
							if (p[pixel[7]] > cb)
								if (p[pixel[8]] > cb)
									if (p[pixel[9]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[6]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[4]] > cb)
														goto is_a_corner;
													else
														if (p[pixel[12]] > cb)
															if (p[pixel[13]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															if (p[pixel[15]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else if (p[pixel[3]] < c_b)
						if (p[pixel[4]] > cb)
							if (p[pixel[13]] > cb)
								if (p[pixel[7]] > cb)
									if (p[pixel[8]] > cb)
										if (p[pixel[9]] > cb)
											if (p[pixel[10]] > cb)
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[6]] > cb)
															if (p[pixel[5]] > cb)
																goto is_a_corner;
															else
																if (p[pixel[14]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
														else
															if (p[pixel[14]] > cb)
																if (p[pixel[15]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else if (p[pixel[13]] < c_b)
								if (p[pixel[11]] > cb)
									if (p[pixel[5]] > cb)
										if (p[pixel[6]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[12]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								if (p[pixel[5]] > cb)
									if (p[pixel[6]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else if (p[pixel[4]] < c_b)
							if (p[pixel[5]] > cb)
								if (p[pixel[14]] > cb)
									if (p[pixel[7]] > cb)
										if (p[pixel[8]] > cb)
											if (p[pixel[9]] > cb)
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
															if (p[pixel[13]] > cb)
																if (p[pixel[6]] > cb)
																	goto is_a_corner;
																else
																	if (p[pixel[15]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[14]] < c_b)
									if (p[pixel[12]] > cb)
										if (p[pixel[6]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[13]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	if (p[pixel[11]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									if (p[pixel[6]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																if (p[pixel[13]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else if (p[pixel[5]] < c_b)
								if (p[pixel[6]] > cb)
									if (p[pixel[15]] < c_b)
										if (p[pixel[13]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																	if (p[pixel[14]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																if (p[pixel[13]] > cb)
																	if (p[pixel[14]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else if (p[pixel[6]] < c_b)
									if (p[pixel[7]] > cb)
										if (p[pixel[14]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																if (p[pixel[13]] > cb)
																	if (p[pixel[15]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else if (p[pixel[7]] < c_b)
										if (p[pixel[8]] < c_b)
											goto is_a_corner;
										else
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[13]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																if (p[pixel[14]] > cb)
																	if (p[pixel[15]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[12]] > cb)
									if (p[pixel[7]] > cb)
										if (p[pixel[8]] > cb)
											if (p[pixel[9]] > cb)
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
																if (p[pixel[6]] > cb)
																	goto is_a_corner;
																else
																	if (p[pixel[15]] > cb)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	if (p[pixel[11]] < c_b)
																		goto is_a_corner;
																	else
																		goto is_not_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							if (p[pixel[11]] > cb)
								if (p[pixel[7]] > cb)
									if (p[pixel[8]] > cb)
										if (p[pixel[9]] > cb)
											if (p[pixel[10]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[6]] > cb)
															if (p[pixel[5]] > cb)
																goto is_a_corner;
															else
																if (p[pixel[14]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
														else
															if (p[pixel[14]] > cb)
																if (p[pixel[15]] > cb)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																if (p[pixel[10]] < c_b)
																	goto is_a_corner;
																else
																	goto is_not_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
					else
						if (p[pixel[10]] > cb)
							if (p[pixel[7]] > cb)
								if (p[pixel[8]] > cb)
									if (p[pixel[9]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[5]] > cb)
														if (p[pixel[4]] > cb)
															goto is_a_corner;
														else
															if (p[pixel[13]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
													else
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															if (p[pixel[15]] > cb)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															if (p[pixel[9]] < c_b)
																goto is_a_corner;
															else
																goto is_not_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
				else
					if (p[pixel[9]] > cb)
						if (p[pixel[7]] > cb)
							if (p[pixel[8]] > cb)
								if (p[pixel[10]] > cb)
									if (p[pixel[11]] > cb)
										if (p[pixel[6]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[4]] > cb)
													if (p[pixel[3]] > cb)
														goto is_a_corner;
													else
														if (p[pixel[12]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
												else
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														if (p[pixel[15]] > cb)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else if (p[pixel[9]] < c_b)
						if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														if (p[pixel[8]] < c_b)
															goto is_a_corner;
														else
															goto is_not_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
			else
				if (p[pixel[8]] > cb)
					if (p[pixel[7]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[6]] > cb)
									if (p[pixel[5]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[3]] > cb)
												if (p[pixel[2]] > cb)
													goto is_a_corner;
												else
													if (p[pixel[11]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
											else
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[11]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[14]] > cb)
													if (p[pixel[15]] > cb)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else if (p[pixel[8]] < c_b)
					if (p[pixel[9]] < c_b)
						if (p[pixel[10]] < c_b)
							if (p[pixel[11]] < c_b)
								if (p[pixel[12]] < c_b)
									if (p[pixel[13]] < c_b)
										if (p[pixel[14]] < c_b)
											if (p[pixel[15]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[2]] < c_b)
									if (p[pixel[3]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[7]] < c_b)
														goto is_a_corner;
													else
														goto is_not_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
		else
			if (p[pixel[7]] > cb)
				if (p[pixel[8]] > cb)
					if (p[pixel[9]] > cb)
						if (p[pixel[6]] > cb)
							if (p[pixel[5]] > cb)
								if (p[pixel[4]] > cb)
									if (p[pixel[3]] > cb)
										if (p[pixel[2]] > cb)
											if (p[pixel[1]] > cb)
												goto is_a_corner;
											else
												if (p[pixel[10]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[10]] > cb)
												if (p[pixel[11]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[10]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[10]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[10]] > cb)
									if (p[pixel[11]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[14]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
			else if (p[pixel[7]] < c_b)
				if (p[pixel[8]] < c_b)
					if (p[pixel[9]] < c_b)
						if (p[pixel[6]] < c_b)
							if (p[pixel[5]] < c_b)
								if (p[pixel[4]] < c_b)
									if (p[pixel[3]] < c_b)
										if (p[pixel[2]] < c_b)
											if (p[pixel[1]] < c_b)
												goto is_a_corner;
											else
												if (p[pixel[10]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
										else
											if (p[pixel[10]] < c_b)
												if (p[pixel[11]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
									else
										if (p[pixel[10]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
								else
									if (p[pixel[10]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
							else
								if (p[pixel[10]] < c_b)
									if (p[pixel[11]] < c_b)
										if (p[pixel[12]] < c_b)
											if (p[pixel[13]] < c_b)
												if (p[pixel[14]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
						else
							if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
													goto is_a_corner;
												else
													goto is_not_a_corner;
											else
												goto is_not_a_corner;
										else
											goto is_not_a_corner;
									else
										goto is_not_a_corner;
								else
									goto is_not_a_corner;
							else
								goto is_not_a_corner;
					else
						goto is_not_a_corner;
				else
					goto is_not_a_corner;
			else
				goto is_not_a_corner;

		is_a_corner:
			bmin = b;
			goto end_if;

		is_not_a_corner:
			bmax = b;
			goto end_if;

		end_if:

			if (bmin == bmax - 1 || bmin == bmax)
				return bmin;
			b = (bmin + bmax) / 2;
	}
}

void Fast::make_offsets(int32_t pixel[], int32_t row_stride)
{
	pixel[0] = 0 + row_stride * 3;
	pixel[1] = 1 + row_stride * 3;
	pixel[2] = 2 + row_stride * 2;
	pixel[3] = 3 + row_stride * 1;
	pixel[4] = 3 + row_stride * 0;
	pixel[5] = 3 + row_stride * -1;
	pixel[6] = 2 + row_stride * -2;
	pixel[7] = 1 + row_stride * -3;
	pixel[8] = 0 + row_stride * -3;
	pixel[9] = -1 + row_stride * -3;
	pixel[10] = -2 + row_stride * -2;
	pixel[11] = -3 + row_stride * -1;
	pixel[12] = -3 + row_stride * 0;
	pixel[13] = -3 + row_stride * 1;
	pixel[14] = -2 + row_stride * 2;
	pixel[15] = -1 + row_stride * 3;
}



void Fast::computeScore(uint8_t* img, int32_t stride, int32_t* pointList, int32_t numCorners, int32_t threshold)
{
	int32_t pixel[16];
	make_offsets(pixel, stride);

	for (int32_t n = 0; n < numCorners; n++)
		*(pointList + n * 3 + 2) = computeCornerScore(img + *(pointList + n * 3 + 1) *stride + *(pointList + n * 3 + 0), pixel, threshold);
}

void Fast::detect(int32_t* ptr, uint8_t* img, int32_t width, int32_t height, int32_t stride, int32_t threshold, int32_t* numCorners)
{
	// int32_t rsize = 512;
	int32_t pixel[16];
	int32_t x, y;
	// corner* ret_corners;
	int32_t num_corners = 0;

	// ret_corners = (corner*)malloc(sizeof(corner)*rsize);
	make_offsets(pixel, stride);

	for (y = 3; y < height - 3; y++)
		for (x = 3; x < width - 3; x++)
		{
			const uint8_t* p = img + y*stride + x;

			int32_t cb = *p + threshold;	// >cb, brighter
			int32_t c_b = *p - threshold;	// <c_b, darker
			if (p[pixel[0]] > cb)
				if (p[pixel[1]] > cb)
					if (p[pixel[2]] > cb)
						if (p[pixel[3]] > cb)
							if (p[pixel[4]] > cb)
								if (p[pixel[5]] > cb)
									if (p[pixel[6]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
											{
											}
											else
												if (p[pixel[15]] > cb)
												{
												}
												else
													continue;
										else if (p[pixel[7]] < c_b)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													continue;
											else if (p[pixel[14]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																	if (p[pixel[13]] < c_b)
																		if (p[pixel[15]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													continue;
											else
												continue;
									else if (p[pixel[6]] < c_b)
										if (p[pixel[15]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[14]] > cb)
												{
												}
												else
													continue;
											else if (p[pixel[13]] < c_b)
												if (p[pixel[7]] < c_b)
													if (p[pixel[8]] < c_b)
														if (p[pixel[9]] < c_b)
															if (p[pixel[10]] < c_b)
																if (p[pixel[11]] < c_b)
																	if (p[pixel[12]] < c_b)
																		if (p[pixel[14]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																	if (p[pixel[13]] < c_b)
																		if (p[pixel[14]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													continue;
											else
												continue;
										else if (p[pixel[13]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																	if (p[pixel[14]] < c_b)
																		if (p[pixel[15]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else if (p[pixel[5]] < c_b)
									if (p[pixel[14]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																		if (p[pixel[11]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else if (p[pixel[12]] < c_b)
											if (p[pixel[6]] < c_b)
												if (p[pixel[7]] < c_b)
													if (p[pixel[8]] < c_b)
														if (p[pixel[9]] < c_b)
															if (p[pixel[10]] < c_b)
																if (p[pixel[11]] < c_b)
																	if (p[pixel[13]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else if (p[pixel[14]] < c_b)
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
																if (p[pixel[13]] < c_b)
																	if (p[pixel[6]] < c_b)
																	{
																	}
																	else
																		if (p[pixel[15]] < c_b)
																		{
																		}
																		else
																			continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										if (p[pixel[6]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																	if (p[pixel[13]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																		if (p[pixel[11]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else if (p[pixel[12]] < c_b)
										if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
												if (p[pixel[9]] < c_b)
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
															if (p[pixel[13]] < c_b)
																if (p[pixel[14]] < c_b)
																	if (p[pixel[6]] < c_b)
																	{
																	}
																	else
																		if (p[pixel[15]] < c_b)
																		{
																		}
																		else
																			continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else if (p[pixel[4]] < c_b)
								if (p[pixel[13]] > cb)
									if (p[pixel[11]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else if (p[pixel[11]] < c_b)
										if (p[pixel[5]] < c_b)
											if (p[pixel[6]] < c_b)
												if (p[pixel[7]] < c_b)
													if (p[pixel[8]] < c_b)
														if (p[pixel[9]] < c_b)
															if (p[pixel[10]] < c_b)
																if (p[pixel[12]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else if (p[pixel[13]] < c_b)
									if (p[pixel[7]] < c_b)
										if (p[pixel[8]] < c_b)
											if (p[pixel[9]] < c_b)
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
															if (p[pixel[6]] < c_b)
																if (p[pixel[5]] < c_b)
																{
																}
																else
																	if (p[pixel[14]] < c_b)
																	{
																	}
																	else
																		continue;
															else
																if (p[pixel[14]] < c_b)
																	if (p[pixel[15]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									if (p[pixel[5]] < c_b)
										if (p[pixel[6]] < c_b)
											if (p[pixel[7]] < c_b)
												if (p[pixel[8]] < c_b)
													if (p[pixel[9]] < c_b)
														if (p[pixel[10]] < c_b)
															if (p[pixel[11]] < c_b)
																if (p[pixel[12]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																	if (p[pixel[10]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else if (p[pixel[11]] < c_b)
									if (p[pixel[7]] < c_b)
										if (p[pixel[8]] < c_b)
											if (p[pixel[9]] < c_b)
												if (p[pixel[10]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[6]] < c_b)
																if (p[pixel[5]] < c_b)
																{
																}
																else
																	if (p[pixel[14]] < c_b)
																	{
																	}
																	else
																		continue;
															else
																if (p[pixel[14]] < c_b)
																	if (p[pixel[15]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else if (p[pixel[3]] < c_b)
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else if (p[pixel[10]] < c_b)
								if (p[pixel[7]] < c_b)
									if (p[pixel[8]] < c_b)
										if (p[pixel[9]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[6]] < c_b)
													if (p[pixel[5]] < c_b)
														if (p[pixel[4]] < c_b)
														{
														}
														else
															if (p[pixel[12]] < c_b)
																if (p[pixel[13]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
													else
														if (p[pixel[12]] < c_b)
															if (p[pixel[13]] < c_b)
																if (p[pixel[14]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
																if (p[pixel[15]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
																if (p[pixel[9]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else if (p[pixel[10]] < c_b)
								if (p[pixel[7]] < c_b)
									if (p[pixel[8]] < c_b)
										if (p[pixel[9]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[5]] < c_b)
															if (p[pixel[4]] < c_b)
															{
															}
															else
																if (p[pixel[13]] < c_b)
																{
																}
																else
																	continue;
														else
															if (p[pixel[13]] < c_b)
																if (p[pixel[14]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
													else
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
																if (p[pixel[15]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
					else if (p[pixel[2]] < c_b)
						if (p[pixel[9]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else if (p[pixel[9]] < c_b)
							if (p[pixel[7]] < c_b)
								if (p[pixel[8]] < c_b)
									if (p[pixel[10]] < c_b)
										if (p[pixel[6]] < c_b)
											if (p[pixel[5]] < c_b)
												if (p[pixel[4]] < c_b)
													if (p[pixel[3]] < c_b)
													{
													}
													else
														if (p[pixel[11]] < c_b)
															if (p[pixel[12]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
												else
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
															if (p[pixel[13]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															if (p[pixel[15]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else
						if (p[pixel[9]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
															if (p[pixel[8]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else if (p[pixel[9]] < c_b)
							if (p[pixel[7]] < c_b)
								if (p[pixel[8]] < c_b)
									if (p[pixel[10]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[6]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[4]] < c_b)
														if (p[pixel[3]] < c_b)
														{
														}
														else
															if (p[pixel[12]] < c_b)
															{
															}
															else
																continue;
													else
														if (p[pixel[12]] < c_b)
															if (p[pixel[13]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
												else
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
															if (p[pixel[14]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
															if (p[pixel[15]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else
							continue;
				else if (p[pixel[1]] < c_b)
					if (p[pixel[8]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[2]] > cb)
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else if (p[pixel[8]] < c_b)
						if (p[pixel[7]] < c_b)
							if (p[pixel[9]] < c_b)
								if (p[pixel[6]] < c_b)
									if (p[pixel[5]] < c_b)
										if (p[pixel[4]] < c_b)
											if (p[pixel[3]] < c_b)
												if (p[pixel[2]] < c_b)
												{
												}
												else
													if (p[pixel[10]] < c_b)
														if (p[pixel[11]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[10]] < c_b)
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[10]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[10]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														if (p[pixel[15]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else
						continue;
				else
					if (p[pixel[8]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[11]] > cb)
									if (p[pixel[12]] > cb)
										if (p[pixel[13]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[15]] > cb)
												{
												}
												else
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[2]] > cb)
										if (p[pixel[3]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[7]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else if (p[pixel[8]] < c_b)
						if (p[pixel[7]] < c_b)
							if (p[pixel[9]] < c_b)
								if (p[pixel[10]] < c_b)
									if (p[pixel[6]] < c_b)
										if (p[pixel[5]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[3]] < c_b)
													if (p[pixel[2]] < c_b)
													{
													}
													else
														if (p[pixel[11]] < c_b)
														{
														}
														else
															continue;
												else
													if (p[pixel[11]] < c_b)
														if (p[pixel[12]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
														if (p[pixel[13]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
														if (p[pixel[14]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
														if (p[pixel[15]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else
						continue;
			else if (p[pixel[0]] < c_b)
				if (p[pixel[1]] > cb)
					if (p[pixel[8]] > cb)
						if (p[pixel[7]] > cb)
							if (p[pixel[9]] > cb)
								if (p[pixel[6]] > cb)
									if (p[pixel[5]] > cb)
										if (p[pixel[4]] > cb)
											if (p[pixel[3]] > cb)
												if (p[pixel[2]] > cb)
												{
												}
												else
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[10]] > cb)
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[10]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[10]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														if (p[pixel[15]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else if (p[pixel[8]] < c_b)
						if (p[pixel[9]] < c_b)
							if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[2]] < c_b)
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else
						continue;
				else if (p[pixel[1]] < c_b)
					if (p[pixel[2]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[7]] > cb)
								if (p[pixel[8]] > cb)
									if (p[pixel[10]] > cb)
										if (p[pixel[6]] > cb)
											if (p[pixel[5]] > cb)
												if (p[pixel[4]] > cb)
													if (p[pixel[3]] > cb)
													{
													}
													else
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
															{
															}
															else
																continue;
														else
															continue;
												else
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
															if (p[pixel[13]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															if (p[pixel[15]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else if (p[pixel[9]] < c_b)
							if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else if (p[pixel[2]] < c_b)
						if (p[pixel[3]] > cb)
							if (p[pixel[10]] > cb)
								if (p[pixel[7]] > cb)
									if (p[pixel[8]] > cb)
										if (p[pixel[9]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[6]] > cb)
													if (p[pixel[5]] > cb)
														if (p[pixel[4]] > cb)
														{
														}
														else
															if (p[pixel[12]] > cb)
																if (p[pixel[13]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
													else
														if (p[pixel[12]] > cb)
															if (p[pixel[13]] > cb)
																if (p[pixel[14]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
																if (p[pixel[15]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else if (p[pixel[3]] < c_b)
							if (p[pixel[4]] > cb)
								if (p[pixel[13]] > cb)
									if (p[pixel[7]] > cb)
										if (p[pixel[8]] > cb)
											if (p[pixel[9]] > cb)
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
															if (p[pixel[6]] > cb)
																if (p[pixel[5]] > cb)
																{
																}
																else
																	if (p[pixel[14]] > cb)
																	{
																	}
																	else
																		continue;
															else
																if (p[pixel[14]] > cb)
																	if (p[pixel[15]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else if (p[pixel[13]] < c_b)
									if (p[pixel[11]] > cb)
										if (p[pixel[5]] > cb)
											if (p[pixel[6]] > cb)
												if (p[pixel[7]] > cb)
													if (p[pixel[8]] > cb)
														if (p[pixel[9]] > cb)
															if (p[pixel[10]] > cb)
																if (p[pixel[12]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else if (p[pixel[11]] < c_b)
										if (p[pixel[12]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									if (p[pixel[5]] > cb)
										if (p[pixel[6]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else if (p[pixel[4]] < c_b)
								if (p[pixel[5]] > cb)
									if (p[pixel[14]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[12]] > cb)
																if (p[pixel[13]] > cb)
																	if (p[pixel[6]] > cb)
																	{
																	}
																	else
																		if (p[pixel[15]] > cb)
																		{
																		}
																		else
																			continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else if (p[pixel[14]] < c_b)
										if (p[pixel[12]] > cb)
											if (p[pixel[6]] > cb)
												if (p[pixel[7]] > cb)
													if (p[pixel[8]] > cb)
														if (p[pixel[9]] > cb)
															if (p[pixel[10]] > cb)
																if (p[pixel[11]] > cb)
																	if (p[pixel[13]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else if (p[pixel[12]] < c_b)
											if (p[pixel[13]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																		if (p[pixel[11]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										if (p[pixel[6]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																	if (p[pixel[13]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else if (p[pixel[5]] < c_b)
									if (p[pixel[6]] > cb)
										if (p[pixel[15]] < c_b)
											if (p[pixel[13]] > cb)
												if (p[pixel[7]] > cb)
													if (p[pixel[8]] > cb)
														if (p[pixel[9]] > cb)
															if (p[pixel[10]] > cb)
																if (p[pixel[11]] > cb)
																	if (p[pixel[12]] > cb)
																		if (p[pixel[14]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else if (p[pixel[13]] < c_b)
												if (p[pixel[14]] < c_b)
												{
												}
												else
													continue;
											else
												continue;
										else
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																	if (p[pixel[13]] > cb)
																		if (p[pixel[14]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else if (p[pixel[6]] < c_b)
										if (p[pixel[7]] > cb)
											if (p[pixel[14]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																	if (p[pixel[13]] > cb)
																		if (p[pixel[15]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													continue;
											else
												continue;
										else if (p[pixel[7]] < c_b)
											if (p[pixel[8]] < c_b)
											{
											}
											else
												if (p[pixel[15]] < c_b)
												{
												}
												else
													continue;
										else
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													continue;
											else
												continue;
									else
										if (p[pixel[13]] > cb)
											if (p[pixel[7]] > cb)
												if (p[pixel[8]] > cb)
													if (p[pixel[9]] > cb)
														if (p[pixel[10]] > cb)
															if (p[pixel[11]] > cb)
																if (p[pixel[12]] > cb)
																	if (p[pixel[14]] > cb)
																		if (p[pixel[15]] > cb)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[12]] > cb)
										if (p[pixel[7]] > cb)
											if (p[pixel[8]] > cb)
												if (p[pixel[9]] > cb)
													if (p[pixel[10]] > cb)
														if (p[pixel[11]] > cb)
															if (p[pixel[13]] > cb)
																if (p[pixel[14]] > cb)
																	if (p[pixel[6]] > cb)
																	{
																	}
																	else
																		if (p[pixel[15]] > cb)
																		{
																		}
																		else
																			continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																		if (p[pixel[11]] < c_b)
																		{
																		}
																		else
																			continue;
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								if (p[pixel[11]] > cb)
									if (p[pixel[7]] > cb)
										if (p[pixel[8]] > cb)
											if (p[pixel[9]] > cb)
												if (p[pixel[10]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[6]] > cb)
																if (p[pixel[5]] > cb)
																{
																}
																else
																	if (p[pixel[14]] > cb)
																	{
																	}
																	else
																		continue;
															else
																if (p[pixel[14]] > cb)
																	if (p[pixel[15]] > cb)
																	{
																	}
																	else
																		continue;
																else
																	continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																	if (p[pixel[10]] < c_b)
																	{
																	}
																	else
																		continue;
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else
							if (p[pixel[10]] > cb)
								if (p[pixel[7]] > cb)
									if (p[pixel[8]] > cb)
										if (p[pixel[9]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[6]] > cb)
														if (p[pixel[5]] > cb)
															if (p[pixel[4]] > cb)
															{
															}
															else
																if (p[pixel[13]] > cb)
																{
																}
																else
																	continue;
														else
															if (p[pixel[13]] > cb)
																if (p[pixel[14]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
													else
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
																if (p[pixel[15]] > cb)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
																if (p[pixel[9]] < c_b)
																{
																}
																else
																	continue;
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										continue;
								else
									continue;
							else
								continue;
					else
						if (p[pixel[9]] > cb)
							if (p[pixel[7]] > cb)
								if (p[pixel[8]] > cb)
									if (p[pixel[10]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[6]] > cb)
												if (p[pixel[5]] > cb)
													if (p[pixel[4]] > cb)
														if (p[pixel[3]] > cb)
														{
														}
														else
															if (p[pixel[12]] > cb)
															{
															}
															else
																continue;
													else
														if (p[pixel[12]] > cb)
															if (p[pixel[13]] > cb)
															{
															}
															else
																continue;
														else
															continue;
												else
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
															if (p[pixel[14]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
															if (p[pixel[15]] > cb)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											continue;
									else
										continue;
								else
									continue;
							else
								continue;
						else if (p[pixel[9]] < c_b)
							if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
															if (p[pixel[8]] < c_b)
															{
															}
															else
																continue;
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
				else
					if (p[pixel[8]] > cb)
						if (p[pixel[7]] > cb)
							if (p[pixel[9]] > cb)
								if (p[pixel[10]] > cb)
									if (p[pixel[6]] > cb)
										if (p[pixel[5]] > cb)
											if (p[pixel[4]] > cb)
												if (p[pixel[3]] > cb)
													if (p[pixel[2]] > cb)
													{
													}
													else
														if (p[pixel[11]] > cb)
														{
														}
														else
															continue;
												else
													if (p[pixel[11]] > cb)
														if (p[pixel[12]] > cb)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
														if (p[pixel[13]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
														if (p[pixel[14]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
														if (p[pixel[15]] > cb)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									continue;
							else
								continue;
						else
							continue;
					else if (p[pixel[8]] < c_b)
						if (p[pixel[9]] < c_b)
							if (p[pixel[10]] < c_b)
								if (p[pixel[11]] < c_b)
									if (p[pixel[12]] < c_b)
										if (p[pixel[13]] < c_b)
											if (p[pixel[14]] < c_b)
												if (p[pixel[15]] < c_b)
												{
												}
												else
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
											else
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
										else
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[2]] < c_b)
										if (p[pixel[3]] < c_b)
											if (p[pixel[4]] < c_b)
												if (p[pixel[5]] < c_b)
													if (p[pixel[6]] < c_b)
														if (p[pixel[7]] < c_b)
														{
														}
														else
															continue;
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								continue;
						else
							continue;
					else
						continue;
			else
				if (p[pixel[7]] > cb)
					if (p[pixel[8]] > cb)
						if (p[pixel[9]] > cb)
							if (p[pixel[6]] > cb)
								if (p[pixel[5]] > cb)
									if (p[pixel[4]] > cb)
										if (p[pixel[3]] > cb)
											if (p[pixel[2]] > cb)
												if (p[pixel[1]] > cb)
												{
												}
												else
													if (p[pixel[10]] > cb)
													{
													}
													else
														continue;
											else
												if (p[pixel[10]] > cb)
													if (p[pixel[11]] > cb)
													{
													}
													else
														continue;
												else
													continue;
										else
											if (p[pixel[10]] > cb)
												if (p[pixel[11]] > cb)
													if (p[pixel[12]] > cb)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[10]] > cb)
											if (p[pixel[11]] > cb)
												if (p[pixel[12]] > cb)
													if (p[pixel[13]] > cb)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[10]] > cb)
										if (p[pixel[11]] > cb)
											if (p[pixel[12]] > cb)
												if (p[pixel[13]] > cb)
													if (p[pixel[14]] > cb)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								if (p[pixel[10]] > cb)
									if (p[pixel[11]] > cb)
										if (p[pixel[12]] > cb)
											if (p[pixel[13]] > cb)
												if (p[pixel[14]] > cb)
													if (p[pixel[15]] > cb)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else
							continue;
					else
						continue;
				else if (p[pixel[7]] < c_b)
					if (p[pixel[8]] < c_b)
						if (p[pixel[9]] < c_b)
							if (p[pixel[6]] < c_b)
								if (p[pixel[5]] < c_b)
									if (p[pixel[4]] < c_b)
										if (p[pixel[3]] < c_b)
											if (p[pixel[2]] < c_b)
												if (p[pixel[1]] < c_b)
												{
												}
												else
													if (p[pixel[10]] < c_b)
													{
													}
													else
														continue;
											else
												if (p[pixel[10]] < c_b)
													if (p[pixel[11]] < c_b)
													{
													}
													else
														continue;
												else
													continue;
										else
											if (p[pixel[10]] < c_b)
												if (p[pixel[11]] < c_b)
													if (p[pixel[12]] < c_b)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
									else
										if (p[pixel[10]] < c_b)
											if (p[pixel[11]] < c_b)
												if (p[pixel[12]] < c_b)
													if (p[pixel[13]] < c_b)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
								else
									if (p[pixel[10]] < c_b)
										if (p[pixel[11]] < c_b)
											if (p[pixel[12]] < c_b)
												if (p[pixel[13]] < c_b)
													if (p[pixel[14]] < c_b)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
							else
								if (p[pixel[10]] < c_b)
									if (p[pixel[11]] < c_b)
										if (p[pixel[12]] < c_b)
											if (p[pixel[13]] < c_b)
												if (p[pixel[14]] < c_b)
													if (p[pixel[15]] < c_b)
													{
													}
													else
														continue;
												else
													continue;
											else
												continue;
										else
											continue;
									else
										continue;
								else
									continue;
						else
							continue;
					else
						continue;
				else
					continue;
				/*
				if (num_corners == rsize)
				{
				rsize *= 2;
				ret_corners = (corner*)realloc(ret_corners, sizeof(corner)*rsize);
				}
				ret_corners[num_corners].x = x;
				ret_corners[num_corners].y = y;
				*/

				*(ptr + num_corners * 3 + 0) = x;
				*(ptr + num_corners * 3 + 1) = y;
				*(ptr + num_corners * 3 + 2) = 0;		// value
				num_corners++;
		}

	*numCorners = num_corners;
	// return ret_corners;
}

void Fast::nonmax_suppression(int32_t* pointList, int32_t numCorners, int32_t* ret_pointList)
{
	int32_t last_row;
	int32_t* row_start;
	const int32_t sz = (int32_t)numCorners;


	/*Point above points (roughly) to the pixel above the one of interest, if there
	is a feature there.*/
	int32_t point_above = 0;
	int32_t point_below = 0;


	if (numCorners < 1)
	{
		numCornersNonmax = 0;
		return;
	}

	/* Find where each row begins
	(the corners are output in raster scan order). A beginning of -1 signifies
	that there are no corners on that row. */
	last_row = *(pointList + (numCorners - 1) * 3 + 1);
	row_start = (int32_t*)malloc((last_row + 1)*sizeof(int32_t));

	for (int32_t i = 0; i < last_row + 1; i++)
		row_start[i] = -1;

	int32_t prev_row = -1;
	for (int32_t i = 0; i < numCorners; i++)
		if (*(pointList + i * 3 + 1) != prev_row)
		{
			row_start[*(pointList + i * 3 + 1)] = i;
			prev_row = *(pointList + i * 3 + 1);
		}


	for (int32_t i = 0; i < sz; i++)
	{

		/*Check left */
		if (i > 0)
			if (*(pointList + (i - 1) * 3 + 0) == *(pointList + i * 3 + 0) - 1
				&& *(pointList + (i - 1) * 3 + 1) == *(pointList + i * 3 + 1)
				&& Compare(*(pointList + (i - 1) * 3 + 2), *(pointList + i * 3 + 2)))
				continue;

		/*Check right*/
		if (i < (sz - 1))
			if (*(pointList + (i + 1) * 3 + 0) == *(pointList + i * 3 + 0) + 1
				&& *(pointList + (i + 1) * 3 + 1) == *(pointList + i * 3 + 1)
				&& Compare(*(pointList + (i + 1) * 3 + 2), *(pointList + i * 3 + 2)))
				continue;

		/*Check above (if there is a valid row above)*/
		if (*(pointList + i * 3 + 1) != 0 && row_start[*(pointList + i * 3 + 1) - 1] != -1)
		{
			/*Make sure that current point_above is one
			row above.*/
			if (*(pointList + point_above * 3 + 1) < *(pointList + i * 3 + 1) - 1)
				point_above = row_start[*(pointList + i * 3 + 1) - 1];

			/*Make point_above point to the first of the pixels above the current point,
			if it exists.*/
			for (; *(pointList + point_above * 3 + 1) < *(pointList + i * 3 + 1)
				&& *(pointList + point_above * 3 + 0) < *(pointList + i * 3 + 0) - 1; point_above++)
			{
			}

			for (int32_t j = point_above; *(pointList + j * 3 + 1) < *(pointList + i * 3 + 1) && *(pointList + j * 3 + 0) <= *(pointList + i * 3 + 0) + 1; j++)
			{
				int32_t x = *(pointList + j * 3 + 0);
				if ((x == *(pointList + i * 3 + 0) - 1 || x == *(pointList + i * 3 + 0) || x == *(pointList + i * 3 + 0) + 1) && Compare(*(pointList + j * 3 + 2), *(pointList + i * 3 + 2)))
					goto cont;
			}

		}

		/*Check below (if there is anything below)*/
		if (*(pointList + i * 3 + 1) != last_row && row_start[*(pointList + i * 3 + 1) + 1] != -1 && point_below < sz) /*Nothing below*/
		{
			if (*(pointList + point_below * 3 + 1) < *(pointList + i * 3 + 1) + 1)
				point_below = row_start[*(pointList + i * 3 + 1) + 1];

			/* Make point below point to one of the pixels belowthe current point, if it
			exists.*/
			for (; point_below < sz && *(pointList + point_below * 3 + 1) == *(pointList + i * 3 + 1) + 1 && *(pointList + point_below * 3 + 0) < *(pointList + i * 3 + 0) - 1; point_below++)
			{
			}

			for (int32_t j = point_below; j < sz && *(pointList + j * 3 + 1) == *(pointList + i * 3 + 1) + 1 && *(pointList + j * 3 + 0) <= *(pointList + i * 3 + 0) + 1; j++)
			{
				int32_t x = *(pointList + j * 3 + 0);
				if ((x == *(pointList + i * 3 + 0) - 1 || x == *(pointList + i * 3 + 0) || x == *(pointList + i * 3 + 0) + 1) && Compare(*(pointList + j * 3 + 2), *(pointList + i * 3 + 2)))
					goto cont;
			}
		}

		*(ret_pointList + numCornersNonmax * 3 + 0) = *(pointList + i * 3 + 0);			// x
		*(ret_pointList + numCornersNonmax * 3 + 1) = *(pointList + i * 3 + 1);			// y
		*(ret_pointList + numCornersNonmax * 3 + 2) = *(pointList + i * 3 + 2);			// score
		numCornersNonmax++;
	cont:
		;
	}

	free(row_start);
}

// function to sort corners
void Fast::sortPointList(int32_t *pointlist, int32_t npoints)
{
	quicksort(pointlist, npoints);
}

void Fast::quicksort(int32_t *pointlist, int32_t n)
{
	uint32_t i, j, ln, rn;

	while (n > 1)
	{
		SWAP3(pointlist, 0, n / 2);
		for (i = 0, j = n;;)
		{
			do
				--j;
			while (pointlist[3 * j + 2] < pointlist[2]);
			do
				++i;
			while (i < j && pointlist[3 * i + 2] > pointlist[2]);
			if (i >= j)
				break;
			SWAP3(pointlist, i, j);
		}
		SWAP3(pointlist, j, 0);
		ln = j;
		rn = n - ++j;
		if (ln < rn)
		{
			quicksort(pointlist, ln);
			pointlist += 3 * j;
			n = rn;
		}
		else
		{
			quicksort(pointlist + 3 * j, rn);
			n = ln;
		}
	}
}

void Fast::SWAP3(int32_t* list, int32_t i, int32_t j)
{
	register int32_t *pi, *pj, tmp;
	pi = list + 3 * (i); pj = list + 3 * (j);

	tmp = *pi;
	*pi++ = *pj;
	*pj++ = tmp;

	tmp = *pi;
	*pi++ = *pj;
	*pj++ = tmp;

	tmp = *pi;
	*pi = *pj;
	*pj = tmp;
}



