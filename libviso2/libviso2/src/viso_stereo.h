#ifndef VISO_STEREO_H
#define VISO_STEREO_H

#include "viso.h"

class VisualOdometryStereo : public VisualOdometry {

public:

	// stereo-specific parameters (mandatory: base)
	struct parameters : public VisualOdometry::parameters {
		double  base;             // baseline (meters)
		int32_t ransac_iters;     // number of RANSAC iterations
		double  inlier_threshold; // fundamental matrix inlier threshold
		bool    reweighting;      // lower border weights (more robust to calibration errors)
		parameters() {
			base = 1.0;
			ransac_iters = 400;
			inlier_threshold = 1.5;		// yx: default is 2
			reweighting = true;
		}
	};

	// constructor, takes as inpute a parameter structure
	VisualOdometryStereo(parameters param);

	// deconstructor
	~VisualOdometryStereo();

	// process a new images, push the images back to an internal ring buffer.
	// valid motion estimates are available after calling process for two times.
	// inputs: I1 ........ pointer to rectified left image (uint8, row-aligned)
	//         I2 ........ pointer to rectified right image (uint8, row-aligned)
	//         dims[0] ... width of I1 and I2 (both must be of same size)
	//         dims[1] ... height of I1 and I2 (both must be of same size)
	//         dims[2] ... bytes per line (often equal to width)
	// output: returns false if an error occured
	bool process(uint8_t *I1, uint8_t *I2, int32_t* dims, bool replace=false);

	using VisualOdometry::process;

protected:

	// for half_resolution
	double _cu, _cv, _f;

private:

	std::vector<double>  estimateMotion(std::vector<Matcher::p_match> p_matched);
	enum                 result { UPDATED, FAILED, CONVERGED };
	result               updateParameters(std::vector<Matcher::p_match> &p_matched, std::vector<int32_t> &active, std::vector<double> &tr, double step_size, double eps);
	void                 computeObservations(std::vector<Matcher::p_match> &p_matched, std::vector<int32_t> &active);
	void                 computeResidualsAndJacobian(std::vector<double> &tr, std::vector<int32_t> &active);
	std::vector<int32_t> getInlier(std::vector<Matcher::p_match> &p_matched, std::vector<double> &tr);

	double *X, *Y, *Z;    // 3d points
	double *p_residual; // residuals (p_residual=p_observe-p_predict)

	// parameters
	parameters param;
};

#endif // VISO_STEREO_H

