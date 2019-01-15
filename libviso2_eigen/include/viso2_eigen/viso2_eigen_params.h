#ifndef VISO2_EIGEN_PARAMS_H
#define VISO2_EIGEN_PARAMS_H

#include <Eigen/Eigen>

namespace QuadMatcherParam{
    struct Parameters {
        // The width and height of the image sequences
        int image_width, image_height;

        // The number of rows and columns that each image will be bucketed into.
        // Generally the height of the each bucket should be smaller, to better imposed the epipolar constraints
        int n_bucket_width = 8, n_bucket_height = 8;
        int epipolar_offset = 0; // This is used when the two cameras' cv or Ty are not perfectly aligned
        int epipolar_tolerance = 10;
        double max_neighbor_ratio = 0.6;
        bool use_bucketing= true;
        int max_features_per_bucket = 20;
        bool compulte_scaled_keys = false;

        // This will be set automatically, do not change
        int bucket_height;
        int bucket_width;
    };
}

namespace StereoMotionEstimatorParam{

    // camera parameters (all are mandatory / need to be supplied)
    struct Calibration {
        double baseline = 1.0;             // baseline (meters)
        double fx = 1;                       // focal length (in pixels)
        double fy = 1;                       // focal length (in pixels)
        double cu = 0;                      // principal point (u-coordinate) aka width
        double cv = 0;                      // principal point (v-coordinate) aka height
    };

    // stereo-specific parameters (mandatory: base)
	struct Parameters{
		
		int ransac_iters = 400;     // number of RANSAC iterations
		double inlier_threshold = 4.0; // fundamental matrix inlier threshold
        double inlier_ratio_min = 0.3;  
		bool reweighting = true;      // lower border weights (more robust to calibration errors)
        int image_width;
        int image_height;

        double good_point_threshold_scale = 0.5;

        Calibration calib_left;
        Calibration calib_right;
	};

}

#endif /* VISO2_EIGEN_PARAMS_H */
