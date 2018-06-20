# viso2
==========
ROS Stack containing a wrapper for libviso2, a visual odometry library. 
http://www.ros.org/wiki/viso2 for the list of contained packages.

## Modification Done
- Pre-processing of image (OpenCV convertto() function, with linear contrast and brightness change).
- Use of native full resolution for best FOV and image clarity, with OpenCV subsampling with 0.5 scale (processing time ~0.07s).
- Re-write bucketing matches algorithm, to perform bucketing at the same time of the matching process (speed-up). Exposed paramters to "bucket_ornot" and "max_features", bucket size is the same as the "match_binsize".
- loosen the constraint of quad matching to tolerate an error up to 2 pixels in u or v direction.
- OpenCV visualisation added for stereo matched points.

## TODO
- add variable covariance based on number of matches (>50 ?) & inlier percentage (>60% ?)
- rectify realsense stereo cameras' vignetting
- if needed implement multithreading for viso, to speed up
