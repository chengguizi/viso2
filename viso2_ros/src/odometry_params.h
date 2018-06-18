#include <ros/ros.h>

#include <viso_stereo.h>

namespace viso2_ros
{

namespace odometry_params
{
 
/// loads matcher params
void loadParams(const ros::NodeHandle& local_nh, Matcher::parameters& params)
{
	local_nh.getParam("match_binsize",          params.match_binsize);
	local_nh.getParam("match_radius",           params.match_radius);
	local_nh.getParam("match_disp_tolerance",   params.match_disp_tolerance);
	local_nh.getParam("outlier_disp_tolerance", params.outlier_disp_tolerance);
	local_nh.getParam("outlier_flow_tolerance", params.outlier_flow_tolerance);
	local_nh.getParam("multi_stage",            params.multi_stage);
	local_nh.getParam("half_resolution",        params.half_resolution);
	local_nh.getParam("remove_outliers",        params.remove_outliers);
	local_nh.getParam("refinement",             params.refinement);
	local_nh.getParam("fast_threshold_dense",   params.fast_threshold_dense);
	local_nh.getParam("fast_threshold_sparse",  params.fast_threshold_sparse);
	local_nh.getParam("numFastFeature_dense",   params.numFastFeature_dense);
	local_nh.getParam("numFastFeature_sparse",  params.numFastFeature_sparse);
}

/// loads bucketing params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometry::bucketing& bucketing)
{
	local_nh.getParam("max_features",  bucketing.max_features);
	local_nh.getParam("bucket_width",  bucketing.bucket_width);
	local_nh.getParam("bucket_height", bucketing.bucket_height);
}

/// loads common odometry params
void loadCommonParams(const ros::NodeHandle& local_nh, VisualOdometry::parameters& params)
{
	loadParams(local_nh, params.match);
	loadParams(local_nh, params.bucket);
}

/// loads common & stereo specific params
void loadParams(const ros::NodeHandle& local_nh, VisualOdometryStereo::parameters& params)
{
	loadCommonParams(local_nh, params);
	local_nh.getParam("ransac_iters",     params.ransac_iters);
	local_nh.getParam("inlier_threshold", params.inlier_threshold);
	local_nh.getParam("reweighting",      params.reweighting);
}

} // end of namespace

std::ostream& operator<<(std::ostream& out, const Matcher::parameters& params)
{
	out << "  match_binsize          = " << params.match_binsize << std::endl;
	out << "  match_radius           = " << params.match_radius << std::endl;
	out << "  match_disp_tolerance   = " << params.match_disp_tolerance << std::endl;
	out << "  outlier_disp_tolerance = " << params.outlier_disp_tolerance << std::endl;
	out << "  outlier_flow_tolerance = " << params.outlier_flow_tolerance << std::endl;
	out << "  multi_stage            = " << params.multi_stage << std::endl;
	out << "  half_resolution        = " << params.half_resolution << std::endl;
	out << "  remove_outliers        = " << params.remove_outliers << std::endl;
	out << "  refinement             = " << params.refinement << std::endl;
	out << "  fast_threshold_dense   = " << params.fast_threshold_dense << std::endl;
	out << "  fast_threshold_sparse  = " << params.fast_threshold_sparse << std::endl;
	out << "  numFastFeature_dense   = " << params.numFastFeature_dense << std::endl;
	out << "  numFastFeature_sparse  = " << params.numFastFeature_sparse << std::endl;
	return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::calibration& calibration)
{
	out << "  f  = " << calibration.f << std::endl;
	out << "  cu = " << calibration.cu << std::endl;
	out << "  cv = " << calibration.cv << std::endl;
	return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::bucketing& bucketing)
{
	out << "  bucket_ornot  = " << bucketing.bucket_ornot << std::endl;
	out << "  max_features  = " << bucketing.max_features << std::endl;
	out << "  bucket_width  = " << bucketing.bucket_width << std::endl;
	out << "  bucket_height = " << bucketing.bucket_height << std::endl;
	return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometry::parameters& params)
{
	out << "Calibration parameters:" << std::endl << params.calib;
	out << "Matcher parameters:" << std::endl << params.match;
	out << "Bucketing parameters:" << std::endl << params.bucket;
	return out;
}

std::ostream& operator<<(std::ostream& out, const VisualOdometryStereo::parameters& params)
{
	out << static_cast<VisualOdometry::parameters>(params);
	out << "Stereo odometry parameters:" << std::endl;
	out << "  base             = " << params.base << std::endl;
	out << "  ransac_iters     = " << params.ransac_iters << std::endl;
	out << "  inlier_threshold = " << params.inlier_threshold << std::endl;
	out << "  reweighting      = " << params.reweighting << std::endl;
	return out;
}


} // end of namespace

