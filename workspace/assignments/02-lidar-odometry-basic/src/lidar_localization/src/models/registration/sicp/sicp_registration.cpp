/*
 * @Description: ICP SVD lidar odometry
 * @Author: Ge Yao
 * @Date: 2020-10-24 21:46:45
 */

#include <nanoflann.hpp>
#include <pcl/common/transforms.h>

#include <Eigen/Dense>

#include "glog/logging.h"

#include "lidar_localization/models/registration/sicp/ICP.h"

#include "lidar_localization/models/registration/sicp/scip_registration.hpp"

//using nanoflann::pi_const;
//#include "nanoflann_utils.h"
#include "icp.h"
using namespace nanoflann;

namespace lidar_localization {


static void cloud2vectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<std::vector<double>> &vec){
	vec.clear();
	int n = cloud->size();
	vec.resize(n);
	for(int i=0; i< n; i++){
		vec[i] = {cloud->points[i].x, cloud->points[i].y,cloud->points[i].z};
	}

}
SICPRegistration::SICPRegistration(
    const YAML::Node& node
) {
    // parse params:

//    params_.p = node['p'].as<float>();
//    params_.mu = node['mu'].as<float>();
//    params_.alpha = node['alpha'].as<float>();
//    params_.max_mu = node['max_mu'].as<float>();
//    params_.max_icp = node['max_icp'].as<int>();
//    params_.max_outer = node['max_outer'].as<int>();
//    params_.max_inner = node['max_inner'].as<int>();
//    params_.stop = node['stop'].as<float>();

}

bool SICPRegistration::SetInputTarget(const CloudData::CLOUD_PTR& input_target) {
    input_target_ = input_target;

    return true;
}

bool SICPRegistration::ScanMatch(
    const CloudData::CLOUD_PTR& input_source, 
    const Eigen::Matrix4f& predict_pose, 
    CloudData::CLOUD_PTR& result_cloud_ptr,
    Eigen::Matrix4f& result_pose
) {
    input_source_ = input_source;

    // pre-process input source:
//    CloudData::CLOUD_PTR transformed_input_source(new CloudData::CLOUD());
//    pcl::transformPointCloud(*input_source_, *transformed_input_source, predict_pose);

    Icp icp;

    auto &cloud_in = input_source_;
    auto &cloud_out = input_target_;

    std::vector<std::vector<double>> src_vec_ptr;
    std::vector<std::vector<double>> target_vec_ptr;
    cloud2vectors(cloud_in, src_vec_ptr);
	cloud2vectors(cloud_out, target_vec_ptr);
	icp.setInputSource(src_vec_ptr);
	icp.setInputTarget(target_vec_ptr);

	std::vector<double> initial_trans_rot(6);
	PoseInfo Tts;
	Tts.construct_fromT(predict_pose.cast<double>());

	initial_trans_rot[0] = Tts.t_[0];
	initial_trans_rot[1] = Tts.t_[1];
	initial_trans_rot[2] = Tts.t_[2];

	initial_trans_rot[3] = Tts.ypr_[0];
	initial_trans_rot[4] = Tts.ypr_[1];
	initial_trans_rot[5] = Tts.ypr_[2];

	icp.align(initial_trans_rot);
//	std::cout<<icp.cur_tranform_.t_<<endl;
//	std::cout<<icp.cur_tranform_.ypr_ * 180/M_PI <<endl;

	result_pose = icp.cur_tranform_.T_.cast<float>();

    //
    // TODO: second option -- adapt existing implementation
    //
    // TODO: format inputs for SICP:
    
    // TODO: SICP registration:

    // set output:
//    result_pose = transformation_ * predict_pose;
    pcl::transformPointCloud(*input_source_, *result_cloud_ptr, result_pose);
    
    return true;
}

} // namespace lidar_localization
