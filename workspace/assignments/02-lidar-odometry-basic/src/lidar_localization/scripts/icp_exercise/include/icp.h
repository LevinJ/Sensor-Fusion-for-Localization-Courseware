#pragma once


#include <vector>
#include <memory>
#include <map>

#include "nanoflann.hpp"
#include "nanoflann_utils.h"
#include "PoseInfo.h"

typedef nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, nanoflann::PointCloud<double> > ,
		nanoflann::PointCloud<double>,
        3 /* dim */
        > icp_kd_tree_t;

class Icp {
public:
	Icp();
	void setInputSource(std::vector<std::vector<double>> point_vec_ptr);
	void setInputTarget(std::vector<std::vector<double>> point_vec_ptr);
	void align(std::vector<double> initial_trans_rot={0, 0, 0, 0, 0, 0});
	void associate();
	virtual ~Icp();

	std::vector<std::vector<double>> src_point_vec_;
	std::vector<std::vector<double>> target_point_vec_;

	std::unique_ptr<icp_kd_tree_t> target_kdtree_ptr_;

	PoseInfo cur_tranform_;
	std::vector<std::vector<double>> assoctions_;
	void regiger_state_cb(const std::function<void(const std::string &, const std::vector<std::vector<double>> &, const PoseInfo &,
			const std::vector<std::map<std::string, std::vector<std::vector<double>>>>  &)> &f){
		state_cb_ = f;
	}
	bool print_ceres_log_;
private:
	std::function<void(const std::string &, const std::vector<std::vector<double>> &, const PoseInfo &,
			const std::vector<std::map<std::string, std::vector<std::vector<double>>>> &)> state_cb_;
	nanoflann::PointCloud<double> target_kdtree_pts_;
};
