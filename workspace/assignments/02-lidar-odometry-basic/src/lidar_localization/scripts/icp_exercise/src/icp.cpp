#include "icp.h"
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/local_parameterization.h>
#include <eigen3/Eigen/Dense>
#include <string>
#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "CeresDebugInfo.h"
//#include "vins_rotation.h"
//#include "pose_graph_factors.h"
#include <sstream>


class ICPFactorQueterion
{
public:

	ICPFactorQueterion(double obs_src_x, double obs_src_y, double obs_src_z, double obs_target_x, double obs_target_y,double obs_target_z,
			Eigen::Matrix3d sqrt_information, double weight=1.0):obs_src_x_(obs_src_x), obs_src_y_(obs_src_y), obs_src_z_(obs_src_z),
				   obs_target_x_(obs_target_x),obs_target_y_(obs_target_y),obs_target_z_(obs_target_z),
				   sqrt_information_(sqrt_information),weight_(weight){}

	template <typename T>
	bool operator()(const T* const q_param, const T* t_param, T* residuals) const
	{
		const Eigen::Matrix<T, 3, 1> t(t_param[0], t_param[1], t_param[2]);
		Eigen::Map<const Eigen::Quaternion<T> > q(q_param);


		const Eigen::Vector3d obs_src_t(obs_src_x_, obs_src_y_, obs_src_z_);
		const Eigen::Vector3d obs_target_t(obs_target_x_, obs_target_y_, obs_target_z_);

//		Eigen::AngleAxisd rotation_vector_src (obs_src_yaw_, Eigen::Vector3d ( 0,0,1 ) );
//		Eigen::Quaterniond obs_src_q = Eigen::Quaterniond ( rotation_vector_src );
//
//		Eigen::AngleAxisd rotation_vector_target (obs_target_yaw_, Eigen::Vector3d ( 0,0,1 ) );
//		Eigen::Quaterniond obs_target_q = Eigen::Quaterniond ( rotation_vector_target );

		Eigen::Map<Eigen::Matrix<T, 3, 1> > residuals_map(residuals);

		residuals_map.template head<3>() =
				(q * obs_src_t.cast<T>() + t - obs_target_t.cast<T>()).head(3);

//		Eigen::Quaternion<T> expected_target_q = q * obs_src_q.cast<T>();
//		Eigen::Quaternion<T> delta_q =
//				expected_target_q.conjugate() * obs_target_q.cast<T>();
//
//		residuals_map.template block<3, 1>(2, 0) = T(2.0) * delta_q.vec();

//		//weight these constraints
		residuals_map = residuals_map * weight_;
//		// Scale the residuals by the square root information matrix to account for
//		// the measurement uncertainty.
		residuals_map = sqrt_information_.template cast<T>() * residuals_map;

		return true;
	}

	static ceres::CostFunction* Create(double obs_src_x, double obs_src_y, double obs_src_z,
			double obs_target_x, double obs_target_y,double obs_target_z,
			Eigen::Matrix3d sqrt_information, double weight=1.0)
	{
	  return (new ceres::AutoDiffCostFunction<
			  ICPFactorQueterion, 3, 4, 3>(
	          	new ICPFactorQueterion(obs_src_x, obs_src_y, obs_src_z,obs_target_x, obs_target_y,obs_target_z,
	        			sqrt_information, weight)));
	}

	double obs_src_x_, obs_src_y_,obs_src_z_;
	double obs_target_x_, obs_target_y_,obs_target_z_;
	Eigen::Matrix3d  sqrt_information_;
	double weight_;
};




Icp::Icp() {
	// TODO Auto-generated constructor stub
	print_ceres_log_ = false;

}

void Icp::setInputSource(std::vector<std::vector<double>> point_vec){
	src_point_vec_ = point_vec;

}

void Icp::setInputTarget(std::vector<std::vector<double>> point_vec){
	target_point_vec_ = point_vec;
	size_t N = point_vec.size();
	target_kdtree_pts_.pts.resize(N);
	for(int i=0; i< N; i++){
		auto &pnt = target_point_vec_[i];
		target_kdtree_pts_.pts[i].x = pnt[0];
		target_kdtree_pts_.pts[i].y = pnt[1];
		target_kdtree_pts_.pts[i].z = pnt[2];
	}

	target_kdtree_ptr_.reset(new icp_kd_tree_t(3 /*dim*/, target_kdtree_pts_, nanoflann::KDTreeSingleIndexAdaptorParams(1 /* max leaf */)));
	target_kdtree_ptr_->buildIndex();
}
Icp::~Icp() {
	// TODO Auto-generated destructor stub
}

void Icp::align(std::vector<double> initial_trans_rot){
	Eigen::Vector3d t = {initial_trans_rot[0], initial_trans_rot[1], initial_trans_rot[2]};
	Eigen::Vector3d ypr = {initial_trans_rot[3], initial_trans_rot[4], initial_trans_rot[5]};
	cur_tranform_.construct_fromyprradian_t(ypr, t);

	PoseInfo last_transform;
	last_transform = cur_tranform_;

	for(int i=0; i< 10; i++){
		CeresDebugCallback::get_instance()->reset();
		associate();
		if(state_cb_){
			std::stringstream ss;
			ss<<"iter_"<<i<<"_before";
			state_cb_(ss.str(), assoctions_, cur_tranform_,
					CeresDebugCallback::get_instance()->optimization_res_);
		}
		// Build The Problem
		ceres::Problem problem;
		ceres::Solver::Options options;
		options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
		options.callbacks.push_back(CeresDebugCallback::get_instance());
		options.minimizer_progress_to_stdout = print_ceres_log_;
		//options.max_solver_time_in_seconds = SOLVER_TIME * 3;
		options.max_num_iterations = 100;
		ceres::Solver::Summary summary;
		ceres::LossFunction *loss_function;
		loss_function = new ceres::HuberLoss(1.0);
		//loss_function = new ceres::CauchyLoss(1.0);
//		ceres::LocalParameterization* angle_local_parameterization =  AngleLocalParameterization::Create();

		ceres::LocalParameterization* quaternion_local_parameterization =  new ceres::EigenQuaternionParameterization;

		//Add parameter blocks
//		problem.AddParameterBlock(cur_tranform_.ypr_.data(), 1, angle_local_parameterization);
		problem.AddParameterBlock(cur_tranform_.t_.data(), 3);

		problem.AddParameterBlock(cur_tranform_.q_.coeffs().data(), 4, quaternion_local_parameterization);

		double *t_array = cur_tranform_.t_.data();
//		double  *euler_array = cur_tranform_.ypr_.data();
		double  *euler_array = cur_tranform_.q_.coeffs().data();

		//add edges

		for(int src_ind=0; src_ind<assoctions_.size(); src_ind++){
			int target_ind = int(assoctions_[src_ind][0]);
			if (target_ind == -1){
				//no match is found for this source point
				continue;
			}

			auto &src_pt = src_point_vec_[src_ind];
//			double edge_type = 4;
			double obs_src_x = src_pt[0];
			double obs_src_y = src_pt[1];
			double obs_src_z = src_pt[2];

			auto &target_pt = target_point_vec_[target_ind];
			double obs_target_x = target_pt[0];
			double obs_target_y = target_pt[1];
			double obs_target_z = target_pt[2];

//			if(edge_type == 1){
//				//point to point constraint
//				Eigen::DiagonalMatrix<double, 2> cov(0.1, 0.1);
//				Eigen::Matrix2d  sqrt_information = cov;
//				sqrt_information = sqrt_information.inverse().llt().matrixL();
//				double weight = 1.0;
//				ceres::CostFunction* cost_function = RelPoseError::Create(obs_src_x, obs_src_y, obs_target_x, obs_target_y,
//						sqrt_information, weight);
//				ceres::ResidualBlockId rbi = problem.AddResidualBlock(cost_function, loss_function, euler_array, t_array);
//				CeresDebugCallback::get_instance()->add_residual_block("pp_xy", rbi);
//
//			}else if(edge_type == 2){
//				//point to line constraint
//				Eigen::Matrix<double, 1, 1>  sqrt_information;
//				sqrt_information(0, 0) = sqrt(1/0.1);
//				double weight = 1.0;
//				double obs_target_norm_x = target_pt[4];
//				double obs_target_norm_y = target_pt[5];
//				ceres::CostFunction* cost_function = RelPose_PL_Error::Create(obs_src_x, obs_src_y, obs_target_x, obs_target_y,
//						obs_target_norm_x, obs_target_norm_y,
//						sqrt_information, weight);
//				ceres::ResidualBlockId rbi = problem.AddResidualBlock(cost_function, loss_function, euler_array, t_array);
//				CeresDebugCallback::get_instance()->add_residual_block("pl", rbi);
//			}else if(edge_type == 3){
//				//point to point constraint
//				double weight = 1.0;
//				double obs_src_yaw = src_pt[4];
//				double obs_target_yaw = target_pt[4];
//
//
//				Eigen::DiagonalMatrix<double, 3> cov(0.1, 0.1, 1 * M_PI/180.0 *(1 * M_PI/180.0));
//				Eigen::Matrix3d  sqrt_information = cov;
//				sqrt_information = sqrt_information.inverse().llt().matrixL();
//				ceres::CostFunction* cost_function = RelPose_FF_Error::Create(obs_src_x, obs_src_y, obs_src_yaw,
//						obs_target_x, obs_target_y, obs_target_yaw,
//						sqrt_information, weight);
//
////				const double angle_var = 1 * M_PI/180.0 *(1 * M_PI/180.0);
////				Eigen::DiagonalMatrix<double, 5> cov;
////				cov.diagonal() << 0.1, 0.1,angle_var,angle_var,angle_var;
////				Eigen::Matrix<double, 5, 5>  sqrt_information = cov;
////				sqrt_information = sqrt_information.inverse().llt().matrixL();
////				ceres::CostFunction* cost_function = RelPose_FF_QUATERION_Error::Create(obs_src_x, obs_src_y, obs_src_yaw,
////										obs_target_x, obs_target_y, obs_target_yaw,
////										sqrt_information, weight);
//
//
//				ceres::ResidualBlockId rbi = problem.AddResidualBlock(cost_function, loss_function, euler_array, t_array);
//				CeresDebugCallback::get_instance()->add_residual_block("pp_xy_yaw", rbi);
//
//			}



			//point to point constraint
			double weight = 1.0;
//			double obs_src_yaw = src_pt[4];
//			double obs_target_yaw = target_pt[4];


			Eigen::DiagonalMatrix<double, 3> cov(0.1, 0.1, 0.1);
			Eigen::Matrix3d  sqrt_information = cov;
			sqrt_information = sqrt_information.inverse().llt().matrixL();
			ceres::CostFunction* cost_function = ICPFactorQueterion::Create(obs_src_x, obs_src_y, obs_src_z,
					obs_target_x, obs_target_y, obs_target_z,
					sqrt_information, weight);
			ceres::ResidualBlockId rbi = problem.AddResidualBlock(cost_function, loss_function, euler_array, t_array);
			CeresDebugCallback::get_instance()->add_residual_block("pp_xyz", rbi);



		}//end, add edge

		//Solve the problem
		CeresDebugCallback::get_instance()->compute_residuals(-1);
		ceres::Solve(options, &problem, &summary);
		if(print_ceres_log_){
			std::cout << summary.FullReport() << '\n';
			std::cout << summary.BriefReport() << "\n";
		}
		//update current transformation, ypr and t are updated by optimizer
		//we need to update the poseinfo object as well
//		cur_tranform_.update_fromqt();
		cur_tranform_.update_fromqt();

		if(state_cb_){
			std::stringstream ss;
			ss<<"iter_"<<i<<"_after";
			state_cb_(ss.str(), assoctions_, cur_tranform_,
					CeresDebugCallback::get_instance()->optimization_res_);
		}
		//check if we have converged
		PoseInfo estimation_change = last_transform.I() * cur_tranform_;

		last_transform = cur_tranform_;
		if(estimation_change.t_.norm() < 1e-5 && estimation_change.ypr_.norm()< 1e-5){
			break;
		}


	}//end iteration (association and optimization cycles)
}
void Icp::associate(){
	assoctions_.clear();


	for(int i=0; i<src_point_vec_.size(); i++ ){
		if(target_point_vec_.size() == 0){
			assoctions_.push_back({-1, -1});
			continue;
		}
		auto &src_pt = src_point_vec_[i];
		double src_x = src_pt[0];
		double src_y = src_pt[1];
		double src_z = src_pt[2];
//		int obs_src_clsid = int(src_pt[1]);

		Eigen::Vector3d query_pt = cur_tranform_.project_point({src_x, src_y, src_z});
		// do a knn search
		const size_t num_results = 1;
		size_t ret_index;
		double out_dist_sqr;
		nanoflann::KNNResultSet<double> resultSet(num_results);
		resultSet.init(&ret_index, &out_dist_sqr );
		target_kdtree_ptr_->findNeighbors(resultSet, &query_pt[0], nanoflann::SearchParams(32,0,false));
//		auto &target_pt = target_point_vec_[ret_index];
//		int obs_target_clsid = int(target_pt[1]);

		//std::cout << "knnSearch(nn="<<num_results<<"): \n";
		//std::cout << "ret_index=" << ret_index << " out_dist_sqr=" << out_dist_sqr << endl;
//		if(obs_src_clsid != obs_target_clsid){
//			assoctions_.push_back({-1, -1});
//			continue;
//		}
		assoctions_.push_back({double(ret_index), out_dist_sqr});
	}
}

