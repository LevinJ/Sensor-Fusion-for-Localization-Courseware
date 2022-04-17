/*
 * CeresDebugInfo.h
 *
 *  Created on: May 12, 2021
 *      Author: levin
 */

#ifndef VSLAM_LOCALIZATION_SEMANTIC_SLAM_LIBS_SRC_CERESDEBUGINFO_H_
#define VSLAM_LOCALIZATION_SEMANTIC_SLAM_LIBS_SRC_CERESDEBUGINFO_H_

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include "ceres/residual_block.h"

#include <vector>
#include <map>
#include <string>

class CeresDebugCallback : public ceres::IterationCallback {
 public:
  virtual ~CeresDebugCallback() {}
  void compute_residuals(int iteration);

  ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary) {
	  compute_residuals(summary.iteration);

    return ceres::SOLVER_CONTINUE;
  }

  void reset(){
  	  residual_block_map_.clear();
  	  optimization_res_.clear();
    }

  void add_residual_block(std::string name, ceres::ResidualBlockId rbi){
	  if(residual_block_map_.find(name) != residual_block_map_.end()){
		  residual_block_map_[name].push_back(rbi);
	  }else{
		  residual_block_map_[name] = {rbi};
	  }
  }

  static CeresDebugCallback *get_instance(){
	  static CeresDebugCallback ceres_debug;
	  return &ceres_debug;
  }




 std::map<std::string, std::vector<ceres::ResidualBlockId>> residual_block_map_;
 std::vector<std::map<std::string, std::vector<std::vector<double>>> > optimization_res_;
};


class SqrtInfo{
public:
	SqrtInfo(const std::vector<double> &var):var_(var){};
	std::vector<double> get(){
		std::vector<double> res;
		for(auto &item: var_){
			res.push_back(sqrt(1/item));
		}
		return res;
	};

private:
	std::vector<double>  var_;
};
#endif /* VSLAM_LOCALIZATION_SEMANTIC_SLAM_LIBS_SRC_CERESDEBUGINFO_H_ */
