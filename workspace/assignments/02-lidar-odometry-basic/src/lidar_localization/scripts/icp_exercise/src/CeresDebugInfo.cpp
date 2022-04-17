/*
 * CeresDebugInfo.cpp
 *
 *  Created on: May 12, 2021
 *      Author: levin
 */

#include "CeresDebugInfo.h"

void CeresDebugCallback::compute_residuals(int iteration){
	 return;
	  std::map<std::string, std::vector<std::vector<double>>> res;
	  for (const auto &item : residual_block_map_){

		  auto &k = item.first;
		  auto &v = item.second;
		  for(const auto &rbi : v){
			  bool apply_loss_function = true;
			  double cost = -1;
			  double residuals[rbi->NumResiduals()];
			  double** jacobians = NULL;
			  double scratch [rbi->NumScratchDoublesForEvaluate()];
			  bool flag = rbi->Evaluate(apply_loss_function,
						&cost,
						residuals,
						jacobians,
						scratch);
			  std::vector<double> cur_res;
			  cur_res.push_back(double(iteration));
			  cur_res.push_back(double(rbi->index()));
			  cur_res.push_back(cost);
			  for(int i=0; i<rbi->NumResiduals(); i++ ){
				  cur_res.push_back(residuals[i]);
			  }
			  if(res.find(k) != res.end()){
				  res[k].push_back(cur_res);
			  }else{
				  res[k] = {cur_res};
			  }
		  }

	  }

	  optimization_res_.push_back(res);
  }
