//class ICP_QUATERION;

#include "icp.h"

#include <vector>
#include <iostream>
#include <cmath>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include "PoseInfo.h"

using namespace std;

int test_pcl(const std::vector<std::vector<double>> &src_vec_ptr, const std::vector<std::vector<double>> &target_vec_ptr,
		pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_in, pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud_out){



	const int n = src_vec_ptr.size();
	// Fill in the CloudIn data
	//	for (auto& point : *cloud_in)
	//	{
	//		point.x = 1024 * rand() / (RAND_MAX + 1.0f);
	//		point.y = 1024 * rand() / (RAND_MAX + 1.0f);
	//		point.z = 1024 * rand() / (RAND_MAX + 1.0f);
	//	}

	//	std::cout << "Saved " << cloud_in->size () << " data points to input:" << std::endl;

	for(int i=0; i< n; i++){
		auto& point = cloud_in->points[i];
		point.x = src_vec_ptr[i][0];
		point.y = src_vec_ptr[i][1];
		point.z = src_vec_ptr[i][2];
	}

	for (auto& point : *cloud_in)
		std::cout << point << std::endl;



	std::cout << "size:" << cloud_out->size() << std::endl;
	for(int i=0; i< n; i++){
		auto& point = cloud_out->points[i];
		point.x = target_vec_ptr[i][0];
		point.y = target_vec_ptr[i][1];
		point.z = target_vec_ptr[i][2];
	}

	std::cout << "Transformed " << cloud_in->size () << " data points:" << std::endl;

	for (auto& point : *cloud_out)
		std::cout << point << std::endl;

	pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
	icp.setInputSource(cloud_in);
	icp.setInputTarget(cloud_out);

	pcl::PointCloud<pcl::PointXYZ> Final;
	icp.align(Final);

	std::cout << "has converged:" << icp.hasConverged() << " score: " <<
			icp.getFitnessScore() << std::endl;
	std::cout << icp.getFinalTransformation() << std::endl;
	return 0;
}

void cloud2vectors(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, std::vector<std::vector<double>> &vec){
	vec.clear();
	int n = cloud->size();
	vec.resize(n);
	for(int i=0; i< n; i++){
		vec[i] = {cloud->points[i].x, cloud->points[i].y,cloud->points[i].z};
	}

}
int main(int argc, char **argv)
{

	Icp icp;

	std::vector<std::vector<double>> src_vec_ptr;
	std::vector<std::vector<double>> target_vec_ptr;

	src_vec_ptr.push_back({1, 1, 1});
	src_vec_ptr.push_back({2, 5, 2});
	src_vec_ptr.push_back({3, 3, 3});


	Eigen::Vector3d ypr = {2, 1.234, 0.12345};
	Eigen::Vector3d t = {0.5, 0.2, 0.0007};
	PoseInfo Tts = PoseInfo().construct_fromyprt(ypr, t);

	for(auto &pnt : src_vec_ptr){
		auto np = Tts.project_point_vec(pnt);
		target_vec_ptr.push_back(np);
	}


//	target_vec_ptr.push_back({1.1, 1.2, 1.003});
//	target_vec_ptr.push_back({2.1, 5.2, 2.003});
//	target_vec_ptr.push_back({ 3.1, 3.2, 3.003});

	const int n = src_vec_ptr.size();
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in (new pcl::PointCloud<pcl::PointXYZ>(n,1));
	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZ>(n,1));
	test_pcl(src_vec_ptr, target_vec_ptr, cloud_in, cloud_out);

	cloud2vectors(cloud_in, src_vec_ptr);
	cloud2vectors(cloud_out, target_vec_ptr);
	icp.setInputSource(src_vec_ptr);
	icp.setInputTarget(target_vec_ptr);

	icp.align();
	std::cout<<icp.cur_tranform_.t_<<endl;
	std::cout<<icp.cur_tranform_.ypr_ * 180/M_PI <<endl;

	return 0;
}
