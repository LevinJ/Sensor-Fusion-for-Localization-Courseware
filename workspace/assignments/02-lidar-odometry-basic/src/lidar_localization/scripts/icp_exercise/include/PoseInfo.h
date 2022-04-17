/*
 * PoseInfo.h
 *
 *  Created on: Dec 25, 2020
 *      Author: levin
 */

#ifndef LOOP_FUSION_SRC_UTILITY_POSEINFO_H_
#define LOOP_FUSION_SRC_UTILITY_POSEINFO_H_

#include <eigen3/Eigen/Dense>
#include <vector>
class RyprTransform{
public:
	Eigen::Vector3d R2ypr(const Eigen::Matrix3d &R)
	{
		Eigen::Vector3d n = R.col(0);
		Eigen::Vector3d o = R.col(1);
		Eigen::Vector3d a = R.col(2);

		Eigen::Vector3d ypr(3);
		double y = atan2(n(1), n(0));
		double p = atan2(-n(2), n(0) * cos(y) + n(1) * sin(y));
		double r = atan2(a(0) * sin(y) - a(1) * cos(y), -o(0) * sin(y) + o(1) * cos(y));
		ypr(0) = y;
		ypr(1) = p;
		ypr(2) = r;

		return ypr;
	}

	template <typename Derived>
	Eigen::Matrix<typename Derived::Scalar, 3, 3> ypr2R(const Eigen::MatrixBase<Derived> &ypr)
	{
		typedef typename Derived::Scalar Scalar_t;

		Scalar_t y = ypr(0) / 180.0 * M_PI;
		Scalar_t p = ypr(1) / 180.0 * M_PI;
		Scalar_t r = ypr(2) / 180.0 * M_PI;

		Eigen::Matrix<Scalar_t, 3, 3> Rz;
		Rz << cos(y), -sin(y), 0,
				sin(y), cos(y), 0,
				0, 0, 1;

		Eigen::Matrix<Scalar_t, 3, 3> Ry;
		Ry << cos(p), 0., sin(p),
				0., 1., 0.,
				-sin(p), 0., cos(p);

		Eigen::Matrix<Scalar_t, 3, 3> Rx;
		Rx << 1., 0., 0.,
				0., cos(r), -sin(r),
				0., sin(r), cos(r);

		return Rz * Ry * Rx;
	}


};

class RotationInfo: public RyprTransform{
public:
	Eigen::Matrix3d R_;
	Eigen::Quaterniond q_;
	Eigen::Vector3d ypr_;
	std::string name_;
	std::string repr_;
	RotationInfo(std::string name = "rot"){
		name_ = name;
	}
	virtual ~RotationInfo(){

	}

	RotationInfo & construct_fromR(const Eigen::Matrix3d &R =  Eigen::Matrix3d::Identity()){
		R_ = R;
		q_ = R;
		ypr_ = R2ypr(R);
		std::stringstream ss;
		ss.precision(8);
		ss << std::fixed<<"name:"<<name_<<",ypr:"<<ypr_.transpose()  / M_PI * 180.0;
		repr_ = ss.str();
		return *this;
	}

	RotationInfo & construct_fromypr(const Eigen::Vector3d &ypr= Eigen::Vector3d::Zero()){
		auto R = ypr2R(ypr);
		construct_fromR(R);
		return *this;
	}
	RotationInfo &  construct_fromyprradian(const Eigen::Vector3d &ypr= Eigen::Vector3d::Zero()){
		Eigen::Vector3d ypr2 = ypr * 180.0/M_PI;
		return construct_fromypr(ypr2);
	}
	RotationInfo & construct_fromqt(const Eigen::Quaterniond &q= Eigen::Quaterniond::Identity()){
		construct_fromR(q.toRotationMatrix());
		return *this;
	}

	virtual RotationInfo& operator = (const RotationInfo& other) {
		R_ = other.R_;
		q_ = other.q_;
		ypr_ = other.ypr_;
		name_ = other.name_;
		repr_ = other.repr_;
		return *this;
	}

	virtual RotationInfo operator * (const RotationInfo& other) {
		RotationInfo temp;
		temp.construct_fromR(R_ * other.R_);
		return temp;
	}

	virtual RotationInfo  I() {
		RotationInfo temp;
		temp.construct_fromR(R_.inverse());
		return temp;
	}

};

class PoseInfo :public RyprTransform{
public:
	PoseInfo(std::string name = "pose"){
		name_ = name;
	}
	virtual ~PoseInfo(){

	}
	Eigen::Matrix3d R_;
	Eigen::Vector3d t_;
	Eigen::Matrix4d T_;
	Eigen::Quaterniond q_;
	Eigen::Vector3d ypr_;
	std::string name_;
	std::string repr_;

	PoseInfo &  construct_frmxyyaw_radian(const Eigen::Vector3d &x_y_yaw= Eigen::Vector3d::Zero() ){
		auto x = x_y_yaw[0], y = x_y_yaw[1],yaw = x_y_yaw[2];
		return construct_fromyprradian_t({yaw, 0, 0}, {x,y,0});
	}

	PoseInfo &  construct_frmxyyaw_radian_vec(const std::vector<double> x_y_yaw){
		const Eigen::Vector3d x_y_yaw2(x_y_yaw.data());
		return construct_frmxyyaw_radian(x_y_yaw2);
	}

	PoseInfo & construct_fromRt(const Eigen::Matrix3d &R =  Eigen::Matrix3d::Identity(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		R_ = R;
		t_ = t;
		T_ = Eigen::Matrix4d::Identity();
		T_.block<3, 3>(0, 0) = R;
		T_.block<3, 1>(0, 3) = t;
		q_ = R;
		ypr_ = R2ypr(R);
		std::stringstream ss;
		ss.precision(8);
		ss << std::fixed<<"name:"<<name_<<",t:" <<t_.transpose()<<",ypr:"<<ypr_.transpose()  / M_PI * 180.0;
		repr_ = ss.str();
		return *this;
	}

	PoseInfo & construct_fromT(const Eigen::Matrix4d &T =  Eigen::Matrix4d::Identity()){
		auto R = T.block<3, 3>(0, 0);
		auto t = T.block<3, 1>(0, 3);
		construct_fromRt(R, t);
		return *this;
	}

	PoseInfo & construct_fromyprt(const Eigen::Vector3d &ypr= Eigen::Vector3d::Zero(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		auto R = ypr2R(ypr);
		construct_fromRt(R, t);
		return *this;
	}

	PoseInfo &  construct_fromyprradian_t(const Eigen::Vector3d &ypr= Eigen::Vector3d::Zero(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		Eigen::Vector3d ypr2 = ypr * 180.0/M_PI;
		return construct_fromyprt(ypr2, t);
	}

	void  update_fromyprradian_t(){
		construct_fromyprradian_t(ypr_, t_);
	}
	void  update_fromqt(){
		construct_fromqt(q_, t_);
	}

	PoseInfo & construct_fromqt(const Eigen::Quaterniond &q= Eigen::Quaterniond::Identity(), const Eigen::Vector3d &t= Eigen::Vector3d::Zero()){
		construct_fromRt(q.toRotationMatrix(), t);
		return *this;
	}

	std::string get_short_desc(bool tree_dim = true){
		std::stringstream ss;
		ss.precision(8);
		if(tree_dim){
			ss << std::fixed<<name_<<",t:" <<t_.transpose()<<",ypr:"<<ypr_.transpose()  / M_PI * 180.0;
		}else{
			ss << std::fixed<<name_<<",t=[" <<t_.head(2).transpose()<<"],yaw="<<ypr_[0] / M_PI * 180.0;
		}
		return ss.str();
	}


	Eigen::Vector3d project_point(const Eigen::Vector3d &pnt){
		Eigen::Vector4d temp_pnt;
		temp_pnt << pnt[0],pnt[1],pnt[2],1;
		Eigen::Vector4d res = T_ * temp_pnt;
		return res.head<3>();
	}

	std::vector<double> project_point_vec(const std::vector<double> &pnt){
		Eigen::Vector3d pnt2 = {pnt[0], pnt[1], pnt[2]};
		auto res = project_point(pnt2);
		return {res[0], res[1],res[2]};
	}

	virtual PoseInfo& operator = (const PoseInfo& other) {
		R_ = other.R_;
		t_ = other.t_;
		T_ = other.T_;
		q_ = other.q_;
		ypr_ = other.ypr_;
//		name_ = other.name_;
		repr_ = other.repr_;
		return *this;
	}

	virtual PoseInfo operator * (const PoseInfo& other) {
		PoseInfo temp;
		temp.construct_fromT(T_ * other.T_);
		return temp;
	}

	virtual PoseInfo  I() {
		PoseInfo temp;
		temp.construct_fromT(T_.inverse());
		return temp;
	}

	std::vector<double> output_xy_yaw(){
		auto x = t_[0], y = t_[1];
		auto yaw = ypr_[0];
		return {x,y,yaw};
	}

	Eigen::Vector3d output_xy_yaw_eigen(){
		return Eigen::Vector3d(t_[0], t_[1], ypr_[0]);
	}


};

#endif /* LOOP_FUSION_SRC_UTILITY_POSEINFO_H_ */
