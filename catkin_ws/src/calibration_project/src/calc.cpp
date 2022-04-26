#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>

#include "calc.hpp"

void intrinsic_CameraMatrix(const Eigen::MatrixXd& projection)
{
	Eigen::MatrixXd calc = projection.block(0,0,3,3);
	Eigen::MatrixXd K = Eigen::MatrixXd::Identity(calc.rows(),calc.cols());
	K = K / K(2,2);

	std::cout << "K : " << K << "\n";
	std::cout << "R:  " << calc << "\n";
	std::cout << "t: " << K.inverse() * projection.block(0,3,3,1) << "\n";
}

//Small issues I need the corresponding 2d image points and 3d world points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i, const std::vector<Eigen::Vector3d>& u_i)
{
	if(X_i.size() < 6 || u_i.size() < 6){
		std::cerr << "Need at least 6 points to find the projection matrix\n";
		throw std::runtime_error("Need more points");
	}
	Eigen::MatrixXd Projection_Matrix(2*u_i.size(), X_i.size() * u_i.size());
	Projection_Matrix.setZero();
	for(int i = 0; i < u_i.size(); ++i){
		Projection_Matrix.block(2*i, X_i.size(),1,X_i.size()) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix.block(2*i, X_i.size() * 2,1,X_i.size()) = X_i[i](1) * u_i[i].transpose();
		Projection_Matrix.block(2*i + 1, 0,1,X_i.size()) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix.block(2*i + 1, X_i.size() * 2,1,X_i.size()) = X_i[i](0) * u_i[i].transpose();


	}
	auto svd  = Projection_Matrix.jacobiSvd(Eigen::ComputeFullV);

	Eigen::VectorXd  nullspace = svd.matrixV().col(X_i.size()-1);

	Eigen::MatrixXd projection(u_i.size(),X_i.size());
	projection.row(0) = nullspace.block(0,0,3,1).transpose();
	projection.row(1) = nullspace.block(3,0,3,1).transpose();
	projection.row(2) = nullspace.block(6,0,3,1).transpose();

	return projection;
}
