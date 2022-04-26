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

Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i, const std::vector<Eigen::Vector3d>& u_i)
{
	if(X_i.size() < 6 || u_i.size() < 6){
		std::cerr << "Need at least 6 points to find the projection matrix\n";
		throw std::runtime_error("Need more points");
	}

	// P matrix is 2n * 12 large
	Eigen::MatrixXd A(2*u_i.size(), 12);
	A.setZero();
	int row = 0;
	for(int i = 0; i < u_i.size(); ++i){
		A.block(row, 4, 1, 4) = -u_i.at(i)(2) * X_i.at(i).transpose();
		A.block(row, 8, 1, 4) = u_i.at(i)(1) * X_i.at(i).transpose();
		A.block(row+1, 0, 1, 4) = u_i.at(i)(2) * X_i.at(i).transpose();
		A.block(row+1, 8, 1, 4) = -u_i.at(i)(0) * X_i.at(i).transpose();
		row += 2;
	}

	std::cout << "A\n" << A << std::endl;
/*	auto svd  = Projection_Matrix.jacobiSvd(Eigen::ComputeFullV);

	Eigen::VectorXd  nullspace = svd.matrixV().col(X_i.size()-1);

	Eigen::MatrixXd projection(u_i.size(),X_i.size());
	projection.row(0) = nullspace.block(0,0,3,1).transpose();
	projection.row(1) = nullspace.block(3,0,3,1).transpose();
	projection.row(2) = nullspace.block(6,0,3,1).transpose();
*/
//	return projection;
	return A;
}
