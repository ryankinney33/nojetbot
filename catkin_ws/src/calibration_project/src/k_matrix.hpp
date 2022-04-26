#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>
#include <opencv2/core/eigen.hpp>

void intrinsic_CameraMatrix(const Eigen::MatrixXd& projection)
{
	Eigen::MatrixXd calc = projection.block(0,0,3,3);
	Eigen::MatrixXd K = Eigen::MatrixXd::Identity(calc.rows(),calc.cols());
	K = K / K(2,2);

	std::cout << "K : " << K << "\n";
	std::cout << "R:  " << calc << "\n";
	std::cout << "t: " << K.inverse() * projection.block(0,3,3,1) << "\n";
}
