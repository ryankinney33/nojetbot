#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>

#include "calc.hpp"

// Perform Gram-Schmidt process to factor M into K and R
static std::pair<Eigen::Matrix3d, Eigen::Matrix3d> gramschmidt(const Eigen::Matrix3d &M)
{
	// Initialize K and R
	Eigen::Matrix3d R = Eigen::Matrix3d::Zero();
	Eigen::Matrix3d K = Eigen::Matrix3d::Zero();

	// Break up M into 3 vectors
	Eigen::Vector3d c = M.row(0);
	Eigen::Vector3d b = M.row(1);
	Eigen::Vector3d a = M.row(2);

	std::cout << "\nc\n" << c << std::endl;
	std::cout << "\nb\n" << b << std::endl;
	std::cout << "\na\n" << a << std::endl;

	// create the unit vectors r1, r2, r3
	auto r1 = a.normalized();
	auto r2 = (b - (b.transpose()*r1)*r1).normalized();
	auto r3 = (c - (c.transpose()*r2)*r2 - (c.transpose()*r1)*r1).normalized();
	std::cout << "\nr3\n" << r3 << std::endl;
	std::cout << "\nr2\n" << r2 << std::endl;
	std::cout << "\nr1\n" << r1 << std::endl;

	R.row(0) = r3.transpose();
	R.row(1) = r2.transpose();
	R.row(2) = r1.transpose();

	// Top row of K
	K(0,0) = c.transpose()*r3;
	K(0,1) = c.transpose()*r2;
	K(0,2) = c.transpose()*r1;
	// Middle row
	K(1,1) = b.transpose()*r2;
	K(1,2) = b.transpose()*r1;
	// Bottom row
	K(2,2) = a.transpose()*r1;

	// no more work to be done
	return std::make_pair(K, R);
}

void find_k(const Eigen::MatrixXd& P)
{
	Eigen::Matrix3d M = P.leftCols(3);
	Eigen::Matrix3d K, R;
	std::tie(K, R) = gramschmidt(M);

	// "Normalize" the K matrix
	K /= K(2,2);

	// Calculate "t"
	auto  t = K.inverse() * P.rightCols(1);

	// Print the information
	std::cout << "K =\n" << K;
	std::cout << "\nR =\n" << R;
	std::cout << "\nt =\n" << t << std::endl;
}

Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i, const std::vector<Eigen::Vector3d>& u_i)
{
	if(X_i.size() < 6 || u_i.size() < 6){
		std::cerr << "Need at least 6 points to find the projection matrix\n";
		throw std::runtime_error("Need more points");
	}

	// Dimensions of the X and u vectors
	int x_dim = 4;
	int u_dim = 3;

	Eigen::MatrixXd A(2*u_i.size(), x_dim * u_dim);
	A.setZero();
	for(int i = 0; i < u_i.size(); ++i){
		auto x_i = u_i.at(i)(0);
		auto y_i = u_i.at(i)(1);
		auto w_i = u_i.at(i)(2);
		auto X_iT = X_i.at(i).transpose();
		A.block(2*i, x_dim, 1, x_dim) = -w_i * X_iT;
		A.block(2*i, 2*x_dim, 1, x_dim) = y_i * X_iT;
		A.block(2*i+1, 0, 1, x_dim) = w_i * X_iT;
		A.block(2*i+1, 2*x_dim, 1, x_dim) = -x_i * X_iT;
	}

	// Compute SVD and get nullspace
	auto svd  = A.jacobiSvd(Eigen::ComputeFullV);
	//auto nullspace = svd.matrixV().rightCols(1);
	auto nullspace = svd.matrixV().col(10);

	std::cout << "nullspace\n" << nullspace << "\n";

	// Combine into P matrix
	Eigen::MatrixXd P(u_dim, x_dim);

	P.row(0) = nullspace.topRows(x_dim).transpose();
	P.row(1) = nullspace.middleRows(x_dim, x_dim).transpose();
	P.row(2) = nullspace.bottomRows(x_dim).transpose();

	// No more work to be done
	return P;
}
