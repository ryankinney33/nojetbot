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

	std::cout << "\nmultiplying\n" << K*R << std::endl;

	// no more work to be done
	return std::make_pair(K, R);
}

Eigen::Matrix3d find_k(const Eigen::MatrixXd& projection)
{
	Eigen::Matrix3d M = projection.leftCols(3);
	/*auto QR = M.householderQr();
	auto R = QR.matrixQR().triangularView<Eigen::Upper>();
	auto Q = QR.householderQ();
	return Q;
	*/
	Eigen::Matrix3d K, R;
	std::tie(K, R) = gramschmidt(M);

	return K;
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

	// Compute SVD and get nullspace
	auto svd  = A.jacobiSvd(Eigen::ComputeFullV);
	auto nullspace = svd.matrixV().col(11);

	// Combine into P matrix
	Eigen::MatrixXd P(3, 4);

	P.row(0) = nullspace.block(0,0,4,1).transpose();
	P.row(1) = nullspace.block(4,0,4,1).transpose();
	P.row(2) = nullspace.block(8,0,4,1).transpose();

	return P;
}
