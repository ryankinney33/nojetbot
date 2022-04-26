#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>
#include <opencv2/core/eigen.hpp>


//Small issues I need the corresponding 2d image points and 3d world points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector3d>& X_i, const std::vector<Eigen::Vector3d>& u_i)
{
	
	if(X_i.rows() < 6 || u_i.rows() < 6){
		std::cerr << "Need at least 6 points to find the projection matrix\n";
		throw std::runtime_error("Need more points");
	}
	Eigen::MatrixXd Projection_Matrix(2*u_i.rows(), X_i.cols() * u_i.cols());
	Projection_Matrix.setZero();
	for(int i = 0; i < u_i.rows(); ++i){
		Projection_Matrix(2*i, X_i.cols(),1,X_i.cols()) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix(2*i, X_i.cols() * 2,1,X_i.cols()) = X_i[i](1) * u_i[i].transpose();
		Projection_Matrix(2*i + 1, 0,1,X_i.cols()) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix(2*i + 1, X_i.cols() * 2,1,X_i.cols()) = X_i[i](0) * u_i[i].transpose;
		
	
	}
	Eigen::MatrixXd 
	auto svd  = knowns.jacobiSvd(Eigen::ComputeFullV);
	
	Eigen::VectorXd  nullspace = svd.matrixV().col(X_i.cols()-1);

	Eigen::MatrixXd projection(u_i.cols(),X_i.cols());
	projection.row(0) = nullspace.block(0,0,3,1).transpose();
	projection.row(1) = nullspace.block(3,0,3,1).transpose();
	projection.row(2) = nullspace.block(6,0,3,1).transpose();

	return projection;

}
