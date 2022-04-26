#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>
#include <opencv2/core/eigen.hpp>


//Small issues I need the corresponding 2d image points and 3d world points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i , const std::vector<Eigen::Vector3d>& u_i)
{
	
	if(X_i.size() < 6 || u_i.size() < 6){
		std::cerr << "Need at least 6 points to find the projection matrix\n";
		throw std::runtime_error("Need more points");
	}
	int db = X_i.size();
	int bd = u_i.size();
	Eigen::MatrixXd Projection_Matrix(2*u_i.size(), db*bd);
	Projection_Matrix.setZero();
	for(int i = 0; i < u_i.size(); ++i){
		Projection_Matrix.block(2*i, db,1,db) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix.block(2*i, db * 2,1,db) = X_i[i](1) * u_i[i].transpose();
		Projection_Matrix.block(2*i + 1, 0,1,db) = -X_i[i](2) * u_i[i].transpose();
		Projection_Matrix.block(2*i + 1, db * 2,1,db) = X_i[i](0) * u_i[i].transpose();
		
	
	}
	auto svd  = Projection_Matrix.jacobiSvd(Eigen::ComputeFullV);
	
	Eigen::VectorXd  nullspace = svd.matrixV().col(X_i.size()-1);

	Eigen::MatrixXd projection(u_i.size(),X_i.size());
	projection.row(0) = nullspace.block(0,0,3,1).transpose();
	projection.row(1) = nullspace.block(3,0,3,1).transpose();
	projection.row(2) = nullspace.block(6,0,3,1).transpose();

	return projection;

}
