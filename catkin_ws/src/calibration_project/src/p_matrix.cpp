#include <Eigen/Dense>
#include <Eigen/SVD>
#include <vector>
#include <iostream>
#include <opencv2/core/eigen.hpp>


//Small issues I need the corresponding 2d image points and 3d world points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector3d>& X_i, const std::vector<Eigen::Vector3d>& u_i)
{
	/*
	if(points.size() < 0){
		std::cerr << "Need more points to calculate P matrix\n";
		throw std::runtime_error("Need more points");
	}
	*/
	Eigen::MatrixXd Projection_Matrix;
	Eigen::Matrix3d u; //Write the cross product as a matrix operation
	u << 0, -u_i(2), u_i(1),
	  u_i(2), 0, -u_i(0),
	  -u_i(1), u_i(0), 0;
	Eigen::MatrixXd px;
	/* Write P* X_i in terms of the row vectors, since all three terms like P_1.tranpose()* X_i are scalaras hence they are symmetric. p_1^T*X_i = X_I^T * P_1  */
	px << X_i.transpose() * P.rows(0),
	   X_i.transpose() * P.rows(1),
	   X_i.transpose() * P.rows(2);
    /*Matrix Multiplication with unknowns as a single vector, and the knows as a matrix multiplication with the unknowns */
	Eigen::MatrixXd knowns;
	knowns << 0, -u_i(2)*X_i.transpose(), u_i(1) * X_i,
	      u_i(2) * X_i.transpose(), 0, -u_i(0) * X_i.transpose(),
	      -u_i(1) * X.transpose(), u_i(0) * X_i.transpose(), 0;
	unsigned int rows = knowns.row();
	knowns.row(rows).setZero();
//Add a for loop that iterates over all points for u_i
	auto svd  = knowns.jacobiSvd(Eigen::ComputeFullV);
	
	Eigen::VectorXd  nullspace = svd.matrixV().col(11);

	Projection_Matrix = nullspace;

	return Projection_Matrix;

	

}
