#ifndef CALC_H
#define CALC_H

#include <Eigen/Dense>
#include <vector>

// Calculate and print K, R, and t from projection matrix P
void find_k(const Eigen::MatrixXd& P);

// Calculate projection matrix P given image points and corresponding 3d points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i,
		const std::vector<Eigen::Vector3d>& u_i);

#endif /* CALC_H */
