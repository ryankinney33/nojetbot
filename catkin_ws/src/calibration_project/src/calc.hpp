#ifndef CALC_H
#define CALC_H

#include <Eigen/Dense>
#include <vector>

void intrinsic_CameraMatrix(const Eigen::MatrixXd& projection);

//Small issues I need the corresponding 2d image points and 3d world points
Eigen::MatrixXd find_p(const std::vector<Eigen::Vector4d>& X_i,
		const std::vector<Eigen::Vector3d>& u_i);

#endif /* CALC_H */
