#ifndef POINTS_H
#define POINTS_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		std::vector<Eigen::Vector3d> &u_i);

// Gets the 3D representation of the chessboard points
void get_3d_points(int width, int height, double square_size, double bezel_width,
		std::vector<Eigen::Vector4d> &X_i);

#endif /* POINTS_H */
