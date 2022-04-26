#ifndef IMAGE_H
#define IMAGE_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		std::vector<Eigen::Vector3d> &u_i);

// Gets the 3D representation of the chessboard points
void get_3d_points(int width, int height, double bezel_width,
		double square_size, std::vector<Eigen::Vector4d> &X_i);

#endif /* IMAGE_H */
