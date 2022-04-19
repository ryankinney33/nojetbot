#ifndef IMAGE_H
#define IMAGE_H

#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <vector>

// Convert the image centers into an eigen matrix
static Eigen::MatrixXd centers_to_eigen(const std::vector<cv::Point2f> &centers);


// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		std::vector<Eigen::Vector3d> &u_i);

#endif /* IMAGE_H */
