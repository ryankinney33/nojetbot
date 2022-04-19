#ifndef IMAGE_H
#define IMAGE_H

#include <Eigen/Dense>

//#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>

//#include <iostream>
//#include <string>
#include <vector>

// Convert the image centers into an eigen matrix
static Eigen::MatrixXd centers_to_eigen(const std::vector<cv::Point2f> &centers);


// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		Eigen::MatrixXd &us,
		Eigen::MatrixXd &ups);

#endif /* IMAGE_H */
