#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <iostream>

#include "image.hpp"

// Convert the image centers into an eigen matrix
static void centers_to_eigen(const std::vector<cv::Point2f> &centers,
		std::vector<Eigen::Vector3d> &u_i)
{
	Eigen::Vector3d temp;
	temp(2) = 1;

	// Iterate over the points in the vector
	for (auto i: centers) {
		temp(0) = i.x;
		temp(1) = i.y;
		u_i.push_back(temp);
	}
}

// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		std::vector<Eigen::Vector3d> &u_i)
{
	std::vector<cv::Point2f> centers; // filled by the detected centers

	// Find the first set of chessboard points
	bool patternfound = cv::findChessboardCornersSB(img, patternsize, centers);
	if (!patternfound)
		return false;
	// Draw the points on the image
	cv::Mat centers_mat(centers);
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	// Extract the center points
	centers_to_eigen(centers, u_i);

	// Now find the second points
	patternfound = cv::findChessboardCornersSB(img, patternsize, centers);
	if (!patternfound)
		return false;

	// Draw the second set of points on the image
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	// Extract the second set of center points
	centers_to_eigen(centers, u_i);

	return true;
}

