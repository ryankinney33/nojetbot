#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
//#include <opencv2/imgcodecs.hpp>
#include <Eigen/Dense>
#include <iostream>

// Convert the image centers into an eigen matrix
static Eigen::MatrixXd centers_to_eigen(const std::vector<cv::Point2f> &centers)
{
	// The matrix that holds the points
	Eigen::MatrixXd corners(centers.size(), 3);

	// Iterate over the points in the vector
	for (int i = 0; i < centers.size(); ++i) {
		corners(i, 0) = centers[i].x;
		corners(i, 1) = centers[i].y;
	}
	corners.col(2).setOnes(); // Set the 3rd column to ones

	return corners;
}

// Gets the chessboard points in an image
bool get_chessboard_points(cv::Mat &img, const cv::Size &patternsize,
		Eigen::MatrixXd &us,
		Eigen::MatrixXd &ups)
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
	us = centers_to_eigen(centers);

	// Now find the second points
	patternfound = cv::findChessboardCornersSB(img, patternsize, centers);
	if (!patternfound)
		return false;

	// Draw the second set of points on the image
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	// Extract the second set of center points
	ups = centers_to_eigen(centers);

	return true;
}

