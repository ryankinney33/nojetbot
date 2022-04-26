#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <Eigen/Dense>
#include <iostream>
#include <algorithm>

#include "image.hpp"

// Convert the image centers into an eigen matrix
static void centers_to_eigen(std::vector<cv::Point2f> &centers,
		std::vector<Eigen::Vector3d> &u_i)
{
	Eigen::Vector3d temp;
	temp(2) = 1;

	// Compare the first and last point
	auto back = centers.back();
	auto head = centers.front();

	// Check if the order needs to be reversed
	if (head.x > back.x && head.y > back.y) {
		std::reverse(centers.begin(), centers.end());
	}

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

	// Extract the center points
	centers_to_eigen(centers, u_i);

	// Draw the points on the image
	cv::Mat centers_mat(centers);
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	// Now find the second points
	patternfound = cv::findChessboardCornersSB(img, patternsize, centers);
	if (!patternfound)
		return false;

	// Extract the second set of center points
	centers_to_eigen(centers, u_i);

	// Draw the second set of points on the image
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	return true;
}

// Gets the 3D representation of the chessboard points
void get_3d_points(int width, int height, double square_size, std::vector<Eigen::Vector4d> &X_i)
{
	Eigen::Vector4d temp;
	temp(3) = 1;
	/*
	 * Assumptions:
	 *   1. The left chessboard was detected first
	 *   2. The top left point was the first detected
	 *   3. The z-axis is the horizontal position of the left board
	 *   4. The y-axis is the horizontal position of the right board
	 *   5. The x-axis is the vertical position on the board
	 */

	// Go through the left chessboard first
	temp(1) = 0.0; // y = 0 on left chessboard
	for (int i = 0; i < height; ++i) {
		temp(0) = i + 1; // Points start at 1 unit down
		for (int j = 0; j < width; ++j) {
			temp(2) = width + 1 - j; // Points start at width + 1 units left
			X_i.push_back(square_size * temp);
		}
	}

	// Go through the right chessboard first
	temp(2) = 0; // z = 0 on right chessboard
	for (int i = 0; i < height; ++i) {
		temp(0) = i + 1; // Points start at 1 unit down
		for (int j = 0; j < width; ++j) {
			temp(1) = j + 2; // Points start at 2 units right
			X_i.push_back(square_size * temp);
		}
	}
}
