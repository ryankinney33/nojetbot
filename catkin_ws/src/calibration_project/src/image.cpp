#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
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
	std::vector<cv::Point2f> centers2; // filled by the detected centers

	// Find the left set of chessboard points
	cv::Rect roi; // region of interest in the image (growing window)
	roi.x = 0;
	roi.y = 0;
	roi.height = img.size().height;
	roi.width = img.size().width/4.0;
	bool patternfound;
	do {
		patternfound = cv::findChessboardCornersSB(img(roi), patternsize, centers);
		roi.width += img.size().width/4.0 - 1.0;
		if (roi.width > img.size().width) {
			return false;
		}
	} while (!patternfound);

	// Find the maximum x value in the left chessboard
	double x = centers.at(0).x;
	for (const auto &i : centers) {
		if (i.x > x)
			x = i.x;
	}

	roi.x = x;
	roi.width = img.size().width - x;

	// Extract the center points
	centers_to_eigen(centers, u_i);

	// Draw the points on the image
	cv::Mat centers_mat(centers);
	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);

	// Now find the second points
	patternfound = cv::findChessboardCornersSB(img(roi), patternsize, centers2);
	if (!patternfound)
		return false;

	// Add the offset
	for (auto &i : centers2) {
		i.x += x;
	}

	// Extract the second set of center points
	centers_to_eigen(centers2, u_i);

	// Draw the second set of points on the image
	cv::Mat centers_mat2(centers2);
	cv::drawChessboardCorners(img, patternsize, centers_mat2, patternfound);

	return true;
}

// Gets the 3D representation of the chessboard points
void get_3d_points(int width, int height, double square_size, double bezel_width,
		double bezel_height, std::vector<Eigen::Vector4d> &X_i)
{
	Eigen::Vector4d temp;
	temp(3) = 1.0;
	/*
	 * Assumptions:
	 *   1. The left chessboard was detected first
	 *   2. The top left point was the first detected
	 *   3. The z-axis is the horizontal position on the right board
	 *   4. The y-axis is the horizontal position of the left board
	 *   5. The x-axis is the vertical position
	 */

	// Go through the left chessboard first
	temp(1) = 0.0; // y = 0 on left chessboard
	for (int i = 0; i < height; ++i) {
		temp(0) = bezel_height + square_size * (i + 1); // Points start at 1 unit down
		for (int j = 0; j < width; ++j) {
			temp(2) = bezel_width + square_size * (width - j); // Points start at width units left

			X_i.push_back(temp);
		}
	}

	// Go through the right chessboard first
	temp(2) = 0; // z = 0 on right chessboard
	for (int i = 0; i < height; ++i) {
		temp(0) = bezel_height + square_size * (i + 1); // Points start at 1 unit down
		for (int j = 0; j < width; ++j) {
			temp(1) = bezel_width + square_size * (j + 1); // Points start at 1 units right
			X_i.push_back(temp);
		}
	}
}
