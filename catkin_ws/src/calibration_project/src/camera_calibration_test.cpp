#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <string>

#include "image.hpp"

int main(int argc, char *argv[]) {

	// Holds the path of the image to open
	std::string path;

	if (argc < 2) {
		std::cerr << "No image specified." << std::endl;
		return 1;
	}

	path = argv[1];

	std::cout << "Opening: " << path << std::endl;

	cv::Mat img = cv::imread(path, cv::IMREAD_GRAYSCALE);
	cv::Mat img_corners = cv::imread(path);
	if (img.empty()) {
		std::cerr << "Error: Failed to load the image." << std::endl;
		return 1;
	}

	// Show the image on the screen
	cv::namedWindow("windowname", cv::WINDOW_AUTOSIZE);
	cv::imshow("windowname", img);
	cv::waitKey(-1);

	cv::Size patternsize(9,6); // number of centers

	// Matrices to hold the points
	std::vector<Eigen::Vector3d> u_i;

	// Get the points
	bool patternfound = get_chessboard_points(img_corners, patternsize, u_i);
	if (!patternfound) {
		std::cerr << "Pattern not found." << std::endl;
		return -1;
	}

	// Print the points to stdout
	for (auto iter : u_i) {
		std::cout << iter(0) << "\t" << iter(1) << "\t" << iter(2) << std::endl;
	}

	cv::imshow("windowname", img_corners);
	cv::waitKey(-1);

	return 0;
}
