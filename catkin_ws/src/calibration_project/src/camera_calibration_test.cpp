#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

#include <iostream>
#include <string>

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
	std::vector<cv::Point2f> centers; // filled by the detected centers
	bool patternfound = cv::findChessboardCornersSB(img, patternsize, centers);

	if (!patternfound) {
		std::cerr << "Pattern not found." << std::endl;
		return -1;
	}

	cv::Mat centers_mat(centers);

	cv::drawChessboardCorners(img, patternsize, centers_mat, patternfound);
	cv::drawChessboardCorners(img_corners, patternsize, centers_mat, patternfound);

	patternfound = cv::findChessboardCornersSB(img, patternsize, centers);
	if (!patternfound) {
		std::cerr << "Pattern not found." << std::endl;
		return -1;
	}

	cv::drawChessboardCorners(img_corners, patternsize, centers_mat, patternfound);

	cv::imshow("windowname", img_corners);
	cv::waitKey(-1);

	return 0;
}
