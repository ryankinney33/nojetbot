#include <Eigen/Dense>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/eigen.hpp>
#include <iostream>
#include <string>

#include "image.hpp"
#include "calc.hpp"

int main(int argc, char *argv[])
{
	// Holds the path of the image to open
	std::string path;

	if (argc != 6) {
		std::cerr << "Usage: " << argv[0] << " image_path gridWidth gridHeight squareSize bezelWidth\n";
		return 1;
	}

	// Parse the command-line arguments to get the sizes
	// they are image_path, grid width, grid height, square size (mm),
	// vert. bezel size (mm), horiz. bezel size (mm)
	int width, height;
	double square_size, bezel_width;

	try {
		// extract width and height
		width = std::stoi(argv[2]);
		height = std::stoi(argv[3]);

		// physical dimensions
		square_size = std::stod(argv[4])/1000.0;
		bezel_width = std::stod(argv[5])/1000.0;
	} catch(const std::invalid_argument& e) {
		// A conversion failed
		std::cerr << "Error: arguments except the image path must be numerical.\n";
		return 1;
	} catch(const std::out_of_range& e) {
		// Out of range error
		std::cerr << "Error: argument out of range\n";
		return 1;
	}

	// Make sure the numbers were positive
	assert(width > 0 && height > 0 && square_size > 0 && bezel_width > 0);

	// finally, image path
	path = argv[1];
	std::cout << "Opening: " << path << std::endl;

	cv::Mat img_corners = cv::imread(path);
	if (img_corners.empty()) {
		std::cerr << "Error: Failed to load the image." << std::endl;
		return 1;
	}

	// Show the image on the screen
	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	cv::imshow("image", img_corners);
	cv::waitKey(-1);

	cv::Size patternsize(width, height); // number of centers

	// Matrices to hold the points
	std::vector<Eigen::Vector3d> u_i;

	// Get the points
	bool patternfound = get_chessboard_points(img_corners, patternsize, u_i);
	if (!patternfound) {
		std::cerr << "Pattern not found." << std::endl;
		return -1;
	}

	// Print the points to stdout
	std::cout << "u_i (transpose)\n";
	for (const auto &iter : u_i) {
		std::cout << iter(0) << "\t" << iter(1) << "\t" << iter(2) << "\n";
	}

	// Get and print the points X_i
	std::vector<Eigen::Vector4d> X_i;
	get_3d_points(width, height, square_size, bezel_width, X_i);

	std::cout << std::endl;
	// Print the points to stdout
	std::cout << "X_i (transpose)\n";
	for (const auto &iter : X_i) {
		std::cout << iter(0) << "\t" << iter(1) << "\t" << iter(2) << "\t" << iter(3) << "\n";
	}


	cv::imshow("image", img_corners);
	cv::waitKey(-1);

	Eigen::MatrixXd P = find_p(X_i, u_i);
	std::cout << "\nP:\n" << P << "\n\n";

	// finally, find and print the camera matrices
	find_k(P);

	return 0;
}
