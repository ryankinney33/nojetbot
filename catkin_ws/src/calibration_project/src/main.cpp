// For ROS
#include "ros/ros.h"
#include "sensor_msgs/Image.h"

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <iostream>
#include <vector>

#include "points.hpp"
#include "calc.hpp"

static cv::Size patternsize; // holds the dimensions of the chessboard
static std::vector<Eigen::Vector4d> X_i; // holds the 3d points of the chessboard corners

void imageCallback(const sensor_msgs::Image::ConstPtr& msg)
{
	// Read the image into an OpenCV image
	cv_bridge::CvImageConstPtr cv_image_ptr;
	try {
		cv_image_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception& e) {
		ROS_FATAL("cv_bridge exception: %s", e.what());
		return;
	}
	auto img = cv_image_ptr->image.clone();
	// Show the image on the screen
	cv::imshow("IMG", img);
	cv::waitKey(1);

	// Look for chessboard points
	std::vector<Eigen::Vector3d> u_i;
	bool success = get_chessboard_points(img, patternsize, u_i);
	if (success) {
		// Points found, get the P matrix
		Eigen::MatrixXd P = find_p(X_i, u_i);

		// Finally, find and print the camera matrixes
		find_k(P);

		ros::shutdown();
		// Show the updated image
		cv::imshow("IMG", img);
		cv::waitKey(1000);
	}
}

// The main function to setup everything and start the ros listener
int main(int argc, char *argv[])
{
	if (argc < 5) {
		std::cerr << "Usage: " << argv[0] << " gridWidth gridHeight squareSize bezelWidth\n";
		return 1;
	}

	// Parse the command-line arguments to get the sizes
	// they are image_path, grid width, grid height, square size (mm),
	// vert. bezel size (mm), horiz. bezel size (mm)
	int width, height;
	double square_size, bezel_width;

	try {
		// extract width and height
		width = std::stoi(argv[1]);
		height = std::stoi(argv[2]);

		// physical dimensions
		square_size = std::stod(argv[3])/1000.0;
		bezel_width = std::stod(argv[4])/1000.0;
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

	// Set up the necessary data
	patternsize = cv::Size(width, height); // number of centers
	get_3d_points(width, height, square_size, bezel_width, X_i); // 3d chessboard points

	// Start the ROS node
	ros::init(argc, argv, "camera_calibration");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/cv_camera/image_raw", 1, &imageCallback);
	ros::spin();

	return 0;
}
