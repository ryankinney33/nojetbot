#include "points.hpp"
#include "calc.hpp"

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <iostream>
#include <cstdlib>

#include <Eigen/Dense>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <Eigen/Dense>

int main()
{
	return 0;
}

void imageCallback(const sensor_msgs::Image::ConstPtr& msg);
void image_sub(int argc, char *argv[]){

     ros::init(argc,argv, "find_checkerboard");
     ros::NodeHandle n;

     ros::Subscriber sub = n.subscribe("/cv_camera/image_raw",1,&imageCallback);
     ros::spin();
}
void imageCallback(const sensor_msgs::Image::ConstPtr& msg) {

    cv_bridge::CvImageConstPtr cv_image_ptr;
    try {
        cv_image_ptr = cv_bridge::toCvShare(
            msg,
            sensor_msgs::image_encodings::RGB8);
    } catch (cv_bridge::Exception& e) {
        ROS_FATAL("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<Eigen::Vector3d> u_i;
    cv::Size patternsize(6,8);
    auto img = cv_image_ptr->image.clone();
    bool success = get_chessboard_points(img,patternsize,u_i);
    if(success){
	std::cout << "u_i (transpose)\n";
	for (const auto &iter : u_i) {
		std::cout << iter(0) << "\t" << iter(1) << "\t" << iter(2) << "\n";
	}

	// Get and print the points X_i
	std::vector<Eigen::Vector4d> X_i;
	get_3d_points(6, 8, 0.009, 0.025, X_i);

	std::cout << std::endl;
	// Print the points to stdout
	std::cout << "X_i (transpose)\n";
	for (const auto &iter : X_i) {
		std::cout << iter(0) << "\t" << iter(1) << "\t" << iter(2) << "\t" << iter(3) << "\n";
	}

	Eigen::MatrixXd P = find_p(X_i, u_i);
	std::cout << "\nP:\n" << P << "\n\n";

	// finally, find and print the camera matrices
	find_k(P);

	ros::shutdown();
    }
}
