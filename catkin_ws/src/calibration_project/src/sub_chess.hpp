#ifndef SUBCHESS_H
#define SUBCHESS_H

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

void image_sub();
void imageCallback(const sensor_msgs::Image::ConstPtr& msg);

#endif
