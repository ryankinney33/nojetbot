
void image_sub(){

     ros::init(argc,argv "find_checkerboard");
     ros::NodeHandle n;

     ros::Subscriber sub = n.subscribe("/cv_camera/image_raw",1,&imageCallback);
     ros::spin();
}
void imageCallback(
    const sensor_msgs::Image::ConstPtr& msg) {

    // http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
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
    cv::Size patternsize(6,8)
    bool success = get_chessboard_points(cv_image_ptr->image,patternsize,u_i);
    if (success)
        ros::shutdown();
}
