/*
 * getDistance.cpp

 *
 *  Created on: Aug 2, 2016
 *      Author: root
 */

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <opencv2/video/background_segm.hpp>
#include "std_msgs/MultiArrayLayout.h"
#include "std_msgs/MultiArrayDimension.h"
#include "std_msgs/Int32MultiArray.h"
#include "/home/benner/catkin_ws/devel/include/ioc/listOfRoi.h"
#include "/home/benner/catkin_ws/devel/include/ioc/RegionOfInterest.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/PointCloud.h>

using namespace std;
using namespace cv;
using namespace message_filters;

typedef union U_FloatParse {
    float float_data;
    unsigned char byte_data[4];
} U_FloatConvert;

//global

float depth=0;

void imageCallback(const sensor_msgs::ImageConstPtr& image) {
    cv_bridge::CvImagePtr depthImage_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::TYPE_32FC1);
    //depth = depthImage_ptr->image.at<float>(Point2f(406,408));
    depth = depthImage_ptr->image.at<float>(Point2f(358+(281/2),316));
    ROS_INFO("depth = %.2f",depthImage_ptr->image.at<float>(Point2f(406,408)));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "imagegrab");
    ros::NodeHandle n;
    printf("READY to get image\n");
    image_transport::ImageTransport it(n);
    image_transport::Subscriber sub = it.subscribe("/camera/depth/image_raw", 1, imageCallback);
    ros::spin();
    return 0;
}



