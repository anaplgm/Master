/*
 /*
 * findIntervals.cpp
 *
 *  Created on: Aug 10, 2016
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
using namespace sensor_msgs;
using namespace message_filters;

static const std::string OPENCV_WINDOW = "Image window";

int thresh = 79;
int iLowH = 0;
int iHighH = 180;

int iLowS = 0;
int iHighS = 255;

int iLowV = 0;
int iHighV = 255;

ros::Publisher rect_pub;
Mat src, src_gray, dst, srcRoi;


void thresh_callback(int, void* );

ioc::listOfRoi publishRoi;

cv_bridge::CvImagePtr cv_ptr;
cv_bridge::CvImagePtr depthImage_ptr;

image_transport::Subscriber image_sub_;
image_transport::Subscriber depth_sub_;
image_transport::Publisher image_pub_;


void imgCallback(const sensor_msgs::ImageConstPtr& msg)
//void imgCallback()
  {
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

	//src = imread( "/home/benner/Pictures/cenario_1.png", 1 );
	//src = src(Rect(0,0,640,480)).clone();

    Mat subImg; Mat orig;
  //  src = cv_ptr->image;
    orig = src;
    Mat imgThresholded;
    cvtColor( orig, dst, CV_BGR2HSV);

    //Como encontrar o range de cada cor
   /* inRange(dst, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); //Threshold the image
    namedWindow( "ImageT", CV_WINDOW_AUTOSIZE );
    imshow( "ImageT", imgThresholded );*/

    cv::imshow(OPENCV_WINDOW, src);
    cv::waitKey(3);

    thresh_callback( 0, 0 );

  }

void findRedInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;



	 inRange(dst, Scalar(0, 217, 97), Scalar(0, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'R'; //red

		publishRoi.listOfROI.push_back(roi);

	 }
}

void findYellowInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;

	 inRange(dst, Scalar(21, 0, 0), Scalar(38, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'Y'; //red
		publishRoi.listOfROI.push_back(roi);
	}
}

void findBlueInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;

	 inRange(dst, Scalar(118, 0, 0), Scalar(122, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'B'; //red

		publishRoi.listOfROI.push_back(roi);

	 }
}

void findOrangeInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;
    float distance=0;


	 inRange(dst, Scalar(7, 0, 0), Scalar(22, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'O'; //red

		publishRoi.listOfROI.push_back(roi);

	 }
}

void findPurpleInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;
    float distance=0;


	 inRange(dst, Scalar(130, 0, 0), Scalar(179, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'P'; //red

		publishRoi.listOfROI.push_back(roi);

	 }
}

void findGreenInterval()
{
	 Mat imgThresholded;
	 vector<Vec4i> hierarchy;
	 std::vector< std::vector<cv::Point> > contours;
	 ioc::RegionOfInterest roi;
    float distance=0;


	 inRange(dst, Scalar(35,0,0), Scalar(100, 255, 255), imgThresholded); //Threshold the image
	 Canny( imgThresholded, imgThresholded, thresh, thresh*2, 3 );
	 findContours( imgThresholded, contours, hierarchy, RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0) );

	 if (contours.size()>0)
	 {
		Rect temp = cv::boundingRect(contours[0]);
		roi.x_offset = temp.x;
		roi.y_offset = temp.y;
		roi.width = temp.width;
		roi.height = temp.height;
		roi.name = 'G'; //red

		publishRoi.listOfROI.push_back(roi);

	 }
}


void thresh_callback(int, void* )
  {
    Mat canny_output;
    vector<Vec4i> hierarchy;
    float distance=0;

    findRedInterval();
  //  findYellowInterval();
    findBlueInterval();
    findGreenInterval();
    findPurpleInterval();
    findOrangeInterval();

    Mat drawing = Mat::zeros( src.size(), CV_8UC3 );
    /*for( int i = 0; i < publishRoi.listOfROI.size(); i++ )
       {
      	Point p1 = Point(publishRoi.listOfROI[i].x_offset, publishRoi.listOfROI[i].y_offset);
    	Point p2 = Point(publishRoi.listOfROI[i].x_offset + publishRoi.listOfROI[i].width, publishRoi.listOfROI[i].y_offset + publishRoi.listOfROI[i].height);
        cv::rectangle(drawing ,p1,p2,cv::Scalar(100, 100, 200), 2, CV_AA);
       }*/

    for( int i = 0; i < publishRoi.listOfROI.size(); i++ )
	   {
		Point center = Point(publishRoi.listOfROI[i].x_offset + publishRoi.listOfROI[i].width /2 , publishRoi.listOfROI[i].y_offset + publishRoi.listOfROI[i].height/2);
		int radius = publishRoi.listOfROI[i].width /2;
		cv::circle(drawing,center,radius,cv::Scalar(100, 100, 200), 2, CV_AA);
	   }


    /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );

    rect_pub.publish(publishRoi);
    publishRoi.listOfROI.clear();


  }



int main(int argc, char** argv)
{
  ros::init(argc, argv, "TOPVIEW");

 /* namedWindow( "Control", 0 ); //para determinar o range de cada cor
  //Create trackbars in "Control" window
  cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
  cvCreateTrackbar("HighH", "Control", &iHighH, 179);

  cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
  cvCreateTrackbar("HighS", "Control", &iHighS, 255);

  cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
  cvCreateTrackbar("HighV", "Control", &iHighV, 255);*/

  ros::NodeHandle nh;
  rect_pub = nh.advertise< ioc::listOfRoi >("hector/ROI",1000);
 // imgCallback();
  ros::Subscriber sub = nh.subscribe("/hector_1/hector_fei_with_cam_downward_cam/camera/image", 1, imgCallback);
//  ros::Subscriber sub = nh.subscribe("/hector_1/hector_fei_with_laser_fwd_cam_front_cam/camera/image", 1, imgCallback);

  ros::spin();


  return 0;
}


