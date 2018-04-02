
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <boost/algorithm/string.hpp>
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

using namespace std;
using namespace cv;
using namespace sensor_msgs;
//global

string possibleRelations;
ros::Publisher relation_pub;

/*
void translationTable (std::vector<std::string> relations)
{
	int i;


	//cada relação possível para cada intervalo

	//única relação possível
	 if ( std::find(relations.begin(), relations.end(), "S_2 p B") != relations.end())
	 {
		 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
		 {
			 possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
		 }
		 else
		 {
			 possibleRelations = "n.d.";
		 }
	 }
	 //não tem visão
	 else if ( std::find(relations.begin(), relations.end(), "S_2 m B") != relations.end())
	 {
		 possibleRelations = "n.d.";
	 }

	else if ( std::find(relations.begin(), relations.end(), "S_2 o+ B") != relations.end())
	 {
		 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
		 {
			// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
			 possibleRelations = "b_2 { p } g_2";
		 }
		 else

		 {
			 possibleRelations = "n.d.";
		 }
	 }


	else if ( std::find(relations.begin(), relations.end(), "S_2 fi_+ B") != relations.end())
	{
		if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					possibleRelations = "b_2 { p,m, o+, s+, c+, d+, f+ } g_2";
			 	 	 //possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
	}

	else if ( std::find(relations.begin(), relations.end(), "S_2 s+ B") != relations.end())
	 {
		 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
		 {
			// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
			 possibleRelations = "b_2 { p } g_2";
		 }
		 else

		 {
			 possibleRelations = "n.d.";
		 }
	 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 di_+ B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 c+ B") != relations.end())
			 {

				possibleRelations = "b_2 { p } g_2";
		      }
	else if ( std::find(relations.begin(), relations.end(), "S_2 d+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }

	else if ( std::find(relations.begin(), relations.end(), "S_2 si_+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 f+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 oi_+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 mi_ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 pi_ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 m G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 o+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 fi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 fi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 } else if (std::find(relations.begin(), relations.end(), "S_2 s+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 di_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 c+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 d+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 si_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 f+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 oi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 mi_ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi_ } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 mi_ G") != relations.end())
			 {
				possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";

			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }else if ( std::find(relations.begin(), relations.end(), "S_2 o- B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				 possibleRelations = "b_2 {  pi_ } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }


		else if ( std::find(relations.begin(), relations.end(), "S_2 fi_- B") != relations.end())
		{
			if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
					 {
						 //possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
						possibleRelations = "b_2 { pi_,mi_, oi_+, si_+, ci_+, di_+, fi_+ } g_2";
					 }
					 else

					 {
						 possibleRelations = "n.d.";
					 }
		}

		else if ( std::find(relations.begin(), relations.end(), "S_2 s- B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { pi_ } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 di_- B") != relations.end())
			 {
				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					 possibleRelations = "b_2 { pi_ } g_2";
					// possibleRelations = "b_2 { p } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 c- B") != relations.end())
				 {

					possibleRelations = "b_2 { pi_ } g_2";
			      }
		else if ( std::find(relations.begin(), relations.end(), "S_2 d- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }

		else if ( std::find(relations.begin(), relations.end(), "S_2 si_- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 f- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 oi_- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }

	 relation_pub.publish(possibleRelations);

}
*/

std::vector<std::string> split(const std::string &text, char sep)
{
  std::vector<std::string> tokens;
  std::size_t start = 0, end = 0;
  while ((end = text.find(sep, start)) != std::string::npos)
  {
    tokens.push_back(text.substr(start, end - start));
    start = end + 1;
  }
  tokens.push_back(text.substr(start));
  return tokens;
}

void translationTable(vector<std::string> relacao)
{
	vector<std::string> relations = relacao;
	string possibleRelations;
	std_msgs::String msg;

	if ( std::find(relations.begin(), relations.end(), "S_2 p B") != relations.end())
	{
		int i =0;
		if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
		 {
			 possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
		 }
		 else
		 {
			 possibleRelations = "n.d.";
		 }
	}else if ( std::find(relations.begin(), relations.end(), "S_2 m B") != relations.end())
	 {
		 possibleRelations = "n.d.";
	 }

	else if ( std::find(relations.begin(), relations.end(), "S_2 o+ B") != relations.end())
	 {
		 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
		 {
			// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
			 possibleRelations = "b_2 { p } g_2";
		 }
		 else

		 {
			 possibleRelations = "n.d.";
		 }
	  }else if ( std::find(relations.begin(), relations.end(), "S_2 fi_+ B") != relations.end())
		{
			if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
					 {
						possibleRelations = "b_2 { p,m, o+, s+, c+, d+, f+ } g_2";
				 	 	 //possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 }
					 else

					 {
						 possibleRelations = "n.d.";
					 }
		}else if ( std::find(relations.begin(), relations.end(), "S_2 s+ B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }	else if ( std::find(relations.begin(), relations.end(), "S_2 di_+ B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 c+ B") != relations.end())
			 {

				possibleRelations = "b_2 { p } g_2";
		      }
	else if ( std::find(relations.begin(), relations.end(), "S_2 d+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }

	else if ( std::find(relations.begin(), relations.end(), "S_2 si_+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 f+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 oi_+ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 mi_ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
	else if ( std::find(relations.begin(), relations.end(), "S_2 pi_ B") != relations.end())
		 {

			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 m G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 o+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 fi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 fi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 } else if (std::find(relations.begin(), relations.end(), "S_2 s+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 di_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 c+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }else if (std::find(relations.begin(), relations.end(), "S_2 d+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 si_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 f+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 oi_+ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 mi_ G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { p, pi_ } g_2";
			 }
			 else if (std::find(relations.begin(), relations.end(), "S_2 mi_ G") != relations.end())
			 {
				possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";

			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }else if ( std::find(relations.begin(), relations.end(), "S_2 o- B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				 possibleRelations = "b_2 {  pi_ } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }


		else if ( std::find(relations.begin(), relations.end(), "S_2 fi_- B") != relations.end())
		{
			if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
					 {
						 //possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
						possibleRelations = "b_2 { pi_,mi_, oi_+, si_+, ci_+, di_+, fi_+ } g_2";
					 }
					 else

					 {
						 possibleRelations = "n.d.";
					 }
		}

		else if ( std::find(relations.begin(), relations.end(), "S_2 s- B") != relations.end())
		 {
			 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
			 {
				// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
				 possibleRelations = "b_2 { pi_ } g_2";
			 }
			 else

			 {
				 possibleRelations = "n.d.";
			 }
		 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 di_- B") != relations.end())
			 {
				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					 possibleRelations = "b_2 { pi_ } g_2";
					// possibleRelations = "b_2 { p } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 c- B") != relations.end())
				 {

					possibleRelations = "b_2 { pi_ } g_2";
			      }
		else if ( std::find(relations.begin(), relations.end(), "S_2 d- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }

		else if ( std::find(relations.begin(), relations.end(), "S_2 si_- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }
		else if ( std::find(relations.begin(), relations.end(), "S_2 f- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }else if ( std::find(relations.begin(), relations.end(), "S_2 oi_- B") != relations.end())
			 {

				 if (std::find(relations.begin(), relations.end(), "S_2 p G") != relations.end())
				 {
					// possibleRelations = "b_2 { p,m, o+, s+, c+, di_+, d+, f+, oi_+, mi_, pi_ } g_2";
					 possibleRelations = "b_2 { pi_ } g_2";
				 }
				 else

				 {
					 possibleRelations = "n.d.";
				 }
			 }

	msg.data= possibleRelations;
	relation_pub.publish(msg);


}


void callback(const std_msgs::String::ConstPtr& allenRelations)
{
	std::string rel = allenRelations->data.c_str();
	vector<std::string> relations = split(rel, '|');

	translationTable(relations);

}



int main(int argc, char **argv)
{

	  ros::init(argc, argv, "TranslationTable");
	  ros::NodeHandle n;
	  ros::Subscriber sub = n.subscribe("/Relations", 1000, callback);
	  relation_pub = n.advertise<std_msgs::String >("TransfomedRelations",1000);
	  ros::spin();
	  return 0;
}



