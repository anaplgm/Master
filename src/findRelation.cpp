/*
* Copyright (C) 2008, Morgan Quigley and Willow Garage, Inc.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *   * Redistributions of source code must retain the above copyright notice,
 *     this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above copyright
 *     notice, this list of conditions and the following disclaimer in the
 *     documentation and/or other materials provided with the distribution.
 *   * Neither the names of Stanford University or Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

// %Tag(FULLTEXT)%
#include "ros/ros.h"
#include "std_msgs/String.h"

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
#include "/home/benner/catkin_ws/devel/include/ioc/Relation.h"
#include "/home/benner/catkin_ws/devel/include/ioc/ListOfRelation.h"

using namespace std;
using namespace cv;

void calcAllen(ioc::listOfRoi calcPubRoi);

ioc::listOfRoi SortInterval(ioc::listOfRoi publishRoi)
{

	ioc::RegionOfInterest aux;

	int tamanho = publishRoi.listOfROI.size();
	int i,j;

	for(i=tamanho-1; i >= 1; i--)
	{
		for(j=0; j < i ; j++)
		{
			if(publishRoi.listOfROI[j].x_offset > publishRoi.listOfROI[j+1].x_offset)
			{
				aux = publishRoi.listOfROI[j];
				publishRoi.listOfROI[j] = publishRoi.listOfROI[j+1];
				publishRoi.listOfROI[j+1] = aux;
			}
		}
	}

	return publishRoi;
}

void roiCallback(const ioc::listOfRoi pubRoi)
{
	ioc::listOfRoi list = pubRoi;
	list = SortInterval(list);
	calcAllen(list);
}
//Global
ros::Publisher relationString_pub;
ros::Publisher relation_pub;

void calcAllen(ioc::listOfRoi calcPubRoi)
{

	std_msgs::String msg;
	std::stringstream ss;
	char relation[50];

	ioc::Relation relations;
	ioc::ListOfRelation listOfRelations;



	for (int i=0; i< calcPubRoi.listOfROI.size();i++)
	{

			int x_1 = calcPubRoi.listOfROI[i].x_offset;
			int x_2 = calcPubRoi.listOfROI[i].x_offset + calcPubRoi.listOfROI[i].width;
			char x_name = calcPubRoi.listOfROI[i].name;
			float x_layer =  calcPubRoi.listOfROI[i].distance;

			for (int j=0; j< calcPubRoi.listOfROI.size();j++ )
			{
				if (i != j)
				{

					int y_1 = calcPubRoi.listOfROI[j].x_offset;
					int y_2 = calcPubRoi.listOfROI[j].x_offset + calcPubRoi.listOfROI[j].width;
					char y_name = calcPubRoi.listOfROI[j].name;
					float y_layer =  calcPubRoi.listOfROI[j].distance;

					relations.interval_x = x_name;
					relations.interval_y = y_name;

					if (x_1 < y_1 && ( x_2 < y_1 && abs(y_1 - x_2 > 2)))
					{
						sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "p", calcPubRoi.listOfROI[j].name );


						relations.relation = "p";
						//colocando ruído
						//relations.relation = "d";

						/*if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "p+", calcPubRoi.listOfROI[j].name );
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "p-", calcPubRoi.listOfROI[j].name );
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "p", calcPubRoi.listOfROI[j].name );
						}*/
					}else if (y_1 < x_1 && ( y_2 < x_1 && abs(x_1 - y_2) > 2))
					{


						relations.relation = "pi";
						/*if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "pi+", calcPubRoi.listOfROI[j].name );
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "pi-", calcPubRoi.listOfROI[j].name );
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "pi", calcPubRoi.listOfROI[j].name );
						}*/
						sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "pi", calcPubRoi.listOfROI[j].name );

					}else if (x_1 < y_1 &&  (x_2 == y_1 || abs(y_1 - x_2) <= 2))
					{


						relations.relation = "m";
						/*if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "m+", calcPubRoi.listOfROI[j].name );
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "m-", calcPubRoi.listOfROI[j].name );
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "m", calcPubRoi.listOfROI[j].name );
						}*/
						sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "m", calcPubRoi.listOfROI[j].name );

					//}else if (x_1 > y_1 &&  x_2 == y_1)
					}else if (y_1 < x_1 &&  (y_2 == x_1 || abs(x_1 - y_2) <= 2))
					{


						relations.relation = "mi_";
						/*if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "mi_+", calcPubRoi.listOfROI[j].name );
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "mi_-", calcPubRoi.listOfROI[j].name );
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "mi_", calcPubRoi.listOfROI[j].name );
						}*/
						sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "mi_", calcPubRoi.listOfROI[j].name );

					}else if (x_1 < y_1 &&  y_1 < x_2 && x_2 < y_2)
					{
						relations.relation = "o";

						if (x_layer < y_layer)
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "o+", calcPubRoi.listOfROI[j].name );
								relations.layer = '+';

							}
							else if (x_layer > y_layer)
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "o-", calcPubRoi.listOfROI[j].name );
								relations.layer = '-';
							}else
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "o", calcPubRoi.listOfROI[j].name );

							}


					}else if (y_1 < x_1 &&  x_1 < y_2 && y_2 < x_2)
					{
						relations.relation = "oi_";

						if (x_layer < y_layer)
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "oi_+", calcPubRoi.listOfROI[j].name );
								relations.layer = '+';
							}
							else if (x_layer > y_layer)
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "oi_-", calcPubRoi.listOfROI[j].name );
								relations.layer = '-';
							}else
							{
								sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "oi_", calcPubRoi.listOfROI[j].name );

							}

					}else if ((x_1 == y_1 || abs(x_1 - y_1) < 2) &&  y_1 < x_2 && x_2 < y_2 && abs(y_2 - x_2) > 2)
					{
						relations.relation = "s";


						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "s+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "s-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "s", calcPubRoi.listOfROI[j].name );

						}


					}else if ((y_1 == x_1 || abs(x_1 - y_1) < 2) &&  x_1 < y_2 && y_2 < x_2 && abs(y_2 - x_2) > 2)
					{
						relations.relation = "si_";


						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "si_+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "si_-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "si_", calcPubRoi.listOfROI[j].name );

						}

					}else if (y_1 < x_1 && (x_2 == y_2 || abs(x_2 - y_2 ) < 2))
					{
						relations.relation = "f";
						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "f+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "f-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "f", calcPubRoi.listOfROI[j].name );

						}

					}else if (y_1 > x_1 && x_2 == y_2)
					{
						relations.relation = "fi_";
						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "fi_+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "fi_-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "fi_", calcPubRoi.listOfROI[j].name );

						}

					}else if (abs(y_1 - x_1) <=2 && abs(y_2 - x_2) <=2)
					{
						relations.relation = "c";
						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "c+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "c-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "e", calcPubRoi.listOfROI[j].name );
							relations.relation = "e";
						}

					}else if (y_1 < x_1 && x_2 < y_2)
					{
						relations.relation = "d";
						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "d+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "d-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "d", calcPubRoi.listOfROI[j].name );

						}

					}else if (y_1 > x_1 && x_2 > y_2)
					{

						relations.relation = "di_";
						if (x_layer < y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "di_+", calcPubRoi.listOfROI[j].name );
							relations.layer = '+';
						}
						else if (x_layer > y_layer)
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "di_-", calcPubRoi.listOfROI[j].name );
							relations.layer = '-';
						}else
						{
							sprintf(relation, "%c  %s %c |", calcPubRoi.listOfROI[i].name, "di_", calcPubRoi.listOfROI[j].name );

						}

					}
					listOfRelations.ListOfRelation.push_back(relations);

					msg.data+= relation;
				}
			}

	}

	relationString_pub.publish(msg);
	relation_pub.publish(listOfRelations);

}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "AllenInterval");


  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/ROI", 1000, roiCallback);
  relationString_pub = n.advertise<std_msgs::String >("husky_1/RelationsString",1000);
  relation_pub = n.advertise<ioc::ListOfRelation>("husky_1/Relations",1000);
  ros::spin();
  return 0;
}


