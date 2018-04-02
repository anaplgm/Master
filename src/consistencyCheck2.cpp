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
#include "Poco/LinearHashTable.h"
#include "Poco/HashMap.h"
#include "Poco/HashSet.h"
#include "Poco/SimpleHashTable.h"
#include "Poco/HashTable.h"

#include <string>


using namespace std;
using namespace cv;

ros::Publisher relation_pub;
ioc::ListOfRelation inconsistecyList;



bool consistencyCheck(ioc::Relation rel1, ioc::Relation rel2, ioc::Relation rel3  )
{

// linha do p
	if (rel2.relation == "p")
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c" )
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "f" )
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_" )
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if (rel1.relation == "p" || rel1.relation == "pi" || rel1.relation == "m" || rel1.relation == "mi_" ||rel1.relation == "o" || rel1.relation == "oi_"
					|| rel1.relation == "s" || rel1.relation == "si_" || rel1.relation == "d" || rel1.relation == "di_" || rel1.relation == "f" || rel1.relation == "fi_"
					|| rel1.relation == "e" || rel1.relation == "c" )
			{
				return true;
			}else
			{
				return false;
			}
		}
	}

	//linha do m

	if (rel2.relation == "m")
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "m")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c" )
		{
			if (rel1.relation == "m" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" || rel3.relation == "f+" ||  rel3.relation == "f-")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_")
		{
			if (rel1.relation == "p" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_")
		{
			if (rel1.relation == "m" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_" )
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if (rel1.relation == "fi_" || rel1.relation == "e" || rel1.relation == "c" ||rel1.relation == "f" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if (rel1.relation == "pi" || rel1.relation == "mi_" || rel1.relation == "oi_" || rel1.relation == "si_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}
	//linha do o
	if (rel2.relation == "o")
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "o")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "o" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "s")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" ||  rel1.relation == "fi_" ||  rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" ||rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" || rel1.relation == "fi_" || rel1.relation == "e" || rel1.relation == "c" ||  rel1.relation == "f"
					|| rel1.relation == "oi_" || rel1.relation == "si_" || rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if (rel1.relation == "pi" || rel1.relation == "mi_" || rel1.relation == "oi_" || rel1.relation == "si_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}
	//linha do s
	if (rel2.relation == "s")
		{
			if (rel3.relation == "p")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "m")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "o")
			{
				if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "s")
			{
				if (rel1.relation == "s")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "d")
			{
				if (rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "fi_")
			{
				if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "e" || rel3.relation == "c")
			{
				if (rel1.relation == "s" )
				{
					return true;
				}else
				{
					return false;
				}
			}

			else if (rel3.relation == "f" )
			{
				if (rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "di_" )
			{
				if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" ||  rel1.relation == "fi_" ||  rel1.relation == "di_")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "si_" )
			{
				if (rel1.relation == "s" || rel1.relation == "e" || rel1.relation == "c" ||rel1.relation == "si_")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "oi_")
			{
				if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "mi_" )
			{
				if (rel1.relation == "mi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "pi" )
			{
				if (rel1.relation == "pi" )
				{
					return true;
				}else
				{
					return false;
				}
			}

		}
	//linha do d
	if (rel2.relation == "d")
		{
			if (rel3.relation == "p")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "m")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "o")
			{
				if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "s")
			{
				if (rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "d")
			{
				if (rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "fi_")
			{
				if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "e" || rel3.relation == "c")
			{
				if (rel1.relation == "d" )
				{
					return true;
				}else
				{
					return false;
				}
			}

			else if (rel3.relation == "f" )
			{
				if (rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "di_" )
			{
				if (rel1.relation == "p" || rel1.relation == "pi" || rel1.relation == "m" || rel1.relation == "mi_" ||rel1.relation == "o" || rel1.relation == "oi_"
									|| rel1.relation == "s" || rel1.relation == "si_" || rel1.relation == "d" || rel1.relation == "di_" || rel1.relation == "f" || rel1.relation == "fi_"
									|| rel1.relation == "e" || rel1.relation == "c" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "si_" )
			{
				if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" ||rel1.relation == "mi_" ||rel1.relation == "pi")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "oi_")
			{
				if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" ||rel1.relation == "mi_" ||rel1.relation == "pi")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "mi_" )
			{
				if (rel1.relation == "pi" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "pi" )
			{
				if (rel1.relation == "pi" )
				{
					return true;
				}else
				{
					return false;
				}
			}

		}
//linha do fi
	if (rel2.relation == "fi_")
		{
			if (rel3.relation == "p")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "m")
			{
				if (rel1.relation == "p")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "o")
			{
				if (rel1.relation == "o")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "s")
			{
				if (rel1.relation == "o")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "d")
			{
				if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "fi_")
			{
				if (rel1.relation == "fi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "e" || rel3.relation == "c")
			{
				if (rel1.relation == "fi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}

			else if (rel3.relation == "f" )
			{
				if (rel1.relation == "fi_" || rel1.relation == "e" || rel1.relation == "c" || rel1.relation == "f")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "di_" )
			{
				if (rel1.relation == "di_")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "si_" )
			{
				if (rel1.relation == "di_")
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "oi_")
			{
				if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "mi_" )
			{
				if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" )
				{
					return true;
				}else
				{
					return false;
				}
			}
			else if (rel3.relation == "pi" )
			{
				if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi" )
				{
					return true;
				}else
				{
					return false;
				}
			}

		}
	//linha do eq
	if (rel2.relation == "e" || rel2.relation == "c")
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "o")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "s")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "fi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "e" || rel1.relation == "c")
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "f")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "si_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if (rel1.relation == "oi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if (rel1.relation == "mi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}

	}
	//linha d f
	if (rel2.relation == "f" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "p")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "fi_" || rel1.relation == "e" ||  rel1.relation == "c" || rel1.relation == "f" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "f" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "f")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if (rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}

	}

	// linha do di
	if (rel2.relation == "di_" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" || rel1.relation == "fi_" || rel1.relation == "e" || rel1.relation == "c"
					|| rel1.relation == "f" || rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if ( rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if ( rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if ( rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}

	// linha do si
	if (rel2.relation == "si_" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "s" || rel1.relation == "e" || rel1.relation == "c" || rel1.relation == "si_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d" || rel1.relation == "f" ||  rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "si_" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "di_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "si_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if ( rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if ( rel1.relation == "mi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}

	// linha do oi
	if (rel2.relation == "oi_" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "o" || rel1.relation == "s" || rel1.relation == "d" || rel1.relation == "fi_" || rel1.relation == "e" || rel1.relation == "c"
								|| rel1.relation == "f" || rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "f" || rel1.relation == "d" || rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d" || rel1.relation == "f" ||  rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_"  )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "oi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "di_" || rel1.relation == "si_" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if ( rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if ( rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}
//linha do mi
	if (rel2.relation == "mi_" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p" || rel1.relation == "m" || rel1.relation == "o" || rel1.relation == "fi_" || rel1.relation == "di_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "s" || rel1.relation == "si_" || rel1.relation == "e" || rel1.relation == "c")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "d" || rel1.relation == "f" ||  rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "d" || rel1.relation == "f" ||  rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d" || rel1.relation == "f" ||  rel1.relation == "oi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "mi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "mi_" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "mi_")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if ( rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}

//linha do pi
	if (rel2.relation == "pi" )
	{
		if (rel3.relation == "p")
		{
			if (rel1.relation == "p" || rel1.relation == "pi" || rel1.relation == "m" || rel1.relation == "mi_" ||rel1.relation == "o" || rel1.relation == "oi_"
								|| rel1.relation == "s" || rel1.relation == "si_" || rel1.relation == "d" || rel1.relation == "di_" || rel1.relation == "f" || rel1.relation == "fi_"
								|| rel1.relation == "e" || rel1.relation == "c" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "m")
		{
			if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "o")
		{
			if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "s")
		{
			if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "d")
		{
			if (rel1.relation == "d" || rel1.relation == "f" || rel1.relation == "oi_" || rel1.relation == "mi_" || rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "fi_")
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "e" || rel3.relation == "c")
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}

		else if (rel3.relation == "f" )
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "di_" )
		{
			if (rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "si_" )
		{
			if (rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "oi_")
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "mi_" )
		{
			if ( rel1.relation == "pi" )
			{
				return true;
			}else
			{
				return false;
			}
		}
		else if (rel3.relation == "pi" )
		{
			if ( rel1.relation == "pi")
			{
				return true;
			}else
			{
				return false;
			}
		}

	}

}

void createCompositionTable()
{
//ainda a ser implmentado
}


void relationCallback(const ioc::ListOfRelation list)
{
	bool consistency;

	for (int i = 0; i < list.ListOfRelation.size() ; i++)
	{
		ioc::Relation rel = list.ListOfRelation[i];
		for (int j = 0; j < list.ListOfRelation.size()  ; j++)
		{
			if ( i != j)
			{
				if (list.ListOfRelation[j].interval_x == rel.interval_x) // procura relações com o primeiro elemento igual
				{
					//procura relações que o último elemento seja igual
					for (int k = 0; k < list.ListOfRelation.size()  ; k++)
					{
						if ( j != k)
						{
							if (list.ListOfRelation[k].interval_x == list.ListOfRelation[j].interval_y && list.ListOfRelation[k].interval_y == rel.interval_y )
							{

								consistency = consistencyCheck(rel, list.ListOfRelation[j], list.ListOfRelation[k] );

								if (!consistency)
								{
									inconsistecyList.ListOfRelation.push_back(rel);
									inconsistecyList.ListOfRelation.push_back(list.ListOfRelation[j]);
									inconsistecyList.ListOfRelation.push_back(list.ListOfRelation[k]);
								}

							}
						}
					}

				}
			}
		}
	}

	relation_pub.publish(inconsistecyList);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ConsistencyCheck_2");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/Relations_2", 1000, relationCallback);
  //relationString_pub = n.advertise<std_msgs::String >("InconsistentRelationsString",1000);
  relation_pub = n.advertise<ioc::ListOfRelation>("InconsistentRelations_2",1000);
  ros::spin();
  return 0;
}
