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
#include "std_msgs/Header.h"
#include "/home/benner/catkin_ws/devel/include/ioc/listOfRoi.h"
#include "/home/benner/catkin_ws/devel/include/ioc/RegionOfInterest.h"
#include "/home/benner/catkin_ws/devel/include/ioc/Relation.h"
#include "/home/benner/catkin_ws/devel/include/ioc/ListOfRelation.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/cache.h>
#include <sensor_msgs/PointCloud.h>
#include <math.h>
using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace message_filters;

#define PI 3.14159265

static const std::string OPENCV_WINDOW = "Image window";

enum Situation
{
	desconexas,
	conectadaExternamente,
	interseccionando,
	conectadaInternamente,
	dentro
};


struct Reta
   {
          float angular;
          float linear;
   };

struct LinhaDeVisao
	{
		char intervalo_1;
		char intervalo_2;
		Situation situacao;
		Reta toptotop;
		Reta bottobot;
		Reta toptobot;
		Reta bottotop;
	};
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
void SortInterval();
ioc::ListOfRelation pointOfView(ioc::ListOfRelation list);
ioc::ListOfRelation mainRelations(ioc::ListOfRelation list);
ioc::listOfRoi getMainInterval(ioc::ListOfRelation list);
LinhaDeVisao getLineOfSight (ioc::listOfRoi list);
ioc::ListOfRelation getObservation(ioc::listOfRoi listRoi, ioc::ListOfRelation listRelation, LinhaDeVisao linhadeVisao);
ioc::ListOfRelation getObservationInterval (ioc::ListOfRelation list, ioc::listOfRoi listRoi);
//ioc::ListOfRelation getObservation(ioc::ListOfRelation observation);
//void checkRegion(int cena, ioc::Relation rel);


ioc::listOfRoi publishRoi;

cv_bridge::CvImagePtr cv_ptr;

ioc::ListOfRelation husky_1_relations;
ioc::ListOfRelation husky_2_relations;

image_transport::Subscriber image_sub_;
image_transport::Publisher image_pub_;

Mat drawing;



int Lenght = 50;
LinhaDeVisao cena[50];
char husky_1_perspective[22];

void callback(const ioc::listOfRoiConstPtr roi, const ioc::ListOfRelationConstPtr& relations_1, const ioc::ListOfRelationConstPtr& relations_2)
//void callback(const sensor_msgs::ImageConstPtr& msg, const sensor_msgs::ImageConstPtr& list1)
  {

	  publishRoi =  *roi;
      husky_1_relations = *relations_1;
      husky_2_relations = *relations_2;
      thresh_callback( 0, 0 );

  }

void thresh_callback(int, void* )
  {


    SortInterval();
    publishRoi.listOfROI.clear();
    drawing = drawing(Rect(0,0,640,450)).clone();

    ioc::ListOfRelation observerHusky_1 = pointOfView(husky_1_relations);
    ioc::ListOfRelation observerHusky_2 = pointOfView(husky_2_relations);

    ioc::ListOfRelation observerMainRelation_1 = mainRelations(observerHusky_1);
    ioc::ListOfRelation observerMainRelation_2 = mainRelations(observerHusky_2);

    ioc::listOfRoi observerMainInterval_1 = getMainInterval(observerMainRelation_1);
    ioc::listOfRoi observerMainInterval_2 = getMainInterval(observerMainRelation_2);

    LinhaDeVisao lineOfSight_1 = getLineOfSight (observerMainInterval_1);
    LinhaDeVisao lineOfSight_2 = getLineOfSight (observerMainInterval_2);

    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );

  }

ioc::ListOfRelation getObservationInterval (ioc::ListOfRelation list, ioc::listOfRoi listRoi)
{
	ioc::ListOfRelation relations;

	for (int i = 0; i < list.ListOfRelation.size(); i++)
	{
		if (list.ListOfRelation[i].interval_x ==  listRoi.listOfROI[0].name || list.ListOfRelation[i].interval_x ==  listRoi.listOfROI[1].name)
		{
			relations.ListOfRelation.push_back(list.ListOfRelation[i]);
		}
	}

	return relations;

}

ioc::ListOfRelation getObservation(ioc::listOfRoi listRoi, ioc::ListOfRelation listRelation, LinhaDeVisao linhadeVisao)
{
	ioc::ListOfRelation list;


	if (linhadeVisao.situacao == desconexas)
	{

	}else if (linhadeVisao.situacao == conectadaExternamente)
	{

	}else if (linhadeVisao.situacao == interseccionando)
	{

	}else if (linhadeVisao.situacao == conectadaInternamente)
	{

	}else if (linhadeVisao.situacao == dentro)
	{

	}

	return list;
}

LinhaDeVisao getLineOfSight (ioc::listOfRoi list)
{
	LinhaDeVisao lineOfsight;

	int top_x_1;
	int top_x_2;
	int top_y_1;
	int top_y_2;
	int bot_x_1;
	int bot_x_2;
	int bot_y_1;
	int bot_y_2;

	 Reta toptoTop;
	 Reta bottoBot;
	 Reta toptoBot;
	 Reta bottoTop;

	 Point bottom1;
	 Point bottom2;

	 Point center1;
	 Point center2;

	 Point top1;
	 Point top2;

   lineOfsight.intervalo_1 = list.listOfROI[0].name;
   lineOfsight.intervalo_2 = list.listOfROI[1].name;

   int center_1_x = list.listOfROI[0].x_offset + (list.listOfROI[0].width / 2);
   int center_1_y = list.listOfROI[0].y_offset + (list.listOfROI[0].height / 2);

   center1.x = center_1_x;
   center1.y = center_1_y;

   int center_2_x = list.listOfROI[1].x_offset + (list.listOfROI[1].width / 2);
   int center_2_y = list.listOfROI[1].y_offset + (list.listOfROI[1].height / 2);

   center2.x = center_2_x;
   center2.y = center_2_y;

   int raio_1 = list.listOfROI[0].width;
   int raio_2 = list.listOfROI[1].width;

   circle(drawing,center1,raio_1, cv::Scalar(100, 100, 200), 2, CV_AA);
   circle(drawing,center2,raio_2, cv::Scalar(100, 100, 200), 2, CV_AA);

   // achar a equação da reta que liga os dois centros

   int delta_x = center_2_x - center_1_x;
   int delta_y = center_2_y - center_1_y;
   float coefAngular = delta_y/delta_x;

   //achar a distancia entre os dois centros

   float dist = sqrt(pow((center_1_x - center_2_x),2) + pow((center_1_y - center_2_y),2));

     float angulo = atan(coefAngular*(-1));
     int raio_x_1 = raio_1 * cos((PI/2) - angulo);
     int raio_y_1 = raio_1 * sin((PI/2) - angulo);

     int raio_x_2 = raio_2 * cos((PI/2) - angulo);
     int raio_y_2 = raio_2 * sin((PI/2) - angulo);

     if (coefAngular > 0)
     {
    	  top_x_1 = center_1_x - raio_x_1 ;
		  top_y_1 = center_1_y - raio_y_1 ;

		  bot_x_1 = center_1_x + raio_x_1 ;
		  bot_y_1 = center_1_y + raio_y_1 ;

		  //-------------------------------------------

		  top_x_2 = center_2_x - raio_x_2 ;
		  top_y_2 = center_2_y - raio_y_2 ;

		  bot_x_2 = center_2_x + raio_x_2 ;
		  bot_y_2 = center_2_y + raio_y_2 ;


     }else if  (coefAngular < 0)
     {
    	  top_x_1 = center_1_x - raio_x_1 ;
		  top_y_1 = center_1_y - raio_y_1 ;

		  bot_x_1 = center_1_x + raio_x_1 ;
		  bot_y_1 = center_1_y + raio_y_1 ;

		 //-------------------------------------------

    	  top_x_2 = center_2_x - raio_x_2 ;
		  top_y_2 = center_2_y - raio_y_2 ;

		  bot_x_2 = center_2_x + raio_x_2 ;
		  bot_y_2 = center_2_y + raio_y_2 ;

     }else // se for igual zero
     {
    	 if (center_1_x == center_2_x)
		   {
			   top_x_1 = center_1_x + raio_1;
			   top_y_1 = center_1_y;

			   top_x_2 = center_2_x + raio_2;
			   top_y_2 = center_2_y;

			   bot_x_1 = center_1_x + raio_1;
			   bot_y_1 = center_1_y;

			   bot_x_2 = center_2_x + raio_2;
			   bot_y_2 = center_2_y;
		   }else
		   {
			   top_x_1 = center_1_x;
			   top_y_1 = center_1_y - raio_1;

			   top_x_2 = center_2_x;
			   top_y_2 = center_2_y - raio_2;

			   bot_x_1 = center_1_x;
			   bot_y_1 = center_1_y - raio_1;

			   bot_x_2 = center_2_x;
			   bot_y_2 = center_2_y - raio_2;

		   }
     }

     if (dist == abs(raio_1 - raio_2))
     {
    	 lineOfsight.situacao = conectadaInternamente;
     }else if (dist < abs(raio_1 - raio_2))
     {
    	 lineOfsight.situacao = dentro;
     }

     if (dist > abs(raio_1 - raio_2))
    {

   	  if (dist == (raio_1 + raio_2))
   	  {
   		 lineOfsight.situacao = conectadaExternamente;
   	  }
   	  else if (dist < (raio_1 + raio_2))
   	  {
   		  lineOfsight.situacao = interseccionando;
   	  }
     //desenhando as linhas de visao

      top1.x = top_x_1 ; top1.y = top_y_1 ;
	  top2.x = top_x_2 ; top2.y = top_y_2 ;

	  bottom1.x = bot_x_1 ; bottom1.y = bot_y_1 ;
	  bottom2.x = bot_x_2 ; bottom2.y = bot_y_2 ;

	  line(drawing, top1, top2, cv::Scalar(100, 100, 200), 2, CV_AA);
	  line(drawing, bottom1, bottom2, cv::Scalar(100, 100, 200), 2, CV_AA);


		 int deltaXToptoTop = (top_x_2 - top_x_1);
		 int deltaYToptoTop = (top_y_2 - top_y_1);

		 float coefAngularToptoTop =  deltaYToptoTop / deltaXToptoTop;
		 float coefLinearToptoTop = top_y_1 - (coefAngularToptoTop * top_x_1);

		 toptoTop.angular = coefAngularToptoTop;
		 toptoTop.linear = coefLinearToptoTop;
		 //-----------------------------------------------------------------------
		 int deltaXBottoBot = (bot_x_2 - bot_x_1);
		 int deltaYBottoBot = (bot_y_2 - bot_y_1);

		 float coefAngularBottoBot =  deltaYBottoBot / deltaXBottoBot;
		 float coefLinearBottoBot = bot_y_1 - (coefAngularBottoBot * bot_x_1);

		 bottoBot.angular = coefAngularBottoBot;
		 bottoBot.linear = coefLinearBottoBot;

     //-----------------------------------------------------------------------
     if (dist > (raio_1 + raio_2))
     {
    	 lineOfsight.situacao = desconexas;

    	 int deltaXBottoTop = (top_x_2 - bot_x_1);
		 int deltaYBottoTop = (top_y_2 - bot_y_1);

		 float coefAngularBottoTop =  deltaYBottoTop / deltaXBottoTop;
		 float coefLinearBottoTop = bot_y_1 - (coefAngularBottoTop * bot_x_1);

		 bottoTop.angular = coefAngularBottoTop;
		 bottoTop.linear = coefLinearBottoBot;

		 line(drawing, top1, bottom2, cv::Scalar(100, 100, 200), 2, CV_AA);

		 //-----------------------------------------------------------------------
		 int deltaXToptoBot = (bot_x_2 - top_x_1);
		 int deltaYToptoBot = (bot_y_2 - top_y_1);

		 float coefAngularToptoBot =  deltaYToptoBot / deltaXToptoBot;
		 float coefLinearToptoBot = top_y_1 - (coefAngularToptoBot * top_x_1);

		 toptoBot.angular = coefAngularToptoBot;
		 toptoBot.linear = coefLinearToptoBot;

		 line(drawing, bottom1, top2, cv::Scalar(100, 100, 200), 2, CV_AA);
     }

   }

   lineOfsight.bottobot = bottoBot;
   lineOfsight.bottotop = bottoTop;
   lineOfsight.toptobot = toptoBot;
   lineOfsight.toptotop = toptoTop;

   return lineOfsight;

}
ioc::listOfRoi getMainInterval(ioc::ListOfRelation list)
{
	ioc::listOfRoi roiList;
	for (int i=0; i< publishRoi.listOfROI.size(); i++)
	{
		if(publishRoi.listOfROI[i].name == list.ListOfRelation[0].interval_y || publishRoi.listOfROI[i].name == list.ListOfRelation[1].interval_y )
		{
			roiList.listOfROI.push_back(publishRoi.listOfROI[i]);
		}
	}

	return roiList;
}

ioc::ListOfRelation mainRelations(ioc::ListOfRelation list)
{
	for (int i=0; i< list.ListOfRelation.size();i++)
	{
		if (i==0 && (list.ListOfRelation[i].relation == "p" || list.ListOfRelation[i].relation == "m" || list.ListOfRelation[i].relation == "o" || list.ListOfRelation[i].relation == "s")) // quer dizer que precede a todos
		{
			//então vamos pegar apenas os dois primeiros intervalos
			list.ListOfRelation.push_back(list.ListOfRelation[i]);
			list.ListOfRelation.push_back(list.ListOfRelation[i+1]);
			return list;
		}else if ((i != list.ListOfRelation.size() -1) && (list.ListOfRelation[i].relation == "p" || list.ListOfRelation[i].relation == "m" || list.ListOfRelation[i].relation == "o" || list.ListOfRelation[i].relation == "s"))
		{
			list.ListOfRelation.push_back(list.ListOfRelation[i]);
			list.ListOfRelation.push_back(list.ListOfRelation[i-1]);
			return list;
		}else
		{
			list.ListOfRelation.push_back(list.ListOfRelation[i]);
			list.ListOfRelation.push_back(list.ListOfRelation[i-1]);
			return list;
		}


	}
	return list;
}

ioc::ListOfRelation pointOfView(ioc::ListOfRelation list)
{
	ioc::ListOfRelation listOfObserver;
	//localiza o outro observador
	for (int i = 0; i < list.ListOfRelation.size(); i++)
	{
		if (list.ListOfRelation[i].interval_x == 'Y')
		{
			listOfObserver.ListOfRelation.push_back(list.ListOfRelation[i]);
		}

	}

	return listOfObserver;
}
void SortInterval()
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
  rect_pub = nh.advertise< ioc::listOfRoi >("topView",1000);

  message_filters::Subscriber<ioc::ListOfRelation> relation_sub_1(nh, "/husky_1/Relations", 1);
  message_filters::Subscriber<ioc::ListOfRelation> relation_sub_2(nh, "/husky_2/Relations", 1);
  message_filters::Subscriber<ioc::listOfRoi> roi_sub(nh, "/hector/ROI", 1);
  TimeSynchronizer<ioc::listOfRoi, ioc::ListOfRelation, ioc::ListOfRelation > sync(roi_sub, relation_sub_1, relation_sub_2, 1);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3));

  ros::spin();

  return 0;
}


