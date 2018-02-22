/*
 *    Copyright (C) 2015 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

/**
       \brief
       @author authorname
*/

#ifndef SPECIFICWORKER_H
#define SPECIFICWORKER_H

#ifndef Q_MOC_RUN
//  #include <pcl/point_cloud.h>
//  #include <pcl/pcl_base.h>
//  #include <pcl/point_types.h>
//  #include <pcl/filters/passthrough.h>
//  #include <pcl/segmentation/extract_clusters.h>
//  #include <pcl/filters/statistical_outlier_removal.h>
//  #include <pcl/conversions.h>
//  #include <pcl/point_types_conversion.h>
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
 //#include <pcl/surface/convex_hull.h>
 //#include <pcl/surface/concave_hull.h>
 //#include <boost/thread/thread.hpp>
#endif

#include <genericworker.h>

#ifndef Q_MOC_RUN
	#include <innermodel/innermodel.h>
	#include <innermodel/innermodelviewer.h>
	//#include "color_segmentation/Segmentator.h"
	//#include "shapes/table.h"
	//#include "descriptors/descriptors.h"
	#ifdef USE_QTGUI
		//#include "viewer/viewer.h"
		#include <QGraphicsPixmapItem>
	#endif
	//#include "pointcloud/pointcloud.h"
	#include "time.h"
#endif


#define DEBUG 1
#define SAVE_DATA 0
#define THRESHOLD 0.8
#define MEDIDA 1.
#define offset_object 0.

#define MAX_OBJECTS 5

#define SUB(dst, src1, src2) \
  { \
    if ((src2)->tv_nsec > (src1)->tv_nsec) { \
      (dst)->tv_sec = (src1)->tv_sec - (src2)->tv_sec - 1; \
      (dst)->tv_nsec = ((src1)->tv_nsec - (src2)->tv_nsec) + 1000000000; \
    } \
    else { \
      (dst)->tv_sec = (src1)->tv_sec - (src2)->tv_sec; \
      (dst)->tv_nsec = (src1)->tv_nsec - (src2)->tv_nsec; \
    } \
  }

//typedef pcl::PointXYZRGB PointT;

//using namespace computepointcloud;
#include <cmath>
#include <cstdlib>
#include <ctime>

enum class States { YoloInit, YoloWait, Predict, Compare, Stress, Moving };

class SpecificWorker : public GenericWorker
{
	string image_path;
	float tx, ty, tz, rx, ry, rz;
	QString id_robot, id_camera,id_camera_transform;

	InnerModel *innermodel;

	//For poses calculation respect to the canonical one
	tagsList tags;
	QMutex april_mutex;
	int num_scene;

	//Image of the current view for opencv
	cv::Mat rgb_image;

	//Point cloud grabing
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	RoboCompRGBD::PointSeq pointMatrix;

#ifdef USE_QTGUI
	QGraphicsPixmapItem* item_pixmap;
	QGraphicsScene scene;
#endif

Q_OBJECT
public:
	SpecificWorker(MapPrx& mprx);
	~SpecificWorker();

	bool setParams(RoboCompCommonBehavior::ParameterList params);

	//// SERVANT for ObjectDetection.ice
	bool findObjects(const StringVector &objectsTofind, ObjectVector &objects){return false;};	

	//Method of Interface AprilTags.ice
	void newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState);
	void newAprilTag(const tagsList &tags);

public slots:
	void compute();

private:
#ifdef USE_QTGUI
	void updatergbd(const RoboCompRGBD::ColorSeq &rgbMatrix, const RoboCompJointMotor::MotorStateMap &h, const RoboCompGenericBase::TBaseState &b);
#endif
	
private:
	// State machine
	States state;
	void setState(States state);
	void predict(); 									// Project objects on camera creating yoloSLabels
	void yoloInit();									// Access to yolo is divided into two states so that it has time to process and send the result
	void yoloWait();									// Wait for yolo server
	void compare(RoboCompRGBD::PointSeq pointMatrix);	// Compare real and synthetic objects
	void stress();										// Correct a stressful situation
	
	int yoloId; 		// Yolo server id
	float yawPosition; 	// Engine yaw position
	int ids[10]; 		// Auxiliary array for cups ID (only allows 10 objects)
	bool moved; 		// If you're moving because the previous object, you mustn't move in the same check by other object

	void setDefaultHeadPosition(); 	// Initial head position 
	void getRgbd();					// Gets camera information
	void updateinner();				// Update motor state
	void getYawMotorState();		// Check if yaw motor is moving
	void checkTime(); 				// Check time to change head position
	int getId(); 					// Return the first free id
	
	//Yolo
	RoboCompYoloServer::Image yoloImage;
	RoboCompYoloServer::Labels yoloLabels;
	
	//Objects geometry
	struct TObject
	{
		QString name;			//instance
		std::string type;		//class
		bool explained;		
		int idx;
		std::vector<QVec> bb;	//bounding box
		std::vector<QVec> projbb;
		float intersectArea;
		QPoint error;
		QVec pose;
		bool assigned;
		float prob;
		QElapsedTimer time;
		RoboCompYoloServer::Box box;
		std::vector<std::pair<float, QPoint>> candidates;
		TObject() 
		{
			name = "";
			type = "cup";
			explained = false;
			intersectArea = 0;
			idx = -1;
		};
	};
	
	struct TCandidate
	{
	  float area;
	  QPoint error;
	  TObject *yolo;
	};
	
	typedef std::vector<TObject> TObjects;

	TObjects listObjects;
	TObjects listYoloObjects;
	TObjects listCreate;
	TObjects listDelete;
	TObjects listVisible;
};

#endif
