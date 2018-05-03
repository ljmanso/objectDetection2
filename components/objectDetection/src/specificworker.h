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
 #include <opencv2/core/core.hpp>
 #include <opencv2/highgui/highgui.hpp>
 #include <opencv2/imgproc/imgproc.hpp>
#endif

#include <genericworker.h>

#ifndef Q_MOC_RUN
	#include <innermodel/innermodel.h>
	#include <innermodel/innermodelviewer.h>
	#include "time.h"
#endif

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

#define MAX_OBJECTS 5
#define CELL_WIDTH 40
#define CELL_HEIGHT 40

#include <cmath>
#include <cstdlib>
#include <ctime>

#include <boost/functional/hash.hpp>
#include <unordered_map>
#include <fcl/collision_func_matrix.h>

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
	void updateTableMap();			// Cold and warm function
	
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
	
	// Map creation to model the table
	struct Key
	{
		long int x;
		long int z;
	
		public:
			Key(): x(0), z(0) {};
			Key(long int &&x, long int &&z): x(std::move(x)), z(std::move(z)){};
			Key(long int &x, long int &z): x(x), z(z){};
			Key(const long int &x, const long int &z): x(x), z(z){};
			bool operator==(const Key &other) const
				{ return x == other.x && z == other.z; };
			void print(std::ostream &os) const 	{ os << " x:" << x << " z:" << z; };
	};
		
	struct Value
	{
		float x;
		float y;
		float z;
		int temperature;	// 0 cold ----> 100 hot
	};

	struct KeyHasher
	{
		std::size_t operator()(const Key& k) const
		{
			using boost::hash_value;
			using boost::hash_combine;

			// Start with a hash value of 0    .
			std::size_t seed = 0;

			// Modify 'seed' by XORing and bit-shifting in one member of 'Key' after the other:
			hash_combine(seed,hash_value(k.x));
			hash_combine(seed,hash_value(k.z));
			return seed;
		};
	};	
			
	typedef	std::unordered_map<Key, Value, KeyHasher> map;
	map tableMap;
	
	void cool(std::pair<Key, Value> cell);	// Cools the map, more if there is an object over this area

};

#endif
