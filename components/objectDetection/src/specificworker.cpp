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
#include "specificworker.h"

/**
* \brief Default constructor
*/
SpecificWorker::SpecificWorker(MapPrx& mprx) : GenericWorker(mprx)
{
	rgb_image = cv::Mat(480,640, CV_8UC3, cv::Scalar::all(0));
	
	tx = 0;
	ty = 0;
	tz = 0;
	rx = 0;
	ry = 0;
	rz = 0;
	
	num_scene = 15;
#ifdef USE_QTGUI
	graphic->setScene(&scene);
	graphic->show();
	item_pixmap=new QGraphicsPixmapItem();
	scene.addItem(item_pixmap);
#endif
}

/**
* \brief Default destructor
*/

SpecificWorker::~SpecificWorker()
{
}

bool SpecificWorker::setParams(RoboCompCommonBehavior::ParameterList params)
{
	string name = PROGRAM_NAME;
	innermodel = new InnerModel(params[name+".innermodel"].value);
	
	id_robot=QString::fromStdString(params[name+".id_robot"].value);
	id_camera=QString::fromStdString(params[name+".id_camera"].value);
	id_camera_transform=QString::fromStdString(params[name+".id_camera_transform"].value);

	setState(States::Predict);
	
	for(uint a=0;a<10; a++)
	    ids[a] = 0;
		
	TObject taza;
	taza.name = "taza_1";
	taza.type = "cup";
	taza.idx = 1;
	taza.time.start();
	taza.explained = false;
	taza.bb.push_back(QVec::vec3(40,0,40));
	taza.bb.push_back(QVec::vec3(-40,0,40));
	taza.bb.push_back(QVec::vec3(40,0,-40));
	taza.bb.push_back(QVec::vec3(-40,-0,-40));
	taza.bb.push_back(QVec::vec3(40,80,40));
	taza.bb.push_back(QVec::vec3(-40,80,40));
	taza.bb.push_back(QVec::vec3(40,80,-40));
	taza.bb.push_back(QVec::vec3(-40,80,-40));
	listObjects.push_back(taza);
	
	TObject taza1;
	taza1.bb.clear();
	taza1.name = "taza_2";
	taza1.type = "cup";
	taza1.idx = 2;
	taza1.time.start();
	taza1.explained = false;
	taza1.bb.push_back(QVec::vec3(40,0,40));
	taza1.bb.push_back(QVec::vec3(-40,0,40));
	taza1.bb.push_back(QVec::vec3(40,0,-40));
	taza1.bb.push_back(QVec::vec3(-40,-0,-40));
	taza1.bb.push_back(QVec::vec3(40,80,40));
	taza1.bb.push_back(QVec::vec3(-40,80,40));
	taza1.bb.push_back(QVec::vec3(40,80,-40));
	taza1.bb.push_back(QVec::vec3(-40,80,-40));
	listObjects.push_back(taza1);
		
	setDefaultHeadPosition();
	yawPosition = 0.0;

	timer.start(50);
	return true;
}

void SpecificWorker::compute()
{
	qDebug() << "------------------------------------------------------------";
	
	getRgbd();
	updateinner();

#ifdef USE_QTGUI
	try
	{
		updatergbd(rgbMatrix, h, b);
	}
	catch(...){}
#endif
	getYawMotorState();

	//State machine
	switch(state)
	{		
		case States::Predict:
			qDebug() << "Predict";
			predict();
			break;
		
		case States::YoloInit:
			qDebug() << "YoloInit";
			yoloInit();
			break;
			
		case States::YoloWait:
			qDebug() << "yoloWait";
			yoloWait();
			break;
			
		case States::Compare:
			qDebug() << "compare";
			compare(pointMatrix);
			break;
			
		case States::Stress:
			qDebug() << "stress";
			stress();
			break;
			
		case States::Moving:
			qDebug() << "moving";
			setState(States::Predict);
			break;
	}
}


void SpecificWorker::predict()
{
	listVisible.clear();
	//For each synthetic object it is projected on the camera to create the labels
	for(auto &o: listObjects)
	{
		o.projbb.clear();
		InnerModelCamera *c = innermodel->getCamera("rgbd");
		std::vector<QVec> bbInCam;
		ids[o.name.at(5).digitValue()] = 1;
		bool valid = true;
		for(uint i = 0; i<o.bb.size(); i++)
		{
			QVec res = c->project(innermodel->transform("rgbd", o.bb.at(i), o.name));
			
			if(res.x() < 640 and res.x() > 0 and res.y() > 0 and res.y() < 480)// Control between 0 and 640 res
				bbInCam.push_back(res); //The transformed coordinates are added
			else
			{	
			  valid = false;
			  // The object box is deleted
			  o.box.x = 0.0; o.box.y = 0.0; o.box.w = 0.0; o.box.h = 0.0;
			  o.box.label = "";
			  o.box.prob = 0.0;
			  goto stop; //get out the loop
			}
		}
		
		stop:
		if(valid)
		{
			// Compute a bounding box of pixel coordinates
			// Sort the coordinates x
			auto xExtremes = std::minmax_element(bbInCam.begin(), bbInCam.end(),
								  [](const QVec& lhs, const QVec& rhs) { return lhs.x() < rhs.x();});
			// Sort the coordinates y
			auto yExtremes = std::minmax_element(bbInCam.begin(), bbInCam.end(),
								  [](const QVec& lhs, const QVec& rhs) { return lhs.y() < rhs.y();});
			// Take the most separated ends to build the rectangle
			o.box.x = xExtremes.first->x(); o.box.y = yExtremes.first->y(); o.box.w = xExtremes.second->x(); o.box.h = yExtremes.second->y();
			o.box.label = o.name.toStdString();
			o.box.prob = 100;
			o.projbb = bbInCam;
			o.time.restart();
			listVisible.push_back(o);
		}
	}
	
	setState(States::YoloInit);
}

void SpecificWorker::yoloInit()
{
	try
	{
		//Store the id of the request to yolo
		yoloId = yoloserver_proxy->addImage(yoloImage);
		setState(States::YoloWait);
	}catch(const Ice::Exception &e){ std::cout << e << std::endl; }
}

void SpecificWorker::yoloWait()
{
	try
	{
		//Get the result of the previous request
		yoloLabels = yoloserver_proxy->getData(yoloId);
		if(yoloLabels.isReady)
		{
			listYoloObjects.clear();
			//A real object is created for each label and added to the list of yolo objects
			for(auto y: yoloLabels.lBox)
			{
				TObject o;
				o.type = y.label;
				o.box.x = y.x; o.box.y = y.y; o.box.w = y.w; o.box.h = y.h;
				o.assigned = false;
				o.prob = y.prob;
				listYoloObjects.push_back(o);
			}
			setState(States::Compare);
		}
	}catch(const Ice::Exception &e){std::cout << e << std::endl;}
}

void SpecificWorker::compare(RoboCompRGBD::PointSeq pointMatrix)
{
	// Check if the predicted labels are on sight				
	listCreate.clear();
	listDelete.clear();
	//For each synthetic object
	for(auto &synth: listVisible)
	{
		synth.intersectArea = 0;
		synth.explained = false;
		synth.candidates.clear();
		std::vector<TCandidate> listCandidates;
		//It is compared with real objects
		for(auto &yolo: listYoloObjects)
		{
			//If it is the same type and has not been assigned yet 
			if(synth.type == yolo.type and yolo.assigned == false)
			{
				//A rectangle with the real object is created
				QRect r(QPoint(yolo.box.x,yolo.box.y),QPoint(yolo.box.w, yolo.box.h));
				//A rectangle with the sythetic object is created
				QRect rs(QPoint(synth.box.x,synth.box.y),QPoint(synth.box.w, synth.box.h));
				//Compute intersection percentage between synthetic and real
				QRect i = rs.intersected(r);
				//The area is normalized between 0 and 1 dividing by the minimum between both areas of each object
				float area = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
				//The displacement vector between the two images is calculated
				QPoint error = r.center() - rs.center();
				
				// If the area is 0 there is no intersection
				// If the error is less than twice the width of the synthetic rectangle
				if(area > 0 or error.manhattanLength()< rs.width()*2)
				{
					//A candidate is created and added to the list of candidates in an orderly manner according to the area and the error
					//The object will be placed earlier in the list the less difference there is with the original
					TCandidate tc = {area,error,&yolo};
					listCandidates.insert(std::upper_bound( listCandidates.begin(), listCandidates.end(),tc,
											  [](auto a, auto b) { return (a.area > b.area) or ((a.area==b.area) and (a.error.manhattanLength() < b.error.manhattanLength()));}), 
											  tc);
				}
			}
		}
		
		// If there are candidates, the first one is taken, assigned and the synthetic object is marked as explained
		if(listCandidates.empty() == false)
		{
			listCandidates.front().yolo->assigned = true;
			synth.intersectArea = listCandidates.front().area;
			synth.error = listCandidates.front().error;
			synth.explained = true;
		}
	}
	
	//listDelete: Extract objects not explained by measurements
	for(auto &o : listVisible)
		if(o.explained == false)
			listDelete.push_back(o);
	
	//listCreate: objects to be created due to measurements not assigned to objects
	for(auto &y: listYoloObjects)
		if(y.assigned == false)
		{
			TObject n;
			n.type = y.type;
			// Get 3D pose from RGBD
			QRect r(QPoint(y.box.x,y.box.y),QPoint(y.box.w,y.box.h));
			int idx = r.center().y()*640 + r.center().x();
			RoboCompRGBD::PointXYZ p = pointMatrix[idx];
			n.pose = QVec::vec6(p.x, p.y, p.z, 0, 0, 0);
			listCreate.push_back(n);
		}
		
	setState(States::Stress);
}

void SpecificWorker::stress()
{
	qDebug() << "Stress: size of listObjects" << listObjects.size() << "and listVisible" << listVisible.size();
	//The position of the object is corrected
	for(auto &synth: listVisible)
	{	
		//If the synthetic object is explained, only its position in the innermodel is updated
		if(synth.explained)
		{
			QPoint error = synth.error;
			InnerModelTransform *t = innermodel->getTransform(synth.name);
			// We assume that the cup does not vary in Y because it is on the table
			innermodel->updateTranslationValues(synth.name, t->getTr().x() + error.y(), t->getTr().y(), t->getTr().z() + error.x());
		}
	}
	
	//A non-existent object is removed
	if (listDelete.size() > 0)
	{
		for(auto &n: listDelete)
		{
			if(n.type != "cup")  //Only cups for now
				continue;
			qDebug() << "	delete object " << n.name;
			
			//It is removed from the innermodel
			try{innermodel->removeNode(n.name);}
			catch(QString s){ qDebug() << s;}
			
			//It is removed from the list of synthetic objects
			for(uint i = 0; i<listObjects.size(); i++)
			{
				if(n.idx == listObjects[i].idx)
				{
					listObjects.erase(listObjects.begin()+i);
					ids[n.name.at(5).digitValue()] = 0;
				}
			}
			
			for(uint i = 0; i<listVisible.size(); i++)
				if(n.idx == listVisible[i].idx)
					listVisible.erase(listVisible.begin()+i);
		}
	}
	
	//New objects are added
	if(listObjects.size() < MAX_OBJECTS)
	  for(auto &n: listCreate)
	  {
		  if(n.type != "cup")  //Only cups for now
			  continue;
		  
		  //countertopA is the table
		  QVec ot = innermodel->transform("countertopA", n.pose, "rgbd");
		  // create a new unused name
		  QString name = QString::fromStdString(n.type);
		  //New object is created
		  TObject to;
		  
		  to.type = "cup";
		  to.idx = getId();
		  to.name = "taza_" + QString::number(to.idx);
		  to.explained = false;
		  to.prob = 100;
		  to.time.start();
		  
		  //It is added to the innermodel
		  try{innermodel->newTransform(to.name, "", innermodel->getNode("countertopA"), ot.x(), ot.y(), ot.z(), 0, 0, 0);}
		  catch(QString s){ qDebug() << s;}
		  
		  to.bb.push_back(QVec::vec3(40,0,40));
		  to.bb.push_back(QVec::vec3(-40,0,40));
		  to.bb.push_back(QVec::vec3(40,0,-40));
		  to.bb.push_back(QVec::vec3(-40,-0,-40));
		  to.bb.push_back(QVec::vec3(40,80,40));
		  to.bb.push_back(QVec::vec3(-40,80,40));
		  to.bb.push_back(QVec::vec3(40,80,-40));
		  to.bb.push_back(QVec::vec3(-40,80,-40));
		  
		  //If the box is in the plane you can add the object to the lists
		  InnerModelCamera *c = innermodel->getCamera("rgbd");
		  bool valid = true;
		  for(auto i: to.bb)
		  {
			  QVec res = c->project(innermodel->transform("rgbd", i, to.name));
			  // Control between 0 and 640 res
			  if(res.x() >= 640 or res.x() <= 0 or res.y() <= 0 or res.y() >= 480 or std::isnan(res.x()) or std::isnan(res.y()))
			  {
				  valid = false;
				  ids[to.idx] = 0; //Revisar problemas con nombres. Se llena el vector al acceder a getID
				  try{innermodel->removeNode(to.name);}
				  catch(...){qDebug() << "Trying to remove non-existent node";}
				  goto end; //get out the loop
			  }
		  }
		  end:
		  if(valid == true)
		  {
			  listObjects.push_back(to);
			  listVisible.push_back(to); //The object is added
		  }
	  }
	 
	setState(States::Predict);
}

//////////////////////////////////////////////////////////////////
//// PRIVATE METHODS 
//////////////////////////////////////////////////////////////////

void SpecificWorker::setState(States s)
{
	static bool first = true;
	if (s == state and not first)
		return;
	first = false;
	state = s;
}

void SpecificWorker::setDefaultHeadPosition()
{
    
    RoboCompJointMotor::MotorGoalPosition head_pitch_joint, head_yaw_joint;
    
    head_pitch_joint.name = "head_pitch_joint";
	head_pitch_joint.position = 0.9;
	
	head_yaw_joint.name = "head_yaw_joint";
	head_yaw_joint.position = 0.0;

	try
	{
		jointmotor_proxy->setPosition(head_pitch_joint);
		jointmotor_proxy->setPosition(head_yaw_joint);
	}catch(const Ice::Exception &e){qDebug() << "Init joints exception";}
}

void SpecificWorker::getRgbd()
{
	try
	{	
		rgbd_proxy->getRGB(rgbMatrix,h,b); 	
		rgbd_proxy->getXYZ(pointMatrix,h,b);
	}
	catch(const Ice::Exception &e){ std::cout << e << std::endl; return; };
	
}

void SpecificWorker::updateinner()
{
	try
	{
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	}
	catch(const Ice::Exception &e){std::cout << e << std::endl;}
	
	try
	{
		MotorList motors;
		motors.push_back("head_yaw_joint");
		motors.push_back("head_pitch_joint");
		MotorStateMap motors_states = jointmotor_proxy->getMotorStateMap(motors);
		for(auto motor:motors)
		{
			innermodel->updateJointValue(QString::fromStdString(motor),motors_states[motor].pos);
		}
	}
	catch(...){qFatal("Can't access motors");}
}

void SpecificWorker::updatergbd(const RoboCompRGBD::ColorSeq &rgbMatrix, const RoboCompJointMotor::MotorStateMap &h, const RoboCompGenericBase::TBaseState &b)
{
	yoloImage.data.resize(rgbMatrix.size()*3);
	yoloImage.w=640; yoloImage.h=480;
	int j=0;
	for(unsigned int i=0; i<rgbMatrix.size(); i++, j=i*3)
	{
		int row = (i/480), column = i-(row*640);
		rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
		yoloImage.data[j] = rgbMatrix[i].blue;
		yoloImage.data[j+1] = rgbMatrix[i].green;
		yoloImage.data[j+2] = rgbMatrix[i].red;	
	}
	
	cv::Mat dest;
	cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
	
	for(auto yolo: listYoloObjects)  
	{
		if( yolo.prob > 35)
		{
			CvPoint p1, p2, pt;
			p1.x = int(yolo.box.x); p1.y = int(yolo.box.y);
			p2.x = int(yolo.box.w); p2.y = int(yolo.box.h);
			pt.x = int(yolo.box.x); pt.y = int(yolo.box.y) + ((p2.y - p1.y) / 2);
			cv::rectangle(dest, p1, p2, cv::Scalar(0, 0, 255), 4);
			auto font = cv::FONT_HERSHEY_SIMPLEX;
			cv::putText(dest, yolo.type + " " + std::to_string(int(yolo.prob)) + "%", pt, font, 1, cv::Scalar(255, 255, 255), 1);
		}
	}
	
	for(auto o: listObjects)
	{
		CvPoint p1, p2;
		p1.x = int(o.box.x); p1.y = int(o.box.y);
		p2.x = int(o.box.w); p2.y = int(o.box.h);
		cv::rectangle(dest, p1, p2, cv::Scalar(0, 255, 5), 4);
	}

	QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	item_pixmap->setPixmap(QPixmap::fromImage(image));
	scene.update();
}

void SpecificWorker::getYawMotorState()
{
	try
	{
		RoboCompJointMotor::MotorState mstateMap = jointmotor_proxy->getMotorState("head_yaw_joint");
		checkTime();
		
		if(fabs(yawPosition - mstateMap.pos) > 0.05f)
			setState(States::Moving);
		
		yawPosition = mstateMap.pos;
	}catch(...){qDebug()<<"Error motor access";}
}

void SpecificWorker::checkTime()
{
	moved = false;
	RoboCompJointMotor::MotorGoalPosition head_yaw_joint, head_pitch_joint;

	head_yaw_joint.name = "head_yaw_joint"; 	//side to side
	head_pitch_joint.name = "head_pitch_joint"; //up and down
		  
	for(auto &i: listObjects)
	{
		if (i.time.elapsed() > 15000 and moved == false)
		{
			moved = true;
			
			for(auto &j: listObjects)
			  	j.time.restart();
			
			QVec res = innermodel->transform("rgbd", i.bb.at(0), i.name);
			
			QPoint center = QPoint(170, 240);
			
			head_yaw_joint.position = atan2(res.x() - center.x(), res.z());
			head_pitch_joint.position = atan2(res.z(), center.y() - res.y());
			
			try
			{
				jointmotor_proxy->setPosition(head_yaw_joint);
				jointmotor_proxy->setPosition(head_pitch_joint);
				sleep(2); //wait for the engines
			}catch(const Ice::Exception &e){std::cout << e <<endl;}
		}
	}
}

int SpecificWorker::getId()
{
	uint j;
	for(j = 1; j<10; j++)
	{
		if(ids[j] == 0)
		{
			ids[j] = 1;
			return (j);
		}
	}
	return -1;
}


////////////////////////////////////////////
/// SUBSCRIPTION for interface AprilTags
////////////////////////////////////////////

void SpecificWorker::newAprilTagAndPose(const tagsList& tags, const TBaseState& bState, const MotorStateMap& hState)
{
}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	QMutexLocker locker(&april_mutex);
	if(tags.size()==0)
		return;
	this->tags = tags;   
}
