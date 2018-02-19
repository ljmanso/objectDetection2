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
	
#ifdef USE_QTGUI
	//viewer = boost::shared_ptr<Viewer>(new Viewer(MEDIDA));
#endif
	
	tx = 0;
	ty = 0;
	tz = 0;
	rx = 0;
	ry = 0;
	rz = 0;

	test=false;
	//let's set the sizes
	//table->set_board_size(500/MEDIDA,30/MEDIDA,500/MEDIDA);

	num_scene = 15;
printf("%s %d\n", __FILE__, __LINE__);
#ifdef USE_QTGUI
printf("%s %d\n", __FILE__, __LINE__);
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
	if(params[name+".test"].value=="1")
	{
		std::cout<<"Modo test activo"<<std::endl;
		test=true;
	}
	if (test)
	{
		innermodel->updateJointValue(QString::fromStdString("head_pitch_joint"),0.710744);
		innermodel->updateJointValue(QString::fromStdString("head_yaw_joint"),0.0102265);
	}
	
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
	
/*	
 * Para leer del innermodel
 * */
// 	InnerModelTransform *taza_1 = innermodel->getTransform("taza_1");
// 	TObject taza1;
// 	taza1.bb.clear();
// 	taza1.name = taza_1->id;
// 	//taza1.type = taza_1->type;
// 	QVec ot = QVec::vec6(taza_1->backtX, taza_1->backtY, taza_1->backtZ, 0, 0, 0);
// 	taza1.pose = innermodel->transform("countertopA", ot, "rgbd" );
// 	taza1.idx = 1;
// 	taza1.time.start();
// 	taza1.explained = false;
// 	taza1.bb.push_back(QVec::vec3(50,0,50));
// 	taza1.bb.push_back(QVec::vec3(-50,0,50));
// 	taza1.bb.push_back(QVec::vec3(50,0,-50));
// 	taza1.bb.push_back(QVec::vec3(-50,-0,-50));
// 	taza1.bb.push_back(QVec::vec3(50,100,50));
// 	taza1.bb.push_back(QVec::vec3(-50,100,50));
// 	taza1.bb.push_back(QVec::vec3(50,100,-50));
// 	taza1.bb.push_back(QVec::vec3(-50,100,-50));
// 	listObjects.push_back(taza1);
// 	
// 	InnerModelTransform *taza_2 = innermodel->getTransform("taza_2");
// 	TObject taza2;
// 	taza2.bb.clear();
// 	taza2.name = taza_2->id;
// 	//taza1.type = taza_1->type;
// 	QVec ot2 = QVec::vec6(taza_2->backtX, taza_2->backtY, taza_2->backtZ, 0, 0, 0);
// 	taza2.pose = innermodel->transform("countertopA", ot2, "rgbd" );
// 	taza2.idx = 2;
// 	taza2.time.start();
// 	taza2.explained = false;
// 	taza2.bb.push_back(QVec::vec3(50,0,50));
// 	taza2.bb.push_back(QVec::vec3(-50,0,50));
// 	taza2.bb.push_back(QVec::vec3(50,0,-50));
// 	taza2.bb.push_back(QVec::vec3(-50,-0,-50));
// 	taza2.bb.push_back(QVec::vec3(50,100,50));
// 	taza2.bb.push_back(QVec::vec3(-50,100,50));
// 	taza2.bb.push_back(QVec::vec3(50,100,-50));
// 	taza2.bb.push_back(QVec::vec3(-50,100,-50));
// 	listObjects.push_back(taza2);
	
// 	for(int i = 0; i<2; i++){
// 		TObject taza;
// 		taza.bb.clear();
// 		QString name = "taza_" + (i+1);
// 		taza.name = innermodel->getTransform(name)->id;
// 		//taza1.type = taza_1->type;
// 		taza.idx = 1;
// 		taza.time.start();
// 		taza.explained = false;
// 		taza.bb.push_back(QVec::vec3(50,0,50));
// 		taza.bb.push_back(QVec::vec3(-50,0,50));
// 		taza.bb.push_back(QVec::vec3(50,0,-50));
// 		taza.bb.push_back(QVec::vec3(-50,-0,-50));
// 		taza.bb.push_back(QVec::vec3(50,100,50));
// 		taza.bb.push_back(QVec::vec3(-50,100,50));
// 		taza.bb.push_back(QVec::vec3(50,100,-50));
// 		taza.bb.push_back(QVec::vec3(-50,100,-50));
// 		listObjects.push_back(taza);
// 	}
	
	setDefaultHeadPosition();
	yawPosition = 0.0;

	timer.start(50);
	return true;
}

void SpecificWorker::compute()
{
	qDebug() << "------------------------------------------------------------";
	RoboCompRGBD::ColorSeq rgbMatrix;
	RoboCompJointMotor::MotorStateMap h;
	RoboCompGenericBase::TBaseState b;
	//RoboCompRGBD::DepthSeq depthMatrix;
	RoboCompRGBD::PointSeq pointMatrix;
	
	try
	{	rgbd_proxy->getRGB(rgbMatrix,h,b); 	
		rgbd_proxy->getXYZ(pointMatrix,h,b);
	}
	catch(const Ice::Exception &e)
	{ std::cout << e << std::endl; return; };
			
	if (!test)
		updateinner();

	
#ifdef USE_QTGUI
	try
	{
		updatergbd(rgbMatrix, h, b);
		//viewer->update();
	}
	catch(...){}
#endif

	try
	{
		RoboCompJointMotor::MotorState mstateMap = jointmotor_proxy->getMotorState("head_yaw_joint");
		///Calcular la velocidad con una variable de donde estaba y donde estoy entre el tiempo
		checkTime();
		
		if(fabs(yawPosition - mstateMap.pos) > 0.05f)
			setState(States::Moving);
		
		yawPosition = mstateMap.pos;
	}catch(...)
	  {qDebug()<<"Error motor access";}
	
// 	if(listObjects.size()>3)
// 	  cin.get();
	
	switch(state)
	{		
		// project objects on camera creating yoloSLabels
		case States::Predict:
			qDebug() << "Predict";
			predict();
			break;
		
		// Access to yolo is divided into two states so that it has time to process and send the result
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
			
		// correct a stressful situation
		case States::Stress:
			qDebug() << "stress";
			stress();
			break;
			
		case States::Moving:
			qDebug() << "moving";
// 			sleep(1);
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
			//res.print("Res " + o.name);
			
			
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
			o.box.x = xExtremes.first->x(); o.box.y = yExtremes.first->y(); o.box.w = xExtremes.second->x(); o.box.h = yExtremes.second->y() ;
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
	try{
		//Store the id of the request to yolo
		yoloId = yoloserver_proxy->addImage(yoloImage);
		setState(States::YoloWait);
	}catch(const Ice::Exception &e){ std::cout << e << std::endl; }
}

void SpecificWorker::yoloWait()
{
	try{
		//Get the result of the previous request
		yoloLabels = yoloserver_proxy->getData(yoloId);
		if( yoloLabels.isReady ){
			listYoloObjects.clear();
			//A real object is created for each label and added to the list of yolo objects
			for(auto y: yoloLabels.lBox){
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
	for(auto &synth: listVisible){
		//qDebug() << "Compare: analyzing from listVisible" << synth.name << QString::fromStdString(synth.type);
		synth.intersectArea = 0;
		synth.explained = false;
		synth.candidates.clear();
		std::vector<TCandidate> listCandidates;
		//It is compared with real objects
		for(auto &yolo: listYoloObjects){
			//If it is the same type and has not been assigned yet 
			if(synth.type == yolo.type and yolo.assigned == false){
				//qDebug() << "	analyzing from yoloObject:" << QString::fromStdString(yolo.type);
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
				//qDebug() << "		area" << area <<"dist" << error.manhattanLength() << "dist THRESHOLD" << rs.width()*2;
				// If the area is 0 there is no intersection
				// If the error is less than twice the width of the synthetic rectangle
				if(area > 0 or error.manhattanLength()< rs.width()*2){
					//A candidate is created and added to the list of candidates in an orderly manner according to the area and the error
					//The object will be placed earlier in the list the less difference there is with the original
					TCandidate tc = {area,error,&yolo};
					listCandidates.insert(std::upper_bound( listCandidates.begin(), listCandidates.end(),tc,
											  [](auto a, auto b) { return (a.area > b.area) or ((a.area==b.area) and (a.error.manhattanLength() < b.error.manhattanLength()));}), 
											  tc);
					//qDebug() << "		explain candidate for" << synth.name;
				}
			}
		}
		
		// If there are candidates, the first one is taken, assigned and the synthetic object is marked as explained
		if(listCandidates.empty() == false){
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
		if(y.assigned == false){
			TObject n;
			n.type = y.type;
			// get 3D pose from RGBD
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
		//if the synthetic object is explained, only its position in the innermodel is updated
		if(synth.explained){
			QPoint error = synth.error;
			//qDebug() << "	correcting" << synth.name  << "error" << error;
			InnerModelTransform *t = innermodel->getTransform(synth.name);
			// We assume that the cup does not vary in Y because it is on the table !!!
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
			for(uint i = 0; i<listObjects.size(); i++){
				if(n.idx == listObjects[i].idx){
					listObjects.erase(listObjects.begin()+i);
					ids[n.name.at(5).digitValue()] = 0;
				}
			}
		}
	}
	
	//qDebug() << "Stress: size of newCandidates" << listCreate.size();
	//New objects are added
	if(listObjects.size() < MAX_OBJECTS)
	  for(auto &n: listCreate)
	  {
		  if(n.type != "cup")  //Only cups for now
			  continue;
		  //qDebug() << "	new object candidate: " << QString::fromStdString(n.type);
		  //countertopA is the table
		  QVec ot = innermodel->transform("countertopA", n.pose, "rgbd");
		  // create a new unused name
		  QString name = QString::fromStdString(n.type);
		  //New object is created
		  TObject to;
		  
		  to.type = "cup";
		  //TObjects::iterator maxIdx =  std::max_element(listObjects.begin(), listObjects.end(), [](TObject a, TObject b){ return a.idx > b.idx;});
		  to.idx = getId();
		  to.name = "taza_" + QString::number(to.idx);
		  to.explained = false;
		  to.prob = 100;
		  to.time.start();
		  //qDebug() << "	new name" << to.name << "at" << ot.x() << ot.y() << ot.z();
		  //It is added to the innermodel
		  try{ /*InnerModelTransform *obj =*/innermodel->newTransform(to.name, "", innermodel->getNode("countertopA"), ot.x(), ot.y(), ot.z(), 0, 0, 0);}
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
		  for(auto i: to.bb){
			  
			  QVec res = c->project(innermodel->transform("rgbd", i, to.name));
			  if(res.x() >= 640 or res.x() <= 0 or res.y() <= 0 or res.y() >= 480 or std::isnan(res.x()) or std::isnan(res.y())){// Control between 0 and 640 res
				  valid = false;
				  ids[to.idx] = 0; //Revisar problemas con nombres. Se llena el vector al acceder a getID
				  try{innermodel->removeNode(to.name);}
				  catch(...){qDebug() << "Trying to remove non-existent node";}
				  goto end; //get out the loop
			  }
		  }
		  end:
		  if(valid == true){
			  //qDebug()<<"Valid"<<to.name;
			  listObjects.push_back(to);
			  listVisible.push_back(to); //The object is added
		  }
	  }
	  
	 qDebug()<<"listObjects";
	 for(auto &o : listObjects)
	   qDebug() << o.name;
	 
	 qDebug()<<"listVisible";
	 for(auto &o : listVisible)
	   qDebug() << o.name;
	 
	setState(States::Predict);
}

void SpecificWorker::setDefaultHeadPosition() {
    // Set initial position
    
    RoboCompJointMotor::MotorGoalPosition head_pitch_joint, head_yaw_joint;
    
    head_pitch_joint.name = "head_pitch_joint";
	head_pitch_joint.position = 0.9;
	
	head_yaw_joint.name = "head_yaw_joint";
	head_yaw_joint.position = 0.0;

	try{
		jointmotor_proxy->setPosition(head_pitch_joint);
		jointmotor_proxy->setPosition(head_yaw_joint);
	}catch(const Ice::Exception &e){
		qDebug() << "Init joints exception";
	}
}

// NO fUNCIONA BIEN
int SpecificWorker::getId(){
	uint j;
	for(j = 1; j<10; j++){
		if(ids[j] == 0){
			ids[j] = 1;
			return (j);
		}
	}
	return -1;
}

void SpecificWorker::checkTime()
{
	moved = false;
	for(auto &i: listObjects){
		//qDebug() <<"Tiempo"<< i.name << i.time.elapsed();
		if (i.time.elapsed() > 15000 and moved == false){
			moved = true;
			
			i.time.restart();
// 			for(auto &j: listVisible)
// 			  	j.time.restart();
			
			for(auto &j: listObjects)
			  	j.time.restart();
			
			RoboCompJointMotor::MotorGoalPosition head_yaw_joint, head_pitch_joint;

			head_yaw_joint.name = "head_yaw_joint"; //side to side
			head_pitch_joint.name = "head_pitch_joint"; //up and down
			
			QVec res = innermodel->transform("rgbd", i.bb.at(0), i.name);
			
			QPoint center = QPoint(170, 240);
			
			head_yaw_joint.position = atan2(res.x() - center.x(), res.z());
			head_pitch_joint.position = atan2(res.z(), center.y() - res.y());
			//qDebug()<<"Real"<< head_yaw_joint.position << head_pitch_joint.position;
			try{
				jointmotor_proxy->setPosition(head_yaw_joint);
				jointmotor_proxy->setPosition(head_pitch_joint);
				sleep(2);
			}catch(const Ice::Exception &e){
				std::cout << e <<endl;
			}
			//cin.get();
		}
	}
}


void SpecificWorker::setState(States s)
{
	static bool first = true;
	if (s == state and not first)
		return;
	first = false;
	state = s;

// 	if (state == States::Training)
// 	{
// 		groupBox->show();
// 	}
// 	else
// 	{
// 		groupBox->hide();
// 	}
}

bool SpecificWorker::imageChanges()
{
	if(matIsEqual(rgb_image,last_rgb_image))
		return false;
// 	else if(pointCloudIsEqual())
// 		return false;
	return true;
}

bool SpecificWorker::matIsEqual(const cv::Mat Mat1, const cv::Mat Mat2)
{
	if ( Mat1.size != Mat2.size)
		return false;
	uint32_t v = 0;
	for (int r=0; r< Mat1.rows; r++)
	{
		for (int c=0; c< Mat1.cols; c++)
		{
			for (int x = 0; x<3; x++)
			{
				uint32_t k = abs(Mat1.at<cv::Vec3b>(r, c)[x] - Mat2.at<cv::Vec3b>(r, c)[x]);
				if (k > 5)
				{
					v += k;
				}
			}
		}
	}
	printf("(%dx%d) ->%d\n", Mat1.rows, Mat1.cols, v);
	if(v > 1000000)
		return false;
	return true;
}

void SpecificWorker::updateinner()
{
	try
	{
		RoboCompGenericBase::TBaseState bState;
		omnirobot_proxy->getBaseState(bState);
		innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
	}
	catch(const Ice::Exception &e)
	{
		std::cout << e << std::endl;
	}
	
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
	catch(...)
	{
		if (test)
		{
			innermodel->updateJointValue(QString::fromStdString("head_pitch_joint"),0.710744);
			innermodel->updateJointValue(QString::fromStdString("head_yaw_joint"),0.0102265);
		}
		else
			qFatal("Can't access motors");
	}
}


void SpecificWorker::updatergbd(const RoboCompRGBD::ColorSeq &rgbMatrix, const RoboCompJointMotor::MotorStateMap &h, const RoboCompGenericBase::TBaseState &b)
{
	if(test)
	{
		rgb_image = cv::imread(image_path);
		if(! rgb_image.data )
			cout <<  "Could not open or find the image rgb.png" << std::endl ;
	}
	else
	{
		////qDebug() << __FUNCTION__ << rgbMatrix.size();
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
	}

	cv::Mat dest;
	cv::cvtColor(rgb_image, dest,CV_BGR2RGB);
	
	//ÑAPA
	
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

////////////////////////////////////////////
/// SUBSCRIPTION for interface AprilTags
////////////////////////////////////////////

void SpecificWorker::newAprilTagAndPose(const tagsList &tags, const RoboCompGenericBase::TBaseState &bState, const RoboCompJointMotor::MotorStateMap &hState)
{

}

void SpecificWorker::newAprilTag(const tagsList &tags)
{
	QMutexLocker locker(&april_mutex);
	if(tags.size()==0)
		return;
	////qDebug()<<"Found tags :"<<tags.size();
	this->tags = tags;   
}

//////////////////////////////////////////////////////////////////
//// PRIVATE METHODS 
/////////////////////////////////////////////////////////////////

//////////////////////////////////77 PABLO


	//if several minima, sort by distance
				//std::vector<std::pair<float,QPoint>> matches;
				//std::copy_if(synth.candidates.begin(), synth.candidates.end(), std::back_inserter(matches), 
				//			 [synth](auto v) { return v.first == synth.candidates.front().first;});
				
					
						
// 					{
// 						//qDebug() << "	Compare: explaining " << synth.name;
// 						//compute intersection percentage between synthetic and real
// 						QRect rs(QPoint(synth.box.x,synth.box.y),QPoint(synth.box.w,synth.box.h));
// 						QRect i = rs.intersected(r);
// 						synth.intersectArea = (float)(i.width() * i.height()) / std::min(rs.width() * rs.height(), r.width() * r.height());
// 						//qDebug() << "	Compare: area" << synth.intersectArea;
// 						synth.error = r.center() - rs.center();
// 						
// 						if(synth.intersectArea > 0 and synth.intersectArea <=1)
// 						{
// 							//qDebug() << "	Compare: intersecting" << synth.intersectArea;
// 							synth.explained = true;
// 						}
// 						else if(synth.intersectArea == 0 and synth.error.manhattanLength() < rs.width() )
// 						{
// 							//qDebug() << "	Compare: not intersecting but close" << synth.intersectArea;
// 							synth.explained = true;
// 						}
// 						else //too far
// 							synth.explained = false;
// 					}
// 					else	// potential new object
// 					{
// 						//qDebug() << "	Compare: potential new object";
// 						TObject n;
// 						n.type = yolo.label;
// 						
// 						// get 3D pose from RGBD
// 						int idx = r.center().x()*640 + r.center().y();
// 						RoboCompRGBD::PointXYZ p = pointMatrix[idx];
// 						n.pose = QVec::vec6(p.x, p.y, p.z, 0, 0, 0);
// 						newCandidates.push_back(n);
// 					}

