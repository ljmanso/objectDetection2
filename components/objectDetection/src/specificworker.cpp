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

	std::shared_ptr<InnerModel> innerModelS = std::make_shared<InnerModel>(params[name+".innermodel"].value);
    innermodel = innerModelS.get();

	
	innerViewer = new InnerViewer(innerModelS);
	//innerViewer->start();

    id_robot=QString::fromStdString(params[name+".id_robot"].value);
    id_camera=QString::fromStdString(params[name+".id_camera"].value);
    id_camera_transform=QString::fromStdString(params[name+".id_camera_transform"].value);

	// Start map struct
	#if FCL_SUPPORT==1
		
		auto table = innermodel->getNode<InnerModelNode>("countertopA_mesh");
		QMat r1q = innermodel->getRotationMatrixTo("root", "countertopA");
		fcl::Matrix3f R1(r1q(0,0), r1q(0,1), r1q(0,2), r1q(1,0), r1q(1,1), r1q(1,2), r1q(2,0), r1q(2,1), r1q(2,2));
		QVec t1v = innermodel->getTranslationVectorTo("root", "countertopA");
		fcl::Vec3f T1(t1v(0), t1v(1), t1v(2));
	
		table->collisionObject->setTransform(R1, T1);
		auto box = table->collisionObject->getAABB();
		
		//At this world:
		// - x = width
		// - y = height
		// - z = depth
		int width = box.width();
		int depth = box.depth();

		Table t;
		// Init map

		for(int i=-depth/2; i<depth/2; i+=CELL_WIDTH)
		{
			for(int j=-width/2; j<width/2; j+=CELL_HEIGHT)
			{
				QVec vector = QVec::vec3(j, 0, i);
				QVec res = innermodel->transform("world", vector, "countertopA");
				t.tableMap.emplace(Key(j,i),Value {res.x(), res.y(), res.z(), 0});
			}
		}
		
		t.id = 0;
		t.pose = t1v;
		t.temperature = -TEMP_TABLE;
		t.name = "countertopA";
		tables.push_back(t);

		auto table2 = innermodel->getNode<InnerModelNode>("countertopB_mesh");
		QMat r2q = innermodel->getRotationMatrixTo("root", "countertopB");
		fcl::Matrix3f R2(r2q(0,0), r2q(0,1), r2q(0,2), r2q(1,0), r2q(1,1), r2q(1,2), r2q(2,0), r2q(2,1), r2q(2,2));
		QVec t2v = innermodel->getTranslationVectorTo("root", "countertopB");
		fcl::Vec3f T2(t2v(0), t2v(1), t2v(2));
	
		table2->collisionObject->setTransform(R2, T2);
		auto box2 = table2->collisionObject->getAABB();
		
		//At this world:
		// - x = width
		// - y = height
		// - z = depth
		int width2 = box2.width();
		int depth2 = box2.depth();

		Table t1;
		// Init map
		for(int i=-depth2/2; i<depth2/2; i+=CELL_WIDTH)
		{
			for(int j=-width2/2; j<width2/2; j+=CELL_HEIGHT)
			{
				QVec vector2 = QVec::vec3(j, 0, i);
				QVec res2 = innermodel->transform("world", vector2, "countertopB");
				t1.tableMap.emplace(Key(j,i),Value {res2.x(), res2.y(), res2.z(), 0});
			}
		}
		
		t1.id = 1;
		t1.pose = t2v;
		t1.temperature = 0;
		t1.name = "countertopB";
		tables.push_back(t1);

		
		for(uint a=0; a<100; a++)
			ids[a] = 0;

		TObject taza1;
		taza1.name = "taza_0";
		taza1.type = "cup";
		taza1.idx = getId();
		taza1.explained = false;
		taza1.bb.push_back(QVec::vec3(40,0,40));
		taza1.bb.push_back(QVec::vec3(-40,0,40));
		taza1.bb.push_back(QVec::vec3(40,0,-40));
		taza1.bb.push_back(QVec::vec3(-40,-0,-40));
		taza1.bb.push_back(QVec::vec3(40,80,40));
		taza1.bb.push_back(QVec::vec3(-40,80,40));
		taza1.bb.push_back(QVec::vec3(40,80,-40));
		taza1.bb.push_back(QVec::vec3(-40,80,-40));
		taza1.pose = innermodel->transform("world", QVec::vec3(0,0,0), "taza_0");
		taza1.pose.print("taza0");
		tables[0].listObjects.push_back(taza1);

		TObject taza2;
		taza2.name = "taza_1";
		taza2.type = "cup";
		taza2.idx = getId();
		taza2.explained = false;
		taza2.bb.push_back(QVec::vec3(40,0,40));
		taza2.bb.push_back(QVec::vec3(-40,0,40));
		taza2.bb.push_back(QVec::vec3(40,0,-40));
		taza2.bb.push_back(QVec::vec3(-40,-0,-40));
		taza2.bb.push_back(QVec::vec3(40,80,40));
		taza2.bb.push_back(QVec::vec3(-40,80,40));
		taza2.bb.push_back(QVec::vec3(40,80,-40));
		taza2.bb.push_back(QVec::vec3(-40,80,-40));
		taza2.pose = innermodel->transform("world", QVec::vec3(0,0,0), "taza_1");
		taza2.pose.print("taza1");
		tables[0].listObjects.push_back(taza2);
		
		
		TObject taza3;
		taza3.name = "taza_2";
		taza3.type = "cup";
		taza3.idx = getId();
		taza3.explained = false;
		taza3.bb.push_back(QVec::vec3(40,0,40));
		taza3.bb.push_back(QVec::vec3(-40,0,40));
		taza3.bb.push_back(QVec::vec3(40,0,-40));
		taza3.bb.push_back(QVec::vec3(-40,-0,-40));
		taza3.bb.push_back(QVec::vec3(40,80,40));
		taza3.bb.push_back(QVec::vec3(-40,80,40));
		taza3.bb.push_back(QVec::vec3(40,80,-40));
		taza3.bb.push_back(QVec::vec3(-40,80,-40));
		taza3.pose = innermodel->transform("world", QVec::vec3(0,0,0), "taza_2");
		taza3.pose.print("taza2");
		tables[1].listObjects.push_back(taza3);
		
		//Table in which this is going to start
		processTable = 0;
		
		posAttention = taza1.pose;
		
	#endif
		
    setDefaultHeadPosition();
    yawPosition = 0.0;

	setState(States::Predict);
    timer.start(50);
    return true;
}

void SpecificWorker::compute()
{
    qDebug() << "------------------------------------------------------------";
    getRgbd();
    updateinner();
	innerViewer->run();
#ifdef USE_QTGUI
    try
    {
        updatergbd(rgbMatrix, h, b);
    }
    catch(...) {}
#endif
    getMotorState();

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
    tables[processTable].listVisible.clear();
    //For each synthetic object it is projected on the camera to create the labels
    for(auto &o: tables[processTable].listObjects)
    {
        o.projbb.clear();
        InnerModelCamera *c = innermodel->getCamera("rgbd");
        std::vector<QVec> bbInCam;
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
                o.box.x = 0.0;
                o.box.y = 0.0;
                o.box.w = 0.0;
                o.box.h = 0.0;
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
            [](const QVec& lhs, const QVec& rhs) {
                return lhs.x() < rhs.x();
            });
            // Sort the coordinates y
            auto yExtremes = std::minmax_element(bbInCam.begin(), bbInCam.end(),
            [](const QVec& lhs, const QVec& rhs) {
                return lhs.y() < rhs.y();
            });
            // Take the most separated ends to build the rectangle
            o.box.x = xExtremes.first->x();
            o.box.y = yExtremes.first->y();
            o.box.w = xExtremes.second->x();
            o.box.h = yExtremes.second->y();
            o.box.label = o.name.toStdString();
            o.box.prob = 100;
            o.projbb = bbInCam;
            tables[processTable].listVisible.push_back(o);
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
    } catch(const Ice::Exception &e) {
        std::cout << e << std::endl;
    }
}

void SpecificWorker::yoloWait()
{
    try
    {
        //Get the result of the previous request
        yoloLabels = yoloserver_proxy->getData(yoloId);
        if(yoloLabels.isReady)
        {
            tables[processTable].listYoloObjects.clear();
            //A real object is created for each label and added to the list of yolo objects
            for(auto y: yoloLabels.lBox)
            {
				//If YOLO detects twice the same object
				bool twice = false;
				for(auto yo: tables[processTable].listYoloObjects)
				{
					if(abs(yo.box.x - y.x) < 30 && abs(yo.box.y - y.y) < 30)
						twice = true;
				}
				
				if(!twice && y.x>=0 && y.y>=0 && y.w>0 && y.h>0 && y.x<480 && y.y <640){
					TObject o;
					o.type = y.label;
					o.box.x = y.x;
					o.box.y = y.y;
					o.box.w = y.w;
					o.box.h = y.h;
					o.assigned = false;
					o.prob = y.prob;
					tables[processTable].listYoloObjects.push_back(o);
				}
            }
            setState(States::Compare);
        }
    } catch(const Ice::Exception &e) {
        std::cout << e << std::endl;
    }
}

void SpecificWorker::compare(RoboCompRGBD::PointSeq pointMatrix)
{
    // Check if the predicted labels are on sight
    tables[processTable].listCreate.clear();
    tables[processTable].listDelete.clear();
	
    //For each synthetic object
    for(auto &synth: tables[processTable].listVisible)
    {
        synth.intersectArea = 0;
        synth.explained = false;
        std::vector<TCandidate> listCandidates;
		
        //It is compared with real objects
        for(auto &yolo: tables[processTable].listYoloObjects)
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
                if(area > 0 or error.manhattanLength()< rs.width()*3)
                {
                    //A candidate is created and added to the list of candidates in an orderly manner according to the area and the error
                    //The object will be placed earlier in the list the less difference there is with the original
                    TCandidate tc = {area,error,&yolo};
                    listCandidates.insert(std::upper_bound( listCandidates.begin(), listCandidates.end(),tc,
                    [](auto a, auto b) {
                        return (a.area > b.area) or ((a.area==b.area) and (a.error.manhattanLength() < b.error.manhattanLength()));
                    }),
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
    for(auto &o : tables[processTable].listVisible)
        if(o.explained == false)
            tables[processTable].listDelete.push_back(o);

    //listCreate: objects to be created due to measurements not assigned to objects
    for(auto &y: tables[processTable].listYoloObjects)
        if(y.assigned == false)
        {
            TObject n;
            n.type = y.type;
            // Get 3D pose from RGBD
            QRect r(QPoint(y.box.x,y.box.y),QPoint(y.box.w,y.box.h));
            int idx = r.center().y()*640 + r.center().x();
            RoboCompRGBD::PointXYZ p = pointMatrix[idx];
            n.pose = QVec::vec6(p.x, p.y, p.z, 0, 0, 0);
            tables[processTable].listCreate.push_back(n);
        }
	
    setState(States::Stress);
}

void SpecificWorker::stress()
{
    qDebug() << "Stress: size of listObjects" << tables[processTable].listObjects.size() << "and listVisible" << tables[processTable].listVisible.size() << "table" << processTable;
	//qDebug()<< "listCreate size:" << tables[processTable].listCreate.size() << "listDelete size" << tables[processTable].listDelete.size();
    //The position of the object is corrected
    for(auto &synth: tables[processTable].listVisible)
    {
        //If the synthetic object is explained, only its position in the innermodel is updated
        if(synth.explained)
        {
            QPoint error = synth.error;
            InnerModelTransform *t = innermodel->getTransform(synth.name);
			float maxError = sqrt((error.y()*error.y()) + (error.x()*error.x()));
			if(maxError > 10.0)
			{
				//Warms the position on table where the object is
				updateTemperature(synth, ZERO_TEMP);
				
				// We assume that the cup does not vary in Y because it is on the table
				innermodel->updateTranslationValues(synth.name, t->getTr().x() + error.y(), t->getTr().y(), t->getTr().z() + error.x());
				InnerModelTransform *tn = innermodel->getTransform(synth.name);
				synth.pose = QVec::vec6(tn->getTr().x() , tn->getTr().y(), tn->getTr().z(), 0, 0, 0);
				//Cools the position on table where the object is
				updateTemperature(synth, MIN_TEMP_OBJ);
			}
        }
    }

    //A non-existent object is removed
    if (tables[processTable].listDelete.size() > 0)
    {
        for(auto &n: tables[processTable].listDelete)
        {
            if(n.type != "cup")  //Only cups for now
                continue;
            //qDebug() << "	delete object " << n.name <<n.idx;

            //It is removed from the innermodel
            try {
                innerViewer->ts_removeNode(n.name);
 				updateTemperature(n, ZERO_TEMP);
            }
            catch(QString s) {
                qDebug() << s;
            }

            //It is removed from the list of synthetic objects
            for(uint i = 0; i<tables[processTable].listObjects.size(); i++)
            {
                if(n.idx == tables[processTable].listObjects[i].idx)
                {
                    tables[processTable].listObjects.erase(tables[processTable].listObjects.begin()+i);
                    ids[n.idx] = 0;
                }
            }

            for(uint i = 0; i<tables[processTable].listVisible.size(); i++)
                if(n.idx == tables[processTable].listVisible[i].idx)
                    tables[processTable].listVisible.erase(tables[processTable].listVisible.begin()+i);
        }
    }

    //New objects are added
    if(tables[processTable].listObjects.size() < MAX_OBJECTS)
        for(auto &n: tables[processTable].listCreate)
        {
            if(n.type != "cup")  //Only cups for now
                continue;

            //processTable is the table
            QVec ot = innermodel->transform(tables[processTable].name, n.pose, "rgbd");

			// create a new unused name
			QString name = QString::fromStdString(n.type);
			//New object is created
			TObject to;

			to.type = "cup";
			to.idx = getId();
			to.name = "taza_" + QString::number(to.idx);
			to.explained = false;
			to.prob = 100;
			to.pose = QVec::vec6(ot.x(), ot.y(), ot.z(), 0, 0, 0);
			
			//It is added to the innermodel
			try {
				//qDebug()<<"		New object" << to.name << to.pose;
				innerViewer->ts_addTransform(to.name, tables[processTable].name, to.pose);
				innerViewer->addMesh_ignoreExisting(to.name + "_mesh", to.name, QVec::vec3(0,0,0), QVec::vec3(1.570796371,0,-1.570796371), "/home/robocomp/robocomp/files/osgModels/mobiliario/taza.osg", QVec::vec3(120,120,120));
			}
			catch(QString s) {
				qDebug() << s;
			}
			
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
					valid = false;
				
			}
			if(valid == true)
			{
				//Cools the position on table where the object is
				updateTemperature(to, MIN_TEMP_OBJ);
				
				tables[processTable].listObjects.push_back(to);
				tables[processTable].listVisible.push_back(to); //The object is added
			}else{
				try {
					innerViewer->ts_removeNode(to.name);
					ids[to.idx] = 0;
				}
				catch(...) {
					qDebug() << "		Trying to remove non-existent node" << to.name;
				}
			}
		}
	

    updateTableMap();
	
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
    head_pitch_joint.position = 0.0;

    head_yaw_joint.name = "head_yaw_joint";
    head_yaw_joint.position = 0.0;

    try
    {
        jointmotor_proxy->setPosition(head_pitch_joint);
        jointmotor_proxy->setPosition(head_yaw_joint);
    } catch(const Ice::Exception &e) {
        qDebug() << "Moving joints exception";
    }
}

void SpecificWorker::getRgbd()
{
    try
    {
        rgbd_proxy->getRGB(rgbMatrix,h,b);
        rgbd_proxy->getXYZ(pointMatrix,h,b);
    }
    catch(const Ice::Exception &e) {
        std::cout << e << std::endl;
        return;
    };

}

void SpecificWorker::updateinner()
{
    try
    {
        RoboCompGenericBase::TBaseState bState;
        omnirobot_proxy->getBaseState(bState);
        innermodel->updateTransformValues("robot", bState.x, 0, bState.z, 0, bState.alpha, 0);
    }
    catch(const Ice::Exception &e) {
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
    catch(...) {
        qFatal("Can't access motors");
    }
}

void SpecificWorker::updatergbd(const RoboCompRGBD::ColorSeq &rgbMatrix, const RoboCompJointMotor::MotorStateMap &h, const RoboCompGenericBase::TBaseState &b)
{
    yoloImage.data.resize(rgbMatrix.size()*3);
    yoloImage.w=640;
    yoloImage.h=480;
    int j=0;
    for(unsigned int i=0; i<rgbMatrix.size(); i++, j=i*3)
    {
        int row = (i/480), column = i-(row*640);
        rgb_image.at<cv::Vec3b>(row, column) = cv::Vec3b(rgbMatrix[i].blue, rgbMatrix[i].green, rgbMatrix[i].red);
        yoloImage.data[j] = rgbMatrix[i].blue;
        yoloImage.data[j+1] = rgbMatrix[i].green;
        yoloImage.data[j+2] = rgbMatrix[i].red;
    }

    cv::Mat dest, dest1;
    cv::cvtColor(rgb_image, dest1, CV_BGR2YCrCb);
    vector<cv::Mat> channels;
    cv::split(dest1,channels);
    cv::equalizeHist(channels[0], channels[0]);
    cv::merge(channels,dest1);
    cv::cvtColor(dest1,dest,CV_YCrCb2RGB);
	

    for(auto yolo: tables[processTable].listYoloObjects)
    {
        if( yolo.prob > 35)
        {
            CvPoint p1, p2, pt;
            p1.x = int(yolo.box.x);
            p1.y = int(yolo.box.y);
            p2.x = int(yolo.box.w);
            p2.y = int(yolo.box.h);
            pt.x = int(yolo.box.x);
            pt.y = int(yolo.box.y) + ((p2.y - p1.y) / 2);
            cv::rectangle(dest, p1, p2, cv::Scalar(0, 0, 255), 4);
            auto font = cv::FONT_HERSHEY_SIMPLEX;
            cv::putText(dest, yolo.type + " " + std::to_string(int(yolo.prob)) + "%", pt, font, 1, cv::Scalar(255, 255, 255), 1);
        }
    }

    for(auto o: tables[processTable].listObjects)
    {
        CvPoint p1, p2;
        p1.x = int(o.box.x);
        p1.y = int(o.box.y);
        p2.x = int(o.box.w);
        p2.y = int(o.box.h);
        cv::rectangle(dest, p1, p2, cv::Scalar(0, 255, 5), 4);
    }

    QImage image((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
    item_pixmap->setPixmap(QPixmap::fromImage(image));
    scene.update();
}

void SpecificWorker::getMotorState()
{
    try
    {
        RoboCompJointMotor::MotorState mstateMap = jointmotor_proxy->getMotorState("head_yaw_joint");
		RoboCompJointMotor::MotorState mstateMapP = jointmotor_proxy->getMotorState("head_pitch_joint");
		changeTable();
		findPointAttention();
		centerAttention();
		

        if(fabs(yawPosition - mstateMap.pos) > 0.05f || fabs(pitchPosition - mstateMapP.pos) > 0.05f)
            setState(States::Moving);

        yawPosition = mstateMap.pos;
		pitchPosition = mstateMapP.pos;
		innermodel->updateRotationValues("rgbd",mstateMapP.pos, yawPosition, 0);

    } catch(...) {
        qDebug()<<"Error motor access";
    }
}

void SpecificWorker::changeTable()
{
	qDebug() <<"Table " << processTable <<"  Temp: "<<tables[processTable].temperature;
	bool changed = false;
	for(uint i = 0; i< tables.size() && !changed; i++)
	{
		if(tables[i].temperature < -TEMP_TABLE){
			int min = std::numeric_limits<int>::max();
			int table = 0;
			for(uint j = 0; j < tables.size(); j++)
			{
				if(tables[j].temperature < min)
				{
					min = tables[j].temperature;
					table = j;
				}
			}
			qDebug()<<"CAMBIO MESA";
			processTable = table;
			changed = true;
		}
		
		if((uint)processTable == i)
			tables[i].temperature ++;
		else
			tables[i].temperature --;
	}
}

void SpecificWorker::findPointAttention()
{
	int minTemp = std::numeric_limits<int>::max();
	QVec coor;
	for ( auto cell = tables[processTable].tableMap.begin(); cell != tables[processTable].tableMap.end(); ++cell )
	{
		if(cell->second.temperature < minTemp ){
			minTemp = cell->second.temperature;
			coor = QVec::vec3(cell->second.x, cell->second.y, cell->second.z);
		}
	}
	if(minTemp < std::numeric_limits<int>::max())
	{
		qDebug()<<"Celda nueva"<<coor << "temp:" << minTemp;
		posAttention = coor;
	}
}

void SpecificWorker::centerAttention()
{
	InnerModelCamera *c = innermodel->getCamera("rgbd");
	QVec v = innermodel->transform("rgbd", posAttention, "world");
    QVec res = c->project(v);
	if(sqrt((res.x()-320)*(res.x()-320) + (res.y()-240)*(res.y()-240)) > DIST)
	{
		RoboCompJointMotor::MotorGoalPosition head_yaw_joint, head_pitch_joint;

		RoboCompJointMotor::MotorState mstateMap = jointmotor_proxy->getMotorState("head_yaw_joint");
		RoboCompJointMotor::MotorState mstateMap1 = jointmotor_proxy->getMotorState("head_pitch_joint");
		
		head_yaw_joint.name = "head_yaw_joint"; 	//side to side
		head_pitch_joint.name = "head_pitch_joint"; //up and down

		head_yaw_joint.position = mstateMap.pos + atan2(v.x() , v.z());
		head_pitch_joint.position = mstateMap1.pos - atan2(v.y(), v.z());
		
		qDebug()<< "Yaw pos" << mstateMap.pos << head_yaw_joint.position << "Pitch pos" << mstateMap1.pos << head_pitch_joint.position;
		
		try
		{
			jointmotor_proxy->setPosition(head_yaw_joint);
			jointmotor_proxy->setPosition(head_pitch_joint);
			if(1-abs(mstateMap.pos) > 0.01 || 1-abs(mstateMap1.pos) > 0.01)
				sleep(1); //wait for the engines
		} catch(const Ice::Exception &e) {
			std::cout << e <<endl;
		}
	}
}

int SpecificWorker::getId()
{
    uint j;
    for(j = 0; j<100; j++)
    {
        if(ids[j] == 0)
        {
            ids[j] = 1;
            return (j);
        }
    }
    return -1;
}

void SpecificWorker::updateTableMap()
{
    auto c = std::bind(&SpecificWorker::cool, this, std::placeholders::_1);
    std::for_each(tables[processTable].tableMap.begin(),tables[processTable].tableMap.end(), c);
}

void SpecificWorker::cool(std::pair<const Key, Value>& cell)
{
    for(auto synth: tables[processTable].listObjects)
    {
		QVec pose = synth.pose;
        QRect objProj(pose.z()-40, pose.x()-40, 80, 80);
		QRect cellRect(cell.first.z, cell.first.x, CELL_WIDTH, CELL_HEIGHT);
		
        QRect i = cellRect.intersected(objProj);
        float area = (float)(i.width() * i.height());
		// If the area is 0 there is no intersection
		if(cell.second.temperature > MIN_TEMP_CELL)
		{
			if(area > 0)
				cell.second.temperature -= 15;
			else
				cell.second.temperature -= 1;
		}
    }

    if(tables[processTable].listObjects.empty())
		cell.second.temperature -= 1;
	
	if(cell.second.temperature < MAX_TEMP)
	{
		// Warm
		InnerModelCamera *c = innermodel->getCamera("rgbd");
		QVec cellCoor = QVec::vec3(cell.first.x, 0, cell.first.z);
		QVec res = c->project(innermodel->transform("rgbd", cellCoor, tables[processTable].name));
		if(res.x() < 640 and res.x() > 0 and res.y() > 0 and res.y() < 480) { // Control between 0 and 640 res.x and control between 0 and 480 res.y
			if(res.x() <40 || res.x() >600)
				cell.second.temperature += 1;
			else if((res.x() > 40 && res.x() < 100) || (res.x() > 540 && res.x() < 600)){
				if(res.y() <40 || res.y() >440)
					cell.second.temperature +=1;
				else
					cell.second.temperature += 2;
			}else if((res.x() < 180 && res.x() > 100) || (res.x() > 460 && res.x() < 540)){
				if(res.y() <40 || res.y() >440)
					cell.second.temperature +=1;
				else if((res.y() > 40 && res.y() < 120) || (res.y() > 360 && res.y() < 440))
					cell.second.temperature += 2;
				else
					cell.second.temperature +=4;
			}else if((res.x() < 280 && res.x() > 180) || (res.x() > 360 && res.x() < 460)){
				if(res.y() <40 || res.y() >440)
					cell.second.temperature +=1;
				else if((res.y() > 40 && res.y() < 120) || (res.y() > 360 && res.y() < 440))
					cell.second.temperature += 2;
				else if((res.y() > 120 && res.y() < 180) || (res.y() > 300 && res.y() < 360))
					cell.second.temperature +=4;
				else
					cell.second.temperature += 8;
			}else if(res.x() > 280 && res.x() < 360){
				if(res.y() <40 || res.y() >440)
					cell.second.temperature +=1;
				else if((res.y() > 40 && res.y() < 120) || (res.y() > 360 && res.y() < 440))
					cell.second.temperature += 2;
				else if((res.y() > 120 && res.y() < 180) || (res.y() > 300 && res.y() < 360))
					cell.second.temperature +=4;
				else
					cell.second.temperature += 20;
			}
		}
	}
}

void SpecificWorker::updateTemperature(TObject o, int temp)
{	
	for ( auto cell = tables[processTable].tableMap.begin(); cell != tables[processTable].tableMap.end(); ++cell){
		// Find the projection
		QVec pose = o.pose;
        QRect objProj(pose.z()-40, pose.x()-40, 80, 80);
		QRect cellRect(cell->first.z, cell->first.x, CELL_WIDTH, CELL_HEIGHT);
		
        QRect i = cellRect.intersected(objProj);
        float area = (float)(i.width() * i.height());
		// If the area is 0 there is no intersection
        if(area > 0)
            cell->second.temperature = temp;
	}
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
