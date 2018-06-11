/*
 *    Copyright (C) 2006-2010 by RoboLab - University of Extremadura
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

#include "innerviewer.h"

InnerViewer::InnerViewer( const InnerPtr &innerModel_, const std::string &name_, uint period_) : period(period_)
{	
	QGLFormat fmt;
	fmt.setDoubleBuffer(true);
	QGLFormat::setDefaultFormat(fmt);
	tb = new osgGA::TrackballManipulator;
	osg::Vec3d eye(osg::Vec3(4000., 4000., 1000.));
	osg::Vec3d center(osg::Vec3(0., 0., -0.));
	osg::Vec3d up(osg::Vec3(0., -1., 0.));
	tb->setHomePosition(eye, center, up, true);
	tb->setByMatrix(osg::Matrixf::lookAt(eye, center, up));
	viewer.setCameraManipulator(tb);
	osgViewer::Viewer::ThreadingModel threadingModel = osgViewer::Viewer::AutomaticSelection;
	viewer.setThreadingModel(threadingModel);
	viewer.addEventHandler(new osgViewer::WindowSizeHandler);
	createWindow(viewer, name_);
 	root = new osg::Group();
	
	viewer.getLight()->setPosition(osg::Vec4(1,-1, 1, 0)); // make 4th coord 1 for point
	viewer.getLight()->setAmbient(osg::Vec4(0.2, 0.2, 0.2, 1.0));
	//viewer.getLight()->setDiffuse(osg::Vec4(0.7, 0.4, 0.6, 1.0));
	viewer.getLight()->setSpecular(osg::Vec4(1.0, 1.0, 1.0, 1.0));
	
	innerModel = innerModel_; //std::make_shared<InnerModel>(*innerModel_);
	innerModelViewer = new InnerModelViewer(innerModel.get(), "world", root, true);
	
 	viewer.setSceneData(root);
		
	//////////////////////////
	//RESTORE FORMER VIEW					QUEDA CAPTURAR EL EVENTO DE CIERRE DE LA VENTANA PARA GUARDAR LA MATRIZ ACTUAL
	/////////////////////////
 	settings = new QSettings("RoboComp", "InnerViewer");
	QString path(".");
	QStringList l = settings->value("matrix").toStringList();
	if (l.size() > 0)
	{
		osg::Matrixd m;
		for (int i=0; i<4; i++ )
			for (int j=0; j<4; j++ )
				m(i,j)=l.takeFirst().toDouble();
		tb->setByMatrix(m);
	}
		else
			innerModelViewer->setMainCamera(tb, InnerModelViewer::TOP_POV);
	 	
	viewer.realize();
}

void InnerViewer::run()
{
	while(true)
	{
		if(!stop.load())
		{
           // guard gl(mutex);
            innerModelViewer->update(); 
			viewer.frame();
			std::this_thread::sleep_for(std::chrono::microseconds(period));
		}
		else
			stopped.store(true);
	}
}

void InnerViewer::reloadInnerModel(const InnerPtr &other)
{	
	stop.store(true);
	while(stopped.load() != true);
	guard gl(mutex);
	innerModel.reset(other.get()->copy());

	root->removeChild(innerModelViewer);
	innerModelViewer = new InnerModelViewer(innerModel.get(), "root", root, true);

	stop.store(false);
	stopped.store(false);
}

void InnerViewer::updateTransformValues(const QString item, const QVec &pos, const QString &parent)
{
	innerModel->updateTransformValues(item, pos.x(), pos.y(), pos.z(), pos.rx(), pos.ry(), pos.rz(), parent);
}

//////////////////////////////////////////////////////
//// Non thread saffe API 
//////////////////////////////////////////////////////

void InnerViewer::removeNode(const QString &item_)
{	
	guard gl(mutex);
    //preconditions
	InnerModelNode *node = innerModel->getNode(item_);
	if (node == NULL)
		throw QString("InnerViewer::remove node: Can't remove not existing element " + item_);

	if (item_ == "root")
		throw QString("InnerViewer::remove node: Can't remove root lement " + item_);

	QStringList l;
	innerModelViewer->innerModel->getSubTree(node, &l);
	innerModelViewer->innerModel->removeSubTree(node, &l);

	/// Replicate InnerModel node removals in the InnerModelViewer tree. And in handlers Lists
	foreach(const QString &n, l)
	{
		/// Replicate mesh removals
		if (innerModelViewer->meshHash.contains(n))
		{
			while (innerModelViewer->meshHash[n].osgmeshPaths->getNumParents() > 0)
				innerModelViewer->meshHash[n].osgmeshPaths->getParent(0)->removeChild(innerModelViewer->meshHash[n].osgmeshPaths);
			while(innerModelViewer->meshHash[n].osgmeshes->getNumParents() > 0)
				innerModelViewer->meshHash[n].osgmeshes->getParent(0)->removeChild(innerModelViewer->meshHash[n].osgmeshes);
			while(innerModelViewer->meshHash[n].meshMts->getNumParents() > 0)
				innerModelViewer->meshHash[n].meshMts->getParent(0)->removeChild(innerModelViewer->meshHash[n].meshMts);
			innerModelViewer->meshHash.remove(n);
		}
		/// Replicate transform removals
		if (innerModelViewer->mts.contains(n))
		{
 			while (innerModelViewer->mts[n]->getNumParents() > 0)
				innerModelViewer->mts[n]->getParent(0)->removeChild(innerModelViewer->mts[n]);
 			innerModelViewer->mts.remove(n);
		}
		/// Replicate plane removals
		if (innerModelViewer->planeMts.contains(n))
		{
			while(innerModelViewer->planeMts[n]->getNumParents() > 0)
				((osg::Group *)(innerModelViewer->planeMts[n]->getParent(0)))->removeChild(innerModelViewer->planeMts[n]);
			innerModelViewer->planeMts.remove(n);
			innerModelViewer->planesHash.remove(n);
		}
	}
}

void InnerViewer::addTransform_ignoreExisting(const QString &item_, const QString &parent_, const QVec &pos)
{
	guard gl(mutex);
    //preconditions
	if(pos.size() != 6)
		throw QString("InnerViewer::addPlane_ignoreExisting: Position vector has not dimension 6");
	
	InnerModelNode *parent = innerModelViewer->innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addTransform_ignoreExisting: parent element node doesn't exist");

	if (innerModelViewer->innerModel->getNode(item_) != NULL)
		this->removeNode(item_);

	this->addTransform(item_, parent_, pos);
}

void InnerViewer::addTransform(const QString &item_, const QString &parent_,const QVec &pos)
{
	guard gl(mutex);
    //preconditions
	if(pos.size() != 6)
		throw QString("InnerViewer::addPlane_ignoreExisting: Position vector has not dimension 6");
	
	InnerModelNode *parent = innerModelViewer->innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addTransform: parent node doesn't exist");

	InnerModelNode *node = innerModelViewer->innerModel->getNode(item_);
	if (node != NULL)
		throw QString("InnerViewer::addTransform: item alreaddy exists");

	InnerModelTransform *tr;
	try
	{
		tr = innerModelViewer->innerModel->newTransform(item_, "static", parent, 0,0,0, 0,0,0);
		parent->addChild(tr);
		innerModelViewer->recursiveConstructor(tr, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
		innerModelViewer->innerModel->updateTransformValues(item_, pos.x(), pos.y(), pos.z(), pos.rx(), pos.ry(), pos.rz());
	}
	catch (QString err)
	{
		printf("%s:%s:%d: Exception: %s\n", __FILE__, __FUNCTION__, __LINE__, err.toStdString().c_str());
		throw err;
	}
}

void InnerViewer::drawLine(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, float length, float width, const QString &texture)
{
	guard gl(mutex);
    this->addPlane_ignoreExisting(item_, parent_, center, normal, texture, QVec::vec3(length, width, width));
}

void InnerViewer::addPlane_ignoreExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size)
{
	guard gl(mutex);
    InnerModelNode *parent = innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addPlane_ignoreExisting: parent element node doesn't exist " + item_);
	InnerModelPlane *plane = innerModelViewer->innerModel->newPlane(item_, parent, texture, size(0), size(1), size(2), 1, normal(0), normal(1), normal(2), center(0), center(1), center(2));
	parent->addChild(plane);
	innerModelViewer->recursiveConstructor(plane, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
}

void InnerViewer::addPlane_notExisting(const QString &item_, const QString &parent_, const QVec &center, const QVec &normal, const QString &texture, const QVec &size)
{
	guard gl(mutex);
    InnerModelNode *parent = innerModel->getNode(parent_);
	if (parent == NULL)
		throw QString("InnerViewer::addPlane_notExisting: parent node doesn't exist");
	InnerModelPlane *plane = innerModel->newPlane(item_, parent, texture, size(0), size(1), size(2), 1, normal(0), normal(1), normal(2), center(0), center(1), center(2));
	parent->addChild(plane);
	innerModelViewer->recursiveConstructor(plane, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
}

void InnerViewer::addMesh_ignoreExisting(const QString &item, const QString &base, const QVec &t, const QVec &r, const QString &path, const QVec &scale)
{
	guard gl(mutex);
    InnerModelTransform *parent = innerModel->getNode<InnerModelTransform>(base);
	if( parent == nullptr)
		return;
	
/*	if (innerModel->getNode<InnerModelNode>(item) != nullptr)
	{
		removeNode(item);
	}
*/
	InnerModelMesh *mesh = innerModel->newMesh(
		item,
		parent,
		path,
		scale(0), scale(1), scale(2),
		0,
		t(0), t(1), t(2),
		r(0), r(1), r(2));
	mesh->setScale(scale(0), scale(1), scale(2));
	parent->addChild(mesh);
	innerModelViewer->recursiveConstructor(mesh, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
}

bool InnerViewer::setScale(const QString &item, float scaleX, float scaleY, float scaleZ)
{
	guard gl(mutex);
    InnerModelMesh *aux = innerModel->getNode<InnerModelMesh>(item);
	if(aux == nullptr) 
		return false;
	aux->setScale(scaleX, scaleY, scaleZ);
	return true;
}

bool InnerViewer::addJoint(const QString &item, const QString &base, const QVec &t, const QVec &r, QString &axis)
{
	guard gl(mutex);
    if (axis == "")
		axis = "z";

	InnerModelTransform *parent = innerModel->getNode<InnerModelTransform>(base);
	if(parent == nullptr)
		return false;
	InnerModelJoint *jN = innerModel->newJoint(item,
					   parent,
					   0,0,0,
					   0,0,0,
				           t(0), t(1), t(2),
					   r(0), r(1), r(2),
					   -1000, 1000,
				           0,
				           axis.toStdString());
	parent->addChild(jN);
	innerModelViewer->recursiveConstructor(jN, innerModelViewer->mts[parent->id], innerModelViewer->mts, innerModelViewer->meshHash);
	return true;
}

bool InnerViewer::setPlaneTexture(const QString &item, const QString &texture)
{
	guard gl(mutex);
    InnerModelPlane *aux = innerModel->getNode<InnerModelPlane>(item);
	if(item == nullptr)
		return false;
	
	aux->texture = texture;
	bool constantColor = false;
	if (texture.size() == 7)
	{
		if (texture[0] == '#')
		{
			constantColor = true;
		}
	}
	if (not constantColor)
	{
	  osg::Image *image=NULL;
		image = osgDB::readImageFile(texture.toStdString());
		if (not image)
		{
			throw "Couldn't load texture.";
		}
		innerModelViewer->planesHash[aux->id]->image =image;
		innerModelViewer->planesHash[aux->id]->texture->setImage(image);
	}
	else
	{
		innerModelViewer->planesHash[aux->id]->planeDrawable->setColor(htmlStringToOsgVec4(texture));
	}
	return true;
}

void InnerViewer::drawLine2Points(const QString &name, const QString &parent, const QVec& p1, const QVec& p2, float width, const QString &texture)
{
	guard gl(mutex);
    QLine2D line( p1 , p2 );	
	float dl = (p1-p2).norm2();
	QVec center = p2 + ((p1 - p2)*(float)0.5);
	this->drawLine(name, parent, line.getNormalForOSGLineDraw(), center, dl, width, "#0000ff");
}

bool InnerViewer::removeObject(const QString &name)
{
	guard gl(mutex);
    if (innerModel->getNode<InnerModelNode>(name) != nullptr)
	{
		removeNode(name);
		return true;
	}
	else
	{
		qDebug() << __FUNCTION__ << "Object " << name << "does not exist. Could not be removed";
		return false;
	}
}

///////////////////////////////////////////////////////////////////////////////////77

// UNTIL we know how to capture the close window signal from OSG/X11
// 			osg::Matrixd m = tb->getMatrix();
// 			QString s="";
// 			QStringList l;
// 			for (int i=0; i<4; i++ )
// 				for (int j=0; j<4; j++ )
// 	 			l.append(s.number(m(i,j)));
// 			settings->setValue("matrix", l);
// 			settings->sync();

/////////////////////////////////////
/// Auxiliary methods
/////////////////////////////////////
void InnerViewer::createWindow(osgViewer::Viewer& viewer, const std::string &name)
{
	osg::GraphicsContext::WindowingSystemInterface* wsi = osg::GraphicsContext::getWindowingSystemInterface();
	if (!wsi)
	{
		osg::notify(osg::NOTICE)<<"Error, no WindowSystemInterface available, cannot create windows."<<std::endl;
		return;
	}

	unsigned int width, height;
	wsi->getScreenResolution(osg::GraphicsContext::ScreenIdentifier(0), width, height);

	osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
	traits->x = 0;
	traits->y = 0;
	traits->width = 700;
	traits->height = 700;
	traits->windowDecoration = true;
	traits->doubleBuffer = true;
	traits->sharedContext = 0;
	traits->windowName = "InnerModelViewer " + name;

	osg::ref_ptr<osg::GraphicsContext> gc = osg::GraphicsContext::createGraphicsContext(traits.get());
	if (gc.valid())
	{
		osg::notify(osg::INFO)<<"  GraphicsWindow has been created successfully."<<std::endl;

		// need to ensure that the window is cleared make sure that the complete window is set the correct colour
		// rather than just the parts of the window that are under the camera's viewports
		gc->setClearColor(osg::Vec4f(0.2f,0.2f,0.6f,1.0f));
		gc->setClearMask(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	}
	else
	{
		osg::notify(osg::NOTICE)<<"  GraphicsWindow has not been created successfully."<<std::endl;
	}
	unsigned int numCameras = 1;
	double aspectRatioScale = 1.0;///(double)numCameras;
	for(unsigned int i=0; i<numCameras;++i)
	{
		osg::ref_ptr<osg::Camera> camera = new osg::Camera;
		camera->setGraphicsContext(gc.get());
		camera->setViewport(new osg::Viewport((i*width)/numCameras,(i*height)/numCameras, width/numCameras, height/numCameras));
		GLenum buffer = traits->doubleBuffer ? GL_BACK : GL_FRONT;
		camera->setDrawBuffer(buffer);
		camera->setReadBuffer(buffer);

		viewer.addSlave(camera.get(), osg::Matrixd(), osg::Matrixd::scale(aspectRatioScale,1.0,1.0));
	}
}

