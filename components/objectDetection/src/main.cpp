/*
 *    Copyright (C) 2017 by YOUR NAME HERE
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


/** \mainpage RoboComp::objectDetectionComp
 *
 * \section intro_sec Introduction
 *
 * The objectDetectionComp component...
 *
 * \section interface_sec Interface
 *
 * interface...
 *
 * \section install_sec Installation
 *
 * \subsection install1_ssec Software depencences
 * ...
 *
 * \subsection install2_ssec Compile and install
 * cd objectDetectionComp
 * <br>
 * cmake . && make
 * <br>
 * To install:
 * <br>
 * sudo make install
 *
 * \section guide_sec User guide
 *
 * \subsection config_ssec Configuration file
 *
 * <p>
 * The configuration file etc/config...
 * </p>
 *
 * \subsection execution_ssec Execution
 *
 * Just: "${PATH_TO_BINARY}/objectDetectionComp --Ice.Config=${PATH_TO_CONFIG_FILE}"
 *
 * \subsection running_ssec Once running
 *
 * ...
 *
 */
#include <signal.h>

// QT includes
#include <QtCore>
#include <QtGui>

// ICE includes
#include <Ice/Ice.h>
#include <IceStorm/IceStorm.h>
#include <Ice/Application.h>

#include <rapplication/rapplication.h>
#include <sigwatch/sigwatch.h>
#include <qlog/qlog.h>

#include "config.h"
#include "genericmonitor.h"
#include "genericworker.h"
#include "specificworker.h"
#include "specificmonitor.h"
#include "commonbehaviorI.h"

#include <objectdetectionI.h>
#include <apriltagsI.h>

#include <ObjectDetection.h>
#include <AprilTags.h>
#include <GenericBase.h>
#include <JointMotor.h>
#include <RGBD.h>
#include <JointMotor.h>
#include <GenericBase.h>
#include <JointMotor.h>
#include <YoloServer.h>
#include <OmniRobot.h>
#include <GenericBase.h>


// User includes here

// Namespaces
using namespace std;
using namespace RoboCompCommonBehavior;

class objectDetectionComp : public RoboComp::Application
{
public:
	objectDetectionComp (QString prfx) { prefix = prfx.toStdString(); }
private:
	void initialize();
	std::string prefix;
	MapPrx mprx;

public:
	virtual int run(int, char*[]);
};

void ::objectDetectionComp::initialize()
{
	// Config file properties read example
	// configGetString( PROPERTY_NAME_1, property1_holder, PROPERTY_1_DEFAULT_VALUE );
	// configGetInt( PROPERTY_NAME_2, property1_holder, PROPERTY_2_DEFAULT_VALUE );
}

int ::objectDetectionComp::run(int argc, char* argv[])
{
#ifdef USE_QTGUI
	QApplication a(argc, argv);  // GUI application
#else
	QCoreApplication a(argc, argv);  // NON-GUI application
#endif


	sigset_t sigs;
	sigemptyset(&sigs);
	sigaddset(&sigs, SIGHUP);
	sigaddset(&sigs, SIGINT);
	sigaddset(&sigs, SIGTERM);
	sigprocmask(SIG_UNBLOCK, &sigs, 0);

	UnixSignalWatcher sigwatch;
	sigwatch.watchForSignal(SIGINT);
	sigwatch.watchForSignal(SIGTERM);
	QObject::connect(&sigwatch, SIGNAL(unixSignal(int)), &a, SLOT(quit()));

	int status=EXIT_SUCCESS;

	YoloServerPrx yoloserver_proxy;
	OmniRobotPrx omnirobot_proxy;
	JointMotorPrx jointmotor_proxy;
	RGBDPrx rgbd_proxy;

	string proxy, tmp;
	initialize();


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "YoloServerProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy YoloServerProxy\n";
		}
		yoloserver_proxy = YoloServerPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("YoloServerProxy initialized Ok!");
	mprx["YoloServerProxy"] = (::IceProxy::Ice::Object*)(&yoloserver_proxy);//Remote server proxy creation example


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "OmniRobotProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy OmniRobotProxy\n";
		}
		omnirobot_proxy = OmniRobotPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("OmniRobotProxy initialized Ok!");
	mprx["OmniRobotProxy"] = (::IceProxy::Ice::Object*)(&omnirobot_proxy);//Remote server proxy creation example


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "JointMotorProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy JointMotorProxy\n";
		}
		jointmotor_proxy = JointMotorPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("JointMotorProxy initialized Ok!");
	mprx["JointMotorProxy"] = (::IceProxy::Ice::Object*)(&jointmotor_proxy);//Remote server proxy creation example


	try
	{
		if (not GenericMonitor::configGetString(communicator(), prefix, "RGBDProxy", proxy, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy RGBDProxy\n";
		}
		rgbd_proxy = RGBDPrx::uncheckedCast( communicator()->stringToProxy( proxy ) );
	}
	catch(const Ice::Exception& ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: " << ex;
		return EXIT_FAILURE;
	}
	rInfo("RGBDProxy initialized Ok!");
	mprx["RGBDProxy"] = (::IceProxy::Ice::Object*)(&rgbd_proxy);//Remote server proxy creation example

	IceStorm::TopicManagerPrx topicManager;
	try
	{
		topicManager = IceStorm::TopicManagerPrx::checkedCast(communicator()->propertyToProxy("TopicManager.Proxy"));
	}
	catch (const Ice::Exception &ex)
	{
		cout << "[" << PROGRAM_NAME << "]: Exception: STORM not running: " << ex << endl;
		return EXIT_FAILURE;
	}


	SpecificWorker *worker = new SpecificWorker(mprx);
	//Monitor thread
	SpecificMonitor *monitor = new SpecificMonitor(worker,communicator());
	QObject::connect(monitor, SIGNAL(kill()), &a, SLOT(quit()));
	QObject::connect(worker, SIGNAL(kill()), &a, SLOT(quit()));
	monitor->start();

	if ( !monitor->isRunning() )
		return status;

	while (!monitor->ready)
	{
		usleep(10000);
	}

	try
	{
		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "CommonBehavior.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy CommonBehavior\n";
		}
		Ice::ObjectAdapterPtr adapterCommonBehavior = communicator()->createObjectAdapterWithEndpoints("commonbehavior", tmp);
		CommonBehaviorI *commonbehaviorI = new CommonBehaviorI(monitor );
		adapterCommonBehavior->add(commonbehaviorI, communicator()->stringToIdentity("commonbehavior"));
		adapterCommonBehavior->activate();




		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "ObjectDetection.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy ObjectDetection";
		}
		Ice::ObjectAdapterPtr adapterObjectDetection = communicator()->createObjectAdapterWithEndpoints("ObjectDetection", tmp);
		ObjectDetectionI *objectdetection = new ObjectDetectionI(worker);
		adapterObjectDetection->add(objectdetection, communicator()->stringToIdentity("objectdetection"));
		adapterObjectDetection->activate();
		cout << "[" << PROGRAM_NAME << "]: ObjectDetection adapter created in port " << tmp << endl;





		// Server adapter creation and publication
		if (not GenericMonitor::configGetString(communicator(), prefix, "AprilTagsTopic.Endpoints", tmp, ""))
		{
			cout << "[" << PROGRAM_NAME << "]: Can't read configuration for proxy AprilTagsProxy";
		}
		Ice::ObjectAdapterPtr AprilTags_adapter = communicator()->createObjectAdapterWithEndpoints("apriltags", tmp);
		AprilTagsPtr apriltagsI_ = new AprilTagsI(worker);
		Ice::ObjectPrx apriltags = AprilTags_adapter->addWithUUID(apriltagsI_)->ice_oneway();
		IceStorm::TopicPrx apriltags_topic;
		if(!apriltags_topic){
		try {
			apriltags_topic = topicManager->create("AprilTags");
		}
		catch (const IceStorm::TopicExists&) {
		//Another client created the topic
		try{
			apriltags_topic = topicManager->retrieve("AprilTags");
		}
		catch(const IceStorm::NoSuchTopic&)
		{
			//Error. Topic does not exist
			}
		}
		IceStorm::QoS qos;
		apriltags_topic->subscribeAndGetPublisher(qos, apriltags);
		}
		AprilTags_adapter->activate();

		// Server adapter creation and publication
		cout << SERVER_FULL_NAME " started" << endl;

		// User defined QtGui elements ( main window, dialogs, etc )

#ifdef USE_QTGUI
		//ignoreInterrupt(); // Uncomment if you want the component to ignore console SIGINT signal (ctrl+c).
		a.setQuitOnLastWindowClosed( true );
#endif
		// Run QT Application Event Loop
		a.exec();

		std::cout << "Unsubscribing topic: apriltags " <<std::endl;
		apriltags_topic->unsubscribe( apriltags );

		status = EXIT_SUCCESS;
	}
	catch(const Ice::Exception& ex)
	{
		status = EXIT_FAILURE;

		cout << "[" << PROGRAM_NAME << "]: Exception raised on main thread: " << endl;
		cout << ex;

#ifdef USE_QTGUI
		a.quit();
#endif
		monitor->exit(0);
}

	return status;
}

int main(int argc, char* argv[])
{
	string arg;

	// Set config file
	std::string configFile = "config";
	if (argc > 1)
	{
		std::string initIC("--Ice.Config=");
		size_t pos = std::string(argv[1]).find(initIC);
		if (pos == 0)
		{
			configFile = std::string(argv[1]+initIC.size());
		}
		else
		{
			configFile = std::string(argv[1]);
		}
	}

	// Search in argument list for --prefix= argument (if exist)
	QString prefix("");
	QString prfx = QString("--prefix=");
	for (int i = 2; i < argc; ++i)
	{
		arg = argv[i];
		if (arg.find(prfx.toStdString(), 0) == 0)
		{
			prefix = QString::fromStdString(arg).remove(0, prfx.size());
			if (prefix.size()>0)
				prefix += QString(".");
			printf("Configuration prefix: <%s>\n", prefix.toStdString().c_str());
		}
	}
	::objectDetectionComp app(prefix);

	return app.main(argc, argv, configFile.c_str());
}
